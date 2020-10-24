#include "Map.h"
#include "Feature.h"

#include "frontend/FullSystem.h"
#include "internal/GlobalCalib.h"
#include "internal/FrameHessian.h"
#include "internal/PointHessian.h"
#include "internal/PR.h"

#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_gauss_newton.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/core/optimization_algorithm_dogleg.h>
#include <g2o/solvers/linear_solver_eigen.h>
#include <g2o/core/robust_kernel_impl.h>

using namespace std;
using namespace ldso::internal;

namespace ldso {

    /**
     * 添加一个关键帧
     * */
    void Map::AddKeyFrame(shared_ptr<Frame> kf) {
        // 线程锁住
        unique_lock<mutex> mapLock(mapMutex);
        if (frames.find(kf) == frames.end())// 当前frames中没有该帧 
        {
            // 则插入该关键帧
            frames.insert(kf);
        }
    }
    // 回环之后调用，对所有帧进行回环后的一次优化
    void Map::lastOptimizeAllKFs() {
        LOG(INFO) << "Final pose graph optimization after odometry is finished.";

        {
            unique_lock<mutex> lock(mutexPoseGraph);
            if (poseGraphRunning) {
                LOG(FATAL) << "Should not be called while pose graph optimization is running";
            }
        }

        // no locking of mapMutex since we assume that odometry has finished
        framesOpti = frames;
        currentKF = *frames.rbegin();
        runPoseGraphOptimization();
    }
    // 优化所有的关键帧
    bool Map::OptimizeALLKFs() {
        {
            unique_lock<mutex> lock(mutexPoseGraph);
            // 上一次已经优化完成了
            if (poseGraphRunning)
                return false; // is already running ...
            // if not, starts it
            // 设置为新一次的优化开始了
            poseGraphRunning = true;
            // lock frames to prevent adding new kfs
            unique_lock<mutex> mapLock(mapMutex);
            // 优化的数据
            framesOpti = frames;
            currentKF = *frames.rbegin();
        }

        //  start the pose graph thread
        //  开始优化的线程
        thread th = thread(&Map::runPoseGraphOptimization, this);
        th.detach();    // it will set posegraphrunning to false when returns
        return true;
    }
    // 更新所有地图点，在确定当前帧的位姿后调用
    void Map::UpdateAllWorldPoints() {
        unique_lock<mutex> lock(mutexPoseGraph);
        for (shared_ptr<Frame> frame: frames) {
            for (auto &feat: frame->features) {// 遍历所有点
                if (feat->point) {
                    feat->point->ComputeWorldPos();// 计算点在世界坐标系下的坐标
                }
            }
        }
    }
    // 实际调用位姿优化的单独函数线程
    void Map::runPoseGraphOptimization() {

        LOG(INFO) << "start pose graph thread!" << endl;
        // Setup optimizer
        // 设置优化器
        g2o::SparseOptimizer optimizer;// 稀疏的求解器
        typedef BlockSolver<BlockSolverTraits<7, 3> > BlockSolverType;// 求解
        // 线性求解器
        BlockSolverType::LinearSolverType *linearSolver;
        linearSolver = new g2o::LinearSolverEigen<BlockSolverType::PoseMatrixType>();
        BlockSolverType *solver_ptr = new BlockSolverType(linearSolver);
        // g2o::OptimizationAlgorithmLevenberg *solver = new g2o::OptimizationAlgorithmLevenberg(solver_ptr);
        // 使用高斯牛顿法
        g2o::OptimizationAlgorithmGaussNewton *solver = new g2o::OptimizationAlgorithmGaussNewton(solver_ptr);
        // g2o::OptimizationAlgorithmDogleg *solver = new g2o::OptimizationAlgorithmDogleg(solver_ptr);
        optimizer.setAlgorithm(solver);
        // 不输出调试信息
        optimizer.setVerbose(false);

        // keyframes
        int maxKFid = 0;
        int cntEdgePR = 0;
        // 遍历所有需要优化的帧
        // 关键帧的位姿节点，使用sim3，而非SE3，优化节点类定义为 Sim3
        for (const shared_ptr<Frame> &fr: framesOpti) {

            // each kf has Sim3 pose
            // 当前关键帧的id
            int idKF = fr->kfId;
            if (idKF > maxKFid) {
                // 记录下帧的最大id
                maxKFid = idKF;
            }

            // P+R
            VertexSim3 *vSim3 = new VertexSim3();
            Sim3 Scw = fr->getPoseOpti();
            CHECK(Scw.scale() > 0);
            vSim3->setEstimate(Scw);
            vSim3->setId(idKF);
            // 添加需要优化的节点，vSim3
            optimizer.addVertex(vSim3);

            // fix the last one since we don't want to affect the frames in window
            if (fr == currentKF) {
                vSim3->setFixed(true);
            }

        }
        // 图模型中的边
        // edges
        for (const shared_ptr<Frame> &fr: framesOpti) {// 遍历所有关键帧
            unique_lock<mutex> lock(fr->mutexPoseRel);
            for (auto &rel: fr->poseRel) {// 遍历所有相关联的帧
                VertexSim3 *vPR1 = (VertexSim3 *) optimizer.vertex(fr->kfId);// 当前帧对应的节点 
                VertexSim3 *vPR2 = (VertexSim3 *) optimizer.vertex(rel.first->kfId);// 相关联帧对应的优化节点
                // new 一条新的边
                EdgeSim3 *edgePR = new EdgeSim3();
                // 确保节点不为空
                if (vPR1 == nullptr || vPR2 == nullptr)
                    continue;
                // 设置连接    
                edgePR->setVertex(0, vPR1);
                edgePR->setVertex(1, vPR2);
                // 设置测量值
                edgePR->setMeasurement(rel.second.Tcr);
                if (rel.second.isLoop)
                    edgePR->setInformation(rel.second.info /* *10 */);
                else
                    edgePR->setInformation(rel.second.info);
                // 添加边缘
                optimizer.addEdge(edgePR);
                cntEdgePR++;
            }
        }
        // 优化器初始化
        optimizer.initializeOptimization();
        // 优化器开始迭代计算，最大迭代次数25
        optimizer.optimize(25);

        // recover the pose and points estimation
        for (shared_ptr<Frame> frame: framesOpti) {// 遍历所有优化的关键帧
            // 获取优化结果
            VertexSim3 *vSim3 = (VertexSim3 *) optimizer.vertex(frame->kfId);
            Sim3 Scw = vSim3->estimate();
            CHECK(Scw.scale() > 0);
            // 设置对应帧的位置
            frame->setPoseOpti(Scw);
            // reset the map point world position because we've changed the keyframe pose
            for (auto &feat: frame->features) {
                if (feat->point) {// 遍历所有优化的点
                    feat->point->ComputeWorldPos();// 计算在世界坐标系下的位置
                }
            }
        }
        // 优化过程完全结束
        poseGraphRunning = false;
        // 记录当前帧id
        if (currentKF) {
            latestOptimizedKfId = currentKF->kfId;
        }
        // 更新gui可视化
        if (fullsystem) fullsystem->RefreshGUI();
    }

}
