#pragma once
#ifndef LDSO_COARSE_INITIALIZER_H_
#define LDSO_COARSE_INITIALIZER_H_

#include "NumTypes.h"
#include "Settings.h"
#include "AffLight.h"
#include "internal/OptimizationBackend/MatrixAccumulators.h"

#include "Camera.h"
#include "Frame.h"
#include "Point.h"

using namespace ldso;
using namespace ldso::internal;

namespace ldso {

    /**
     * point structure used in coarse initializer
     */
    // 在粗糙的初始化时使用的点结构体
    struct Pnt {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // index in jacobian. never changes (actually, there is no reason why).
        // 点的坐标，对应jacobian的位置
        float u, v;

        // idepth / isgood / energy during optimization.
        float idepth;// 逆深度
        bool isGood;// 在新图像内，相机前，像素值有穷则好
        Vec2f energy;        // 残差的平方 (UenergyPhotometric, energyRegularizer)
        bool isGood_new;// 在新的一帧中是否为好的
        float idepth_new;// 在新的一帧中的逆深度
        Vec2f energy_new;// 在新的一帧中的残差

        float iR;       // 逆深度的期望值
        float iRSumNum; // 子点逆深度信息矩阵之和

        float lastHessian;// 逆深度的协方差
        float lastHessian_new;// 新一次迭代后的逆深度协方差

        // max stepsize for idepth (corresponding to max. movement in pixel-space).
        float maxstep;// 逆深度增加的最大步长

        // idx (x+y*w) of closest point one pyramid level above.
        int parent;// 上一层中该点的父节点
        float parentDist;// 上一层中与父节点的距离

        // idx (x+y*w) of up to 10 nearest points in pixel space.
        int neighbours[10];// 图像中距离该店最近的10个点
        float neighboursDist[10];// 最近十个点的距离

        float my_type;// 第零层提取是 1 2 4，其他是1 
        float outlierTH;// 外点阈值
    };

    /**
     * initializer for monocular slam
     * 单目slam的初始化
     */
    class CoarseInitializer {
    public:

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 构造函数，w h为图像的大小
        CoarseInitializer(int w, int h);
        // 析构函数
        ~CoarseInitializer();
        // 设置第一帧数据
        void setFirst(shared_ptr<CalibHessian> HCalib, shared_ptr<FrameHessian> newFrameHessian);
        // 第二帧的数据
        bool trackFrame(shared_ptr<FrameHessian> newFrameHessian);
        // 
        void calcTGrads(shared_ptr<FrameHessian> newFrameHessian);

        int frameID = -1;// 当前加入的帧数
        bool fixAffine = true;// 是否优化光度误差
        bool printDebug = false;// 是否输出调试信息

        Pnt *points[PYR_LEVELS];// 存储每一层图像金字塔内的点
        int numPoints[PYR_LEVELS];// 存储每一层图像金字塔内含有点的数量

        AffLight thisToNext_aff;// 当前帧到下一帧的光度仿射变换
        SE3 thisToNext;// 当前帧到下一帧的位姿变换


        shared_ptr<FrameHessian> firstFrame;// 第一帧的数据的指针
        shared_ptr<FrameHessian> newFrame;// 第二帧的数据指针
    private:
        // 金字塔各层对应的相机参数
        Mat33 K[PYR_LEVELS];
        Mat33 Ki[PYR_LEVELS];
        double fx[PYR_LEVELS];
        double fy[PYR_LEVELS];
        double fxi[PYR_LEVELS];
        double fyi[PYR_LEVELS];
        double cx[PYR_LEVELS];
        double cy[PYR_LEVELS];
        double cxi[PYR_LEVELS];
        double cyi[PYR_LEVELS];
        int w[PYR_LEVELS];
        int h[PYR_LEVELS];
        // 计算金字塔各层对应的相机参数，得到上面的所有参数
        void makeK(shared_ptr<CalibHessian> HCalib);

        bool snapped;// 是否尺度收敛
        int snappedAt;// 尺度收敛在第几帧

        // pyramid images & levels on all levels
        Eigen::Vector3f *dINew[PYR_LEVELS];
        Eigen::Vector3f *dIFist[PYR_LEVELS];
        // 对角矩阵 8x8
        Eigen::DiagonalMatrix<float, 8> wM;

        // temporary buffers for H and b.
        Vec10f *JbBuffer;            // 0-7: sum(dd * dp). 8: sum(res*dd). 9: 1/(1+sum(dd*dd))=inverse hessian entry.
        Vec10f *JbBuffer_new; // 迭代更新后的数据

        // 9维向量，乘积获得9*9矩阵
        Accumulator9 acc9;
        Accumulator9 acc9SC;

        Vec3f dGrads[PYR_LEVELS];

        float alphaK;
        float alphaW;
        float regWeight;
        float couplingWeight;

        Vec3f calcResAndGS(
                int lvl,
                Mat88f &H_out, Vec8f &b_out,
                Mat88f &H_out_sc, Vec8f &b_out_sc,
                const SE3 &refToNew, AffLight refToNew_aff,
                bool plot);

        Vec3f calcEC(int lvl); // returns OLD NERGY, NEW ENERGY, NUM TERMS.
        void optReg(int lvl);

        void propagateUp(int srcLvl);

        void propagateDown(int srcLvl);

        float rescale();

        void resetPoints(int lvl);

        void doStep(int lvl, float lambda, Vec8f inc);

        void applyStep(int lvl);

        void makeGradients(Eigen::Vector3f **data);

        void makeNN();
    };

    /**
     * minimal flann point cloud
     * 最小化 flann 点云
     */
    struct FLANNPointcloud {
        inline FLANNPointcloud() {
            num = 0;
            points = 0;
        }

        inline FLANNPointcloud(int n, Pnt *p) : num(n), points(p) {}

        int num;
        Pnt *points;

        inline size_t kdtree_get_point_count() const { return num; }

        inline float kdtree_distance(const float *p1, const size_t idx_p2, size_t /*size*/) const {
            const float d0 = p1[0] - points[idx_p2].u;
            const float d1 = p1[1] - points[idx_p2].v;
            return d0 * d0 + d1 * d1;
        }

        inline float kdtree_get_pt(const size_t idx, int dim) const {
            if (dim == 0) return points[idx].u;
            else return points[idx].v;
        }

        template<class BBOX>
        bool kdtree_get_bbox(BBOX & /* bb */) const { return false; }
    };
}

#endif // LDSO_COARSE_INITIALIZER_H_
