#pragma once
#ifndef LDSO_MAP_H_
#define LDSO_MAP_H_

#include "NumTypes.h"
#include "Frame.h"
#include "Point.h"
#include "internal/CalibHessian.h"

#include <set>
#include <thread>
#include <mutex>

using namespace std;
using namespace ldso::internal;

namespace ldso {

    class FullSystem;

    /**
     * The global map contains all keyframes and map points, even if they are marginalized or outdated.
     * The map can be saved to and loaded from disk, if you wanna reuse it.
     *
     * The loop closing thread will call the optimize function if there is a consistent loop closure.
     * 全局的地图结构，包括了所有的关键帧和地图点
     */

    class Map {
    public:
        // 传递一个FullSystem 的指针进来
        Map(FullSystem *fs) : fullsystem(fs) {}

        /**
         * add a keyframe into the global map
         * 往全局地图中添加一个关键帧
         * @param kf
         */
        void AddKeyFrame(shared_ptr<Frame> kf);

        /**
         * optimize pose graph of all kfs
         * 优化所有的关键帧
         * this will start the pose graph optimization thread (usually takes several seconds to return in my machine)
         * @param allKFs
         * @return true if pose graph thread is started
         */
        bool OptimizeALLKFs();

        // optimize pose graph on all kfs after odometry loop is done
        // 当回环完成后的优化工作
        void lastOptimizeAllKFs();

        /// update the cached 3d position of all points.
        // 优化所有的点
        void UpdateAllWorldPoints();

        /**
         * get number of frames stored in global map
         * 获取当前关键帧的数量
         * @return
         */
        inline int NumFrames() const {
            return frames.size();
        }

        // is pose graph running?
        // 是否在运行优化
        bool Idle() {
            unique_lock<mutex> lock(mutexPoseGraph);
            return !poseGraphRunning;
        }
        // 获得所有的关键帧
        set<shared_ptr<Frame>, CmpFrameID> GetAllKFs() { return frames; }
        // 获取上一次优化关键帧的id
        unsigned long getLatestOptimizedKfId() const { return latestOptimizedKfId; }

    private:
        // the pose graph optimization thread
        // 位姿图优化线程函数
        void runPoseGraphOptimization();

        mutex mapMutex; // map mutex to protect its data
        // 所有的关键帧
        set<shared_ptr<Frame>, CmpFrameID> frames;      // all KFs by ID
        // 回环之后的
        set<shared_ptr<Frame>, CmpFrameID> framesOpti;  // KFs to be optimized
        // 当前的关键帧
        shared_ptr<Frame> currentKF = nullptr;

        // keyframe id of newest optimized keyframe frame
        // 最新的关键帧id
        unsigned long latestOptimizedKfId = 0;
        // 位姿图优化线程是否在运行
        bool poseGraphRunning = false;  // is pose graph running?
        mutex mutexPoseGraph;
        // 指针指向Fullsystem
        FullSystem *fullsystem = nullptr;
    };

}

#endif // LDSO_MAP_H_
