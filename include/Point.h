#pragma once

#include <memory>
#include <set>

#include "NumTypes.h"

using namespace std;

namespace ldso {

    struct Frame;
    struct Feature;

    namespace internal {
        class PointHessian;
    }

    /**
     * @brief Point represents a 3D map point.
     * New map points are generated from immature features in the activiate function of FullSystem
     * NOTE we current don't clean the outlier map points, do this if you really want
     * 3D地图中的点
     */
    struct Point {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         * point can only be created with a host feature
         * it will try to create a point hessian from the immature point stored in the feature
         * but if it doesn't have, will create an empty point hessian
         * @param hostFeature
         * 根据hostFeature创建一个3d地图点
         */
        Point(shared_ptr<Feature> hostFeature);
        Point();
        // 3D地图点的状态
        enum PointStatus {
            ACTIVE = 0,     // 在优化中是被激活的 point is still active in optimization
            OUTLIER,        // 在优化中认为是离群点，不进行优化 considered as outlier in optimization
            OUT,            // 在优化中认为是超出了视野范围 out side the boundary
            MARGINALIZED    // 边缘化了，两种情况，一是超出了视野范围，二是所属的关键帧被边缘化了 marginalized, usually also out of boundary, but also can be set because the host frame is marged
        };  // the status of this point

        /**
         * release the point hessian
         */
        void ReleasePH();

        /**
         * compute 3D position in world
         */
        void ComputeWorldPos();

        // save and load
        void save(ofstream &fout);

        void load(ifstream &fin, vector<shared_ptr<Frame>> &allKFs);

        // =========================================================================================================
        unsigned long id = 0;              // 地图点的id
        static unsigned long mNextId;       // 所有地图点的计数变量
        PointStatus status = PointStatus::ACTIVE;  // 默认当前点的状态为激活状态 status of this point
        Vec3 mWorldPos = Vec3::Zero();        // 在世界坐标系下的坐标，初始化为0 pos in world
        weak_ptr<Feature> mHostFeature;     // 对应的2d特征点的指针 the hosting feature creating this point

        // internal structures
        shared_ptr<internal::PointHessian> mpPH;  // 指针 point with hessians

    };

}