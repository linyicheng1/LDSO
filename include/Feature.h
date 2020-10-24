#pragma once
#ifndef LDSO_FEATURE_H_
#define LDSO_FEATURE_H_

#include <memory>

using namespace std;

#include "NumTypes.h"

namespace ldso {

    struct Frame;
    struct Point;

    namespace internal {
        class ImmaturePoint;
    }

    /**
     * Feature is a 2D point in the image. A triangulated feature will have an associated 3D map point, but an immature
     * feature will not (instead it has a immature point). You can access a feature's host frame and the map point.
     *
     * Feature may have a descriptor (ORB currently) if it is a corner (with isCorner == true),
     * otherwise the descriptor, angle and level are always kept as the default value. Described features can be used
     * for feature matching, loop closing and bag-of-words ... anything you expect in a feature-based SLAM.
     *
     * NOTE outlier features will also be kept in frame know. If you worry about the memory cost you can just clean them
     */
    // 图像中的一个2d特征点
    struct Feature {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        /**
         * Feature status
         */
        // 特征点状态 
        enum FeatureStatus {
            IMMATURE = 0,    // 未成熟的 if immature, the ip will have the immature data
            VALID,           // 可用的，已经对应了一个3d地图点 have a valid map point (but the point may also be outdated or margined ... )
            OUTLIER          // 离群点 the immature point diverges, or the map point becomes an outlier
        };  // the status of feature
        // 构造函数
        // 2d点的位置 (u,v) 相关联的帧 host
        Feature(float u, float v, shared_ptr<Frame> host) : host(host), uv(Vec2f(u, v)) {}
        // 空析构函数
        ~Feature() {}

        /**
         * Create a map point from the immature structure
         */
        void CreateFromImmature();

        /**
         * release the internal immature point data
         */
        void ReleaseImmature();

        /**
         * release the map point
         */
        void ReleaseMapPoint();

        /**
         * release all internal data
         */
        inline void ReleaseAll() {
            ReleaseImmature();
            ReleaseMapPoint();
        }

        // save and load
        void save(ofstream &fout);

        void load(ifstream &fin, vector<shared_ptr<Frame>> &allKFs);

        // =====================================================================================================
        FeatureStatus status = IMMATURE;   // 特征点的状态，初始化为为成熟的 status of this feature

        weak_ptr<Frame> host;   // 持有该点的帧 the host frame

        Vec2f uv = Vec2f(0, 0);     // 特征点的像素位置 pixel position in image
        float invD = -1;            // 特征点的逆深度，初始化为-1，只有大于零时才有效 inverse depth, invalid if < 0, computed by dso's sliding window
        shared_ptr<Point> point = nullptr;    // 对应的3D地图点的指针 corresponding 3D point, nullptr if it is an immature point

        // feature stuffs
        float angle = 0;        // 点特征的角度 rotation
        float score = 0;        // 点特征的得分  shi-tomasi score
        bool isCorner = false; //  当前点是否为角点 indicating if this is a corner
        int level = 0;         //  从金字塔的第几层计算得到的 which pyramid level is the feature computed
        unsigned char descriptor[32] = {0};  // ORB 描述子，用于回环检测 ORB descriptors

        // internal structures for optimizing immature points
        shared_ptr<internal::ImmaturePoint> ip = nullptr;  // 指向未成熟的地图点 the immature point
    };
}

#endif