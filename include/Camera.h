#pragma once
#ifndef LDSO_CAMERA_H_
#define LDSO_CAMERA_H_

#include "NumTypes.h"

namespace ldso {

    namespace internal {
        class CalibHessian;
    }

    /**
     * @brief Pinhole camera model
     * the parameters will be estimated during optimization
     * 针孔相机模型
     */
    struct Camera {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 相机模型初始化，焦距和中心
        Camera(double fx, double fy, double cx, double cy);

        /**
         * Create the internal structure, otherwise it will be nullptr
         * 新建一个优化标定参数项
         * @param cam
         */
        void CreateCH(shared_ptr<Camera> cam);

        /**
         * Release the internal structure
         */
        void ReleaseCH();

        // data
        double fx = 0, fy = 0, cx = 0, cy = 0;// 参数

        // internal structure
        // 标定参数优化结构体
        shared_ptr<internal::CalibHessian> mpCH = nullptr;
    };

}

#endif // LDSO_CAMERA_H_
