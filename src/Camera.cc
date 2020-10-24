#include "Camera.h"
#include "internal/CalibHessian.h"

using namespace ldso::internal;

namespace ldso {
    // 构造函数，得到相机的参数
    Camera::Camera( double fx, double fy, double cx, double cy) {
        this->fx = fx;
        this->fy = fy;
        this->cx = cx;
        this->cy = cy;
    }
    // 创造一个CalibHessan 结构体
    void Camera::CreateCH(shared_ptr<Camera> cam) {
        this->mpCH = shared_ptr<CalibHessian>( new CalibHessian(cam) );
    }
    // 将相互的指针关联删除
    void Camera::ReleaseCH() {
        if ( mpCH ) {
            mpCH->camera = nullptr;
            mpCH = nullptr;
        }
    }

}