#pragma once
#ifndef LDSO_CALIB_HESSIAN_H_
#define LDSO_CALIB_HESSIAN_H_

#include "NumTypes.h"
#include "Camera.h"
#include "Settings.h"

namespace ldso {

    namespace internal {

        /**
         * Camera intrinsics with hessian
         * 相机内参
         */
        class CalibHessian {
        public:
            EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

            CalibHessian(shared_ptr<Camera> cam) : camera(cam) {

                VecC initial_value = VecC::Zero();// 内参设置为零,四维，k1,k2,p1,p2,k3 其中k3默认=0
                initial_value[0] = cam->fx;// 记录下参数
                initial_value[1] = cam->fy;
                initial_value[2] = cam->cx;
                initial_value[3] = cam->cy;
                // 
                setValueScaled(initial_value);
                // 初始参数
                value_zero = value;
                value_minus_value_zero.setZero();
                // gamma映射函数，直接认为B[i] = i,即均匀映射
                for (int i = 0; i < 256; i++) {
                    Binv[i] = B[i] = i;    // set gamma function to identity
                }
            }

            // normal mode: use the optimized parameters everywhere!
            // accessors
            inline float &fxl() {
                return value_scaledf[0];
            }

            inline float &fyl() {
                return value_scaledf[1];
            }

            inline float &cxl() {
                return value_scaledf[2];
            }

            inline float &cyl() {
                return value_scaledf[3];
            }

            inline float &fxli() {
                return value_scaledi[0];
            }

            inline float &fyli() {
                return value_scaledi[1];
            }

            inline float &cxli() {
                return value_scaledi[2];
            }

            inline float &cyli() {
                return value_scaledi[3];
            }

            inline void setValue(const VecC &value) {
                // [0-3: Kl, 4-7: Kr, 8-12: l2r]
                this->value = value;
                value_scaled[0] = SCALE_F * value[0];
                value_scaled[1] = SCALE_F * value[1];
                value_scaled[2] = SCALE_C * value[2];
                value_scaled[3] = SCALE_C * value[3];

                this->value_scaledf = this->value_scaled.cast<float>();
                this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
                this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
                this->value_scaledi[2] = -this->value_scaledf[2] / this->value_scaledf[0];
                this->value_scaledi[3] = -this->value_scaledf[3] / this->value_scaledf[1];
                this->value_minus_value_zero = this->value - this->value_zero;
            };
            // 设置参数
            inline void setValueScaled(const VecC &value_scaled) {
                this->value_scaled = value_scaled;
                this->value_scaledf = this->value_scaled.cast<float>();
                value[0] = SCALE_F_INVERSE * value_scaled[0];
                value[1] = SCALE_F_INVERSE * value_scaled[1];
                value[2] = SCALE_C_INVERSE * value_scaled[2];
                value[3] = SCALE_C_INVERSE * value_scaled[3];

                this->value_minus_value_zero = this->value - this->value_zero;
                this->value_scaledi[0] = 1.0f / this->value_scaledf[0];
                this->value_scaledi[1] = 1.0f / this->value_scaledf[1];
                this->value_scaledi[2] = -this->value_scaledf[2] / this->value_scaledf[0];
                this->value_scaledi[3] = -this->value_scaledf[3] / this->value_scaledf[1];
            };
            // 得到真实的梯度，考虑到gamma映射
            EIGEN_STRONG_INLINE float getBGradOnly(float color) {
                int c = color + 0.5f;
                if (c < 5) {
                    c = 5;
                }
                if (c > 250) {
                    c = 250;
                }
                return B[c + 1] - B[c];
            }

            EIGEN_STRONG_INLINE float getBInvGradOnly(float color) {
                int c = color + 0.5f;
                if (c < 5) {
                    c = 5;
                }
                if (c > 250) {
                    c = 250;
                }
                return Binv[c + 1] - Binv[c];
            }
            // 相机数据结构指针
            shared_ptr<Camera> camera = nullptr;

            // values during estimation
            VecC value_zero = VecC::Zero();
            VecC value_scaled;      // [fx, fy, cx, cy] in double
            VecCf value_scaledf;    // [fx, fy, cx, cy] in float
            VecCf value_scaledi;    // inverse of value_scaledf

            VecC value;
            VecC step;
            VecC step_backup;
            VecC value_backup;
            VecC value_minus_value_zero;

            // gamma function, by default from 0 to 255
            float B[256];
            float Binv[256];
        };

    }

}

#endif // LDSO_CALIB_HESSIAN_H_
