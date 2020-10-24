#pragma once
#ifndef LDSO_AFF_LIGHT_H_
#define LDSO_AFF_LIGHT_H_

#include "NumTypes.h"

namespace ldso {

    /**
     * @brief the affine light function, use a,b here to describe the exposure change
     * see dso's paper for details
     * 放射光照变换
     */
    struct AffLight {

        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        AffLight() {}
        // 系数 a,b
        AffLight(float a_, float b_) : a(a_), b(b_) {}

        // Affine Parameters:
        // 仿射参数
        float a = 0, b = 0;     // I_frame = exp(a)*I_global + b. // I_global = exp(-a)*(I_frame - b).

        /**
         * @brief compute the a,b of "from" to "to"
         * @param exposeF 
         * @param exposeT 
         * @param g2F     
         * @param g2T     
         */
        static Vec2 fromToVecExposure(float exposureF, float exposureT, AffLight g2F, AffLight g2T) {
            // 
            if (exposureF == 0 || exposureT == 0) {
                exposureT = exposureF = 1;
            }
            // A 
            float a = exp(g2T.a - g2F.a) * exposureT / exposureF;
            float b = g2T.b - a * g2F.b;
            return Vec2(a, b);
        }

        /// return the vectorized parameters
        Vec2 vec() {
            return Vec2(a, b);
        }
    };
}

#endif // LDSO_AFF_LIGHT_H_
