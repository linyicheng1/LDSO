#include "Settings.h"
#include "internal/PointHessian.h"
#include "internal/ImmaturePoint.h"

namespace ldso {

    namespace internal {
        /**
         * 从一个未成熟点构造点Hessian
         * */
        PointHessian::PointHessian(shared_ptr<ImmaturePoint> rawPoint) {

            u = rawPoint->feature->uv[0];// 位置u
            v = rawPoint->feature->uv[1];// 位置v
            my_type = rawPoint->my_type;// 类型
            // 逆深度数据，取逆深度最大和最小的均值
            this->idepth = SCALE_IDEPTH_INVERSE * (rawPoint->idepth_max + rawPoint->idepth_min) * 0.5;
            // 缩放后的逆深度，缩放系数初始化为1 
            this->idepth_scaled = this->idepth;
            // pattern 内包含的点数
            int n = patternNum;
            // pattern 内的灰度值也保存下来
            memcpy(color, rawPoint->color, sizeof(float) * n);
            // pattern 个点的权重
            memcpy(weights, rawPoint->weights, sizeof(float) * n);
            // 优化的残差
            energyTH = rawPoint->energyTH;
        }
    }

}
