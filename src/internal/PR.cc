#include "internal/PR.h"
#include "internal/GlobalCalib.h"
#include "internal/ResidualProjections.h"

using namespace ldso::internal;

namespace ldso {
    // 逆深度点之间的误差，直接减去即可
    void EdgeIDPPrior::computeError() {
        const VertexPointInvDepth *vIDP = static_cast<const VertexPointInvDepth *>(_vertices[0]);
        _error(0) = vIDP->estimate() - _measurement;
    }

    /*
    void EdgeIDPPrior::linearizeOplus() {
        _jacobianOplusXi.setZero();
        _jacobianOplusXi(0) = 1;
    }
     */

    /**
     * Erorr = pi(Px)-obs
     * 逆深度参数重投影误差参数
     */
    void EdgePRIDP::computeError() {
        // 获取连接的三个节点
        // 连接的逆深度节点
        const VertexPointInvDepth *vIDP = dynamic_cast<const VertexPointInvDepth *>( _vertices[0]);
        // 参考帧
        const VertexPR *vPR0 = dynamic_cast<const VertexPR *>(_vertices[1]);
        // 当前帧
        const VertexPR *vPRi = dynamic_cast<const VertexPR *>(_vertices[2]);

        // point inverse depth in reference KF
        // 得到上一次的估计值
        double rho = vIDP->estimate();
        if (rho < 1e-6) {// 避免估计得到负数
            // LOG(WARNING) << "Inv depth should not be negative: " << rho << endl;
            return;
        }

        // point coordinate in reference KF, body
        // 图像中位置
        Vec3 P0(x, y, 1.0);
        // 3d位置
        P0 = P0 * (1.0f / rho);
        // w中的位置
        Vec3 Pw = vPR0->estimate().inverse() * P0;
        // i中的位置
        Vec3 Pi = vPRi->estimate() * Pw;

        if (Pi[2] < 0) {
            // LOG(WARNING) << "projected depth should not be negative: " << Pi.transpose() << endl;
            return;
        }
        // 转换到像素坐标系中的位置
        double xi = Pi[0] / Pi[2];
        double yi = Pi[1] / Pi[2];
        double u = cam->fxl() * xi + cam->cxl();
        double v = cam->fyl() * yi + cam->cyl();

        // 误差 = 估计值 - 测量值
        _error = Vec2(u, v) - _measurement;
    }

    //  仅优化位姿的误差函数
    void EdgeProjectPoseOnly::computeError() {
        // 得到一条边的参数
        const VertexPR *vPR = static_cast<VertexPR *> (_vertices[0]);
        // 当前边的估计参数
        SE3 Tcw = vPR->estimate();
        // 当前坐标系下点的位置
        Vec3 pc = Tcw * pw;// pw 为[x,y,z]
        pc = pc * (1.0 / pc[2]);
        if (pc[2] < 0) {
            LOG(WARNING) << "invalid depth: " << pc[2] << endl;
            depthValid = false;
            return;
        }
        // 重投影误差 
        double u = fx * pc[0] + cx;
        double v = fy * pc[1] + cy;
        _error = Vec2(u, v) - _measurement;
    }
    // sim3的冲投影误差，只是加了一个缩放系数而已，其他都一样
    void EdgeProjectPoseOnlySim3::computeError() {

        const VertexSim3 *vSim3 = static_cast<VertexSim3 *> (vertex(0));
        Sim3 Scw = vSim3->estimate();

        Vec3 pc = Scw.scale() * Scw.rotationMatrix() * pw + Scw.translation();
        pc = pc * (1.0 / pc[2]);

        if (pc[2] < 0) {
            LOG(WARNING) << "invalid depth: " << pc[2] << endl;
            setLevel(1);
            depthValid = false;
            return;
        }

        double u = fx * pc[0] + cx;
        double v = fy * pc[1] + cy;
        _error = Vec2(u, v) - _measurement;
    }
}
