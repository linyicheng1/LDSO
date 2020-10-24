#pragma once
#ifndef LDSO_PR_H_
#define LDSO_PR_H_

#include "NumTypes.h"
#include "internal/CalibHessian.h"

#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/base_multi_edge.h>
#include <g2o/core/base_binary_edge.h>
#include <g2o/types/vertex_pointxyz.h>
#include <g2o/types/edge_pointxyz.h>

using namespace Eigen;
using namespace g2o;

using namespace ldso::internal;

namespace ldso {

    struct Frame;

    // ---------------------------------------------------------------------------------------------------------
    // some g2o types will be used in pose graph
    // ---------------------------------------------------------------------------------------------------------
    // G2O的一些优化数据类型，进行了一些重定义

    // 位姿优化节点
    class VertexPR : public BaseVertex<6, SE3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 构造函数
        VertexPR() : BaseVertex<6, SE3>() {}
        // 重载读写，没啥意义，但是必须要重写
        bool read(std::istream &is) override { return true; }
        bool write(std::ostream &os) const override { return true; }
        // 设置初始值，需要重载
        virtual void setToOriginImpl() override {
            _estimate = SE3();
        }
        // 更新状态
        // 更新量 update_ 更新当前状态
        virtual void oplusImpl(const double *update_) override {
            Vec6 update;
            update << update_[0], update_[1], update_[2], update_[3], update_[4], update_[5];
            _estimate = SE3::exp(update) * _estimate;
        }
        // 返回对应的旋转矩阵R
        inline Matrix3d R() const {
            return _estimate.so3().matrix();
        }
        // 返回平移向量t
        inline Vector3d t() const {
            return _estimate.translation();
        }
    };
    // sim3位姿优化节点，多了一个尺度
    class VertexSim3 : public BaseVertex<7, Sim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 没啥啥意义的重载
        VertexSim3() : BaseVertex<7, Sim3>() {}
        bool read(std::istream &is) override { return true; }
        bool write(std::ostream &os) const override { return true; }
        // 设置初始值
        virtual void setToOriginImpl() override {
            _estimate = Sim3();
        }
        // 更新的方法和SE3一样
        virtual void oplusImpl(const double *update_) override {
            Vec7 update;
            update << update_[0], update_[1], update_[2], update_[3], update_[4], update_[5], update_[6];
            _estimate = Sim3::exp(update) * _estimate;// 内部Sophus已经实现了
        }
    };

    /**
     * point with inverse depth
     */
    // 逆深度参数表达的点
    class VertexPointInvDepth : public BaseVertex<1, double> {

    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 逆深度表达的话只有一个参数
        VertexPointInvDepth() : BaseVertex<1, double>() {};
        bool read(std::istream &is) { return true; }
        bool write(std::ostream &os) const { return true; }
        // 初始值设置为 1 
        virtual void setToOriginImpl() {
            _estimate = 1.0;
        }
        // 在欧式空间中，更新直接加起来就好
        virtual void oplusImpl(const double *update) {
            _estimate += update[0];
        }
    };


    // ---- Edges --------------------------------------------------------------------------------------------------
    // G2O中的一些边（误差函数）
    /**
     * Edge of inverse depth prior for stereo-triangulated mappoints
     * Vertex: inverse depth map point
     *
     * Note: User should set the information matrix (inverse covariance) according to feature position uncertainty and baseline
     */
    // 逆深度参数点的误差
    class EdgeIDPPrior : public BaseUnaryEdge<1, double, VertexPointInvDepth> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 重载空函数
        EdgeIDPPrior() : BaseUnaryEdge<1, double, VertexPointInvDepth>() {}
        virtual bool read(std::istream &is) override { return true; }
        virtual bool write(std::ostream &os) const override { return true; }
        // 计算误差
        virtual void computeError() override;
        // 手动求导时需要写
        // virtual void linearizeOplus() override;
    };

    /**
     * Odometry edge
     * 两个SE3之间的误差函数
     * err = T1.inv * T2
     */
    class EdgePR : public BaseBinaryEdge<6, SE3, VertexPR, VertexPR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 重载空函数
        EdgePR() : BaseBinaryEdge<6, SE3, VertexPR, VertexPR>() {}
        virtual bool read(std::istream &is) override { return true; }
        virtual bool write(std::ostream &os) const override { return true; }
        // 计算误差，先旋转过去，再旋转回来，最后取log
        virtual void computeError() override {
            SE3 v1 = (static_cast<VertexPR *> (_vertices[0]))->estimate();
            SE3 v2 = (static_cast<VertexPR *> (_vertices[1]))->estimate();
            _error = (_measurement.inverse() * v1 * v2.inverse()).log();
        };
        // jacobian implemented by numeric jacobian
    };

    /**
     * Monocular Sim3 edge
     * 单目 sim3 之间的误差
     */
    class EdgeSim3 : public BaseBinaryEdge<7, Sim3, VertexSim3, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 重载空函数
        EdgeSim3() : BaseBinaryEdge<7, Sim3, VertexSim3, VertexSim3>() {}
        virtual bool read(std::istream &is) override { return true; }
        virtual bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override {
            Sim3 v1 = (static_cast<VertexSim3 *> (_vertices[0]))->estimate();
            Sim3 v2 = (static_cast<VertexSim3 *> (_vertices[1]))->estimate();
            _error = (_measurement.inverse() * v1 * v2.inverse()).log();
        };

        virtual double initialEstimatePossible (
                const OptimizableGraph::VertexSet &, OptimizableGraph::Vertex *) override { return 1.; }

        virtual void initialEstimate  (
                const OptimizableGraph::VertexSet &from, OptimizableGraph::Vertex * /*to*/) override {
            VertexSim3 *v1 = static_cast<VertexSim3 *>(_vertices[0]);
            VertexSim3 *v2 = static_cast<VertexSim3 *>(_vertices[1]);
            if (from.count(v1) > 0)
                v2->setEstimate(measurement().inverse() * v1->estimate());
            else
                v1->setEstimate(measurement() * v2->estimate());
        }
    };

    /**
    * Edge of reprojection error in one frame. Contain 3 vectices
    * 这才是最重要的误差函数，连接地图点、参考帧和当前帧
    * Vertex 0: inverse depth map point
    * Veretx 1: Host KF PR
    * Vertex 2: Target KF PR
    **/
    class EdgePRIDP : public BaseMultiEdge<2, Vec2> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        // give the normalized x, y and camera intrinsics
        EdgePRIDP(double x, double y, shared_ptr<CalibHessian> &calib) : BaseMultiEdge<2, Vec2>() {
            resize(3);
            this->x = x;
            this->y = y;
            cam = calib;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }
        // 计算误差函数
        virtual void computeError() override;

        // virtual void linearizeOplus() override;
        // 判断深度数据是否可以用了
        bool isDepthValid() {
            return dynamic_cast<const VertexPointInvDepth *>( _vertices[0])->estimate() > 0;
        }

    protected:

        // [x,y] in normalized image plane in reference KF
        double x = 0, y = 0;
        bool linearized = false;

        shared_ptr<CalibHessian> cam;
    };
    i// 仅仅优化位姿的误差
    class EdgeProjectPoseOnly : public BaseUnaryEdge<2, Vector2d, VertexPR> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        // 构造函数
        EdgeProjectPoseOnly(const shared_ptr<Camera> cam, const Vector3d &pw_)
                : BaseUnaryEdge<2, Vector2d, VertexPR>(), pw(pw_) {
            fx = cam->fx;
            fy = cam->fy;
            cx = cam->cx;
            cy = cam->cy;
        }
        bool read(std::istream &is) override { return true; }
        bool write(std::ostream &os) const override { return true; }
        // 计算误差
        virtual void computeError() override;

    public:
        bool depthValid = true;
    private:
        double fx = 0, fy = 0, cx = 0, cy = 0;
        Vector3d pw;    // world 3d position

    };

    class EdgeProjectPoseOnlySim3 : public BaseUnaryEdge<2, Vector2d, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgeProjectPoseOnlySim3(const shared_ptr<Camera> cam, const Vector3d &pw_)
                : BaseUnaryEdge<2, Vector2d, VertexSim3>(), pw(pw_) {
            fx = cam->fx;
            fy = cam->fy;
            cx = cam->cx;
            cy = cam->cy;
        }

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override;

    public:
        bool depthValid = true;

    private:
        double fx = 0, fy = 0, cx = 0, cy = 0;
        Vector3d pw;    // world 3d position

    };

    /**
     * error = measurement - Sim3*pw
     */
    class EdgePointSim3 : public BaseUnaryEdge<3, Vector3d, VertexSim3> {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;

        EdgePointSim3(const Vector3d &pw_)
                : BaseUnaryEdge<3, Vector3d, VertexSim3>(), pw(pw_) {}

        bool read(std::istream &is) override { return true; }

        bool write(std::ostream &os) const override { return true; }

        virtual void computeError() override {

            const VertexSim3 *vSim3 = static_cast<VertexSim3 *> (vertex(0));
            Sim3 Scw = vSim3->estimate();

            if (std::isnan(Scw.scale())) {
                LOG(INFO) << "Scw has nan: \n" << Scw.matrix() << endl;
                return;
            }

            _error = _measurement - Scw * pw;
        }

    private:
        Vector3d pw;    // world 3d position

    };
}


#endif // LDSO_PR_H_
