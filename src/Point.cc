#include "Feature.h"
#include "Point.h"
#include "Frame.h"
#include "internal/PointHessian.h"
#include "internal/GlobalCalib.h"

using namespace ldso::internal;
namespace ldso {
    // static 变量记录所有点的个数
    unsigned long Point::mNextId = 0;
    // 根据2d特征点构造3d地图点
    Point::Point(shared_ptr<Feature> hostFeature) {
        // 指针指向对应的特征点
        mHostFeature = hostFeature;
        if (hostFeature->ip)    // create from immature point 根据未成熟的点数据，创建3d地图点
            mpPH = shared_ptr<PointHessian>(new PointHessian(hostFeature->ip));
        else {
            // 没有未成熟的点，无法创建
            LOG(ERROR) << "map point created without immature point, this should not happen!" << endl;
            mpPH = shared_ptr<PointHessian>(new PointHessian());
        }
        // mpPh 指向 point 
        mpPH->point = hostFeature->point;
        id = mNextId++;
    }
    // 得到一个空的3d点，应该不会被调用
    Point::Point() {
        id = mNextId++;
    }
    // 删除PH点
    void Point::ReleasePH() {
        if (mpPH) {
            // 直接这样删除指针，不会内存泄露嘛。。。
            mpPH->point = nullptr;
            mpPH = nullptr;
        }
    }
    // 计算点在世界坐标系下的坐标
    void Point::ComputeWorldPos() {
        shared_ptr<Feature> feat = mHostFeature.lock();
        if (feat) {
            shared_ptr<Frame> frame = feat->host.lock();
            if (!frame)
                return;
            // 得到当前帧的坐标
            Sim3 Twc = frame->getPoseOpti().inverse();
            // 当前点在当前帧下的坐标
            Vec3 Kip = 1.0 / feat->invD * Vec3(
                    fxiG[0] * feat->uv[0] + cxiG[0],
                    fyiG[0] * feat->uv[1] + cyiG[0],
                    1);
            // 变换到世界坐标系下的坐标        
            mWorldPos = Twc * Kip;
        }
    }

    void Point::save(ofstream &fout) {
        fout.write((char *) &id, sizeof(id));
        fout.write((char *) &status, sizeof(status));
    }

    void Point::load(ifstream &fin, vector<shared_ptr<Frame>> &allKFs) {

        fin.read((char *) &id, sizeof(id));
        fin.read((char *) &status, sizeof(status));
    }
}