#include "Feature.h"
#include "Point.h"
#include "internal/ImmaturePoint.h"
#include "internal/PointHessian.h"

#include <memory>

using namespace std;

using namespace ldso::internal;

namespace ldso {

    /**
     * 从未成熟的点中，创建3d地图点
     * */
    void Feature::CreateFromImmature() {
        // 如果3d地图点已经存在了，就不要创建第二次
        if (point) {
            LOG(WARNING) << "Map point already created! You cannot create twice! " << endl;
            return;
        }
        assert(ip != nullptr);
        // 使用当前特征点，创建一个3d地图点
        point = shared_ptr<Point>(new Point(ip->feature));
        // 指针指向该3d地图点
        point->mpPH->point = point;   // set the point hessians backward pointer
        // 设置当前状态为有效的
        status = Feature::FeatureStatus::VALID;
    }
    /**
     * 删除对应的未成熟的点
     * */
    void Feature::ReleaseImmature() {
        if (ip) {
            ip->feature = nullptr;// 未成熟点不再指向当前特征
            ip = nullptr;// 当前特征也不指向未成熟点
        }
    }
    /**
     * 删除对应的地图点
     * */
    void Feature::ReleaseMapPoint() {
        if (point) {
            point->ReleasePH();// 调用点中的删除函数即可
        }
    }
    /**
     * 保存数据
     * */
    void Feature::save(ofstream &fout) {
        fout.write((char *) &status, sizeof(status));
        fout.write((char *) &uv[0], sizeof(float));
        fout.write((char *) &uv[1], sizeof(float));
        fout.write((char *) &invD, sizeof(float));
        fout.write((char *) &isCorner, sizeof(bool));
        fout.write((char *) &angle, sizeof(float));
        fout.write((char *) &score, sizeof(float));
        fout.write((char *) descriptor, sizeof(uchar) * 32);
        if (point && status == Feature::FeatureStatus::VALID)
            point->save(fout);
    }
    /**
     * 加载数据
     * */
    void Feature::load(ifstream &fin, vector<shared_ptr<Frame>> &allKFs) {

        fin.read((char *) &status, sizeof(status));
        fin.read((char *) &uv[0], sizeof(float));
        fin.read((char *) &uv[1], sizeof(float));
        fin.read((char *) &invD, sizeof(float));
        fin.read((char *) &isCorner, sizeof(bool));
        fin.read((char *) &angle, sizeof(float));
        fin.read((char *) &score, sizeof(float));
        fin.read((char *) descriptor, sizeof(uchar) * 32);

        if (status == Feature::FeatureStatus::VALID) {
            point = shared_ptr<Point>(new Point);
            point->load(fin, allKFs);
        }
    }

}