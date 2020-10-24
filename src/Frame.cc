#include "Frame.h"
#include "Feature.h"
#include "Point.h"

#include "internal/FrameHessian.h"
#include "internal/GlobalCalib.h"


using namespace ldso::internal;

namespace ldso {
    // 计算frame数量
    unsigned long Frame::nextId = 0;
    // 空的一帧数据
    Frame::Frame() {
        id = nextId++;
    }
    // 构造函数
    Frame::Frame(double timestamp) {
        id = nextId++;// 帧数量+1
        // 记录一下时间戳
        this->timeStamp = timestamp;
    }
    // 删除 frameHessian
    void Frame::ReleaseFH() {
        if (frameHessian) {
            frameHessian->frame = nullptr;
            frameHessian = nullptr;
        }
    }
    // 删除所有的特征点
    void Frame::ReleaseFeatures() {
        for (auto &feat: features) {// 遍历所有特征点
            feat->ReleaseAll();// 调用特征点内部函数删除特征点
        }
    }
    // 新建 new 一个 FramehEssian 出来
    void Frame::CreateFH(shared_ptr<Frame> frame) {
        frameHessian = shared_ptr<internal::FrameHessian>(new internal::FrameHessian(frame));
    }
    // 设置特征点栅格
    void Frame::SetFeatureGrid() {
        // 得到栅格数量 gw*gh
        int gw = wG[0] / gridSize, gh = hG[0] / gridSize;
        // 栅格大小设置为 gw*gh
        grid.resize(gw * gh);
        // 遍历所有特征点的个数
        for (size_t i = 0; i < features.size(); i++) {
            if (features[i]->isCorner) {
                // assign feature to grid
                // 得到所属栅格中特征点
                int gridX = features[i]->uv[0] / gridSize;
                int gridY = features[i]->uv[1] / gridSize;
                // 将对应id压入所属栅格中
                grid[gridY * gw + gridX].push_back(i);
            }
        }
    }
    // 获取对应栅格内的特征点id
    vector<size_t> Frame::GetFeatureInGrid(const float &x, const float &y, const float &radius)
    {
        // 活动 gw gh
        vector<size_t> indices;
        int gw = wG[0] / gridSize, gh = hG[0] / gridSize;
        // 栅格 x min 
        int gridXmin = max(0, int(x - radius) / gridSize);
        if (gridXmin >= gw)
            return indices;
        // 栅格 x max
        int gridXmax = min(gw - 1, int(x + radius) / gridSize);
        if (gridXmax < 0)
            return indices;
        // 栅格 y min 
        int gridYmin = max(0, int(y - radius) / gridSize);
        if (gridYmin >= gh)
            return indices;
        // 栅格 y max 
        int gridYmax = min(gh - 1, int(y + radius) / gridSize);
        if (gridYmax < 0)
            return indices;
        // 半径范围
        float r2 = radius * radius;

        for (int ix = gridXmin; ix <= gridXmax; ix++)
            for (int iy = gridYmin; iy <= gridYmax; iy++) 
            {// 遍历范围内的栅格
                const vector<size_t> cell = grid[iy * gw + ix];
                for (auto &k: cell) {
                    float u = features[k]->uv[0];
                    float v = features[k]->uv[1];
                    // 特征点在半径范围内，则记录该特征点对应的id
                    if (((u - x) * (u - x) + (v - y) * (v - y)) < r2)
                        indices.push_back(k);
                }
            }
        // 返回的vector包含了所有在该范围内的特征点id    
        return indices;
    }
    // 计算词袋模型
    void Frame::ComputeBoW(shared_ptr<ORBVocabulary> voc) 
    {
        // convert corners into BoW
        vector<cv::Mat> allDesp;
        for (size_t i = 0; i < features.size(); i++) {
            auto &feat = features[i];
            if (feat->isCorner) {
                cv::Mat m(1, 32, CV_8U);
                for (int k = 0; k < 32; k++)
                    m.data[k] = feat->descriptor[k];
                allDesp.push_back(m);
                bowIdx.push_back(i);
            }
        }
        voc->transform(allDesp, bowVec, featVec, 4);
    }
    // 获取所有和当前帧连接的帧
    set<shared_ptr<Frame>> Frame::GetConnectedKeyFrames() {
        set<shared_ptr<Frame>> connectedFrames;
        for (auto &rel: poseRel)
            connectedFrames.insert(rel.first);
        return connectedFrames;
    }
    // 获取所有激活状态的特征点对应的3d地图点
    vector<shared_ptr<Point>> Frame::GetPoints() {
        vector<shared_ptr<Point>> pts;
        for (auto &feat: features) {
            // 遍历所有特征点
            if (feat->status == Feature::FeatureStatus::VALID) {
                // 取出对应所有的3d地图点
                pts.push_back(feat->point);
            }
        }
        // 返回所有的3d地图点
        return pts;
    }
    // 保存数据
    void Frame::save(ofstream &fout) {

        fout.write((char *) &id, sizeof(id));
        fout.write((char *) &kfId, sizeof(kfId));

        Mat44 Tcw = this->Tcw.matrix();
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++)
                fout.write((char *) &Tcw(i, j), sizeof(double));

        int nFeature = features.size();

        fout.write((char *) &nFeature, sizeof(nFeature));
        for (auto &feat: features) {
            feat->save(fout);   // save the feats
        }

        // save relationship with other keyframes
        int nPoseRel = poseRel.size();
        fout.write((char *) &nPoseRel, sizeof(nPoseRel));

        for (auto &rel: poseRel) {
            fout.write((char *) &rel.first->kfId, sizeof(unsigned long));
            Mat44 T = rel.second.Tcr.matrix();
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++)
                    fout.write((char *) &T(i, j), sizeof(double));
        }

    }
    // 加载数据
    void Frame::load(ifstream &fin, shared_ptr<Frame> &thisFrame, vector<shared_ptr<Frame>> &allKF) {

        fin.read((char *) &id, sizeof(id));
        fin.read((char *) &kfId, sizeof(kfId));

        Mat44 Tcw;
        for (int i = 0; i < 4; i++)
            for (int j = 0; j < 4; j++) {
                fin.read((char *) &Tcw(i, j), sizeof(double));
            }

        this->Tcw = SE3(Tcw);
        TcwOpti = Sim3(this->Tcw.matrix());

        int nufeatures = 0;
        fin.read((char *) &nufeatures, sizeof(int));
        features.resize(nufeatures, nullptr);
        for (auto &feat: features) {
            feat = shared_ptr<Feature>(new Feature(0, 0, thisFrame));
        }

        int n = 0;
        for (auto &feat: features) {
            feat->load(fin, allKF);
            if (feat->status == Feature::FeatureStatus::VALID) {
                feat->point->mHostFeature = feat;
            }
            n++;
        }

        int nuposeRel = 0;
        fin.read((char *) &nuposeRel, sizeof(nuposeRel));

        for (int k = 0; k < nuposeRel; k++) {
            unsigned long kfID = 0;
            fin.read((char *) &kfID, sizeof(kfID));

            Mat44 T;
            for (int i = 0; i < 4; i++)
                for (int j = 0; j < 4; j++) {
                    fin.read((char *) &T(i, j), sizeof(double));
                }

            Sim3 Trel(T);
            Frame::RELPOSE r;
            r.Tcr = Trel;
            poseRel[allKF[kfID]] = r;
        }
    }
}
