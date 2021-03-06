#include "internal/FrameHessian.h"
#include "internal/GlobalCalib.h"
#include "internal/GlobalFuncs.h"
#include "frontend/PixelSelector2.h"
#include "frontend/CoarseInitializer.h"
#include "frontend/nanoflann.h"

#include <iostream>


namespace ldso {
    /**
     * @brief 构造函数
     * @param ww 图片宽度
     * @param hh 图片长度
     * */
    CoarseInitializer::CoarseInitializer(int ww, int hh) : thisToNext_aff(0, 0), thisToNext(SE3()) {
        // 遍历所有金字塔层数
        for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
            points[lvl] = 0;
            numPoints[lvl] = 0;
        }
        // 雅克比矩阵的大小每一个像素点对应一个Vec10f，共ww*hh个
        JbBuffer = new Vec10f[ww * hh];
        JbBuffer_new = new Vec10f[ww * hh];
        // 需要使用仿射变换
        fixAffine = true;
        // 不输出debug信息
        printDebug = false;
        // wM对角矩阵 8x8
        wM.diagonal()[0] = wM.diagonal()[1] = wM.diagonal()[2] = SCALE_XI_ROT;
        wM.diagonal()[3] = wM.diagonal()[4] = wM.diagonal()[5] = SCALE_XI_TRANS;
        wM.diagonal()[6] = SCALE_A;
        wM.diagonal()[7] = SCALE_B;
    }
    // 析构函数
    CoarseInitializer::~CoarseInitializer() {
        // 遍历所有金字塔层数，删除所有的点
        for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
            if (points[lvl] != 0) delete[] points[lvl];
        }
        // 删除临时存储变量
        delete[] JbBuffer;
        delete[] JbBuffer_new;
    }
    // 输入第二帧数据进行处理
    bool CoarseInitializer::trackFrame(shared_ptr<FrameHessian> newFrameHessian) {
        // 记录作为新的一帧
        newFrame = newFrameHessian;
        int maxIterations[] = {5, 5, 10, 30, 50};
        // 一些参数
        alphaK = 2.5 * 2.5;
        alphaW = 150 * 150;
        regWeight = 0.8;
        couplingWeight = 1;
        // 如果位移不够大，认为上一次的数据不够准确，直接重新初始化所有参数，重新计算
        if (!snapped) {
            // 平移量初始化为0
            thisToNext.translation().setZero();
            for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
                int npts = numPoints[lvl];
                Pnt *ptsl = points[lvl];
                for (int i = 0; i < npts; i++) {
                    ptsl[i].iR = 1;// 初始化逆深度为1
                    ptsl[i].idepth_new = 1;// 初始化逆深度为1 
                    ptsl[i].lastHessian = 0;//
                }
            }
        }
        // 平移 和 光度变换 初始值
        // 上一次的值作为这一次计算的初始值
        SE3 refToNew_current = thisToNext;
        AffLight refToNew_aff_current = thisToNext_aff;
        // 根据曝光时间计算一个初始值
        if (firstFrame->ab_exposure > 0 && newFrame->ab_exposure > 0)
            refToNew_aff_current = AffLight(logf(newFrame->ab_exposure / firstFrame->ab_exposure),
                                            0); // coarse approximation.
        // 残差赋初始值为零
        Vec3f latestRes = Vec3f::Zero();    
        for (int lvl = pyrLevelsUsed - 1; lvl >= 0; lvl--)
        {//从低分辨率到高分辨率遍历
            // 除了顶层都要执行函数，进行初始化
            if (lvl < pyrLevelsUsed - 1)
                propagateDown(lvl + 1);// 使用上一层的值来初始化下一层

            Mat88f H, Hsc;
            Vec8f b, bsc;
            // 顶层初始化
            resetPoints(lvl);
            // 迭代计算前线计算H矩阵等
            Vec3f resOld = calcResAndGS(lvl, H, b, Hsc, bsc, refToNew_current, refToNew_aff_current, false);
            // 上一次的误差赋值给新的
            applyStep(lvl);
            // 迭代的参数
            float lambda = 0.1;
            float eps = 1e-4;
            int fails = 0;

            int iteration = 0;// 迭代次数
            while (true) {
                // 开始循环迭代求解
                Mat88f Hl = H;
                for (int i = 0; i < 8; i++) Hl(i, i) *= (1 + lambda);
                // 舒尔补，边缘化掉逆深度
                Hl -= Hsc * (1 / (1 + lambda));
                Vec8f bl = b - bsc * (1 / (1 + lambda));
                // 
                Hl = wM * Hl * wM * (0.01f / (w[lvl] * h[lvl]));
                bl = wM * bl * (0.01f / (w[lvl] * h[lvl]));

                // 求解增量
                Vec8f inc;
                if (fixAffine) {// 固定光度参数
                    inc.head<6>() = -(wM.toDenseMatrix().topLeftCorner<6, 6>() *
                                      (Hl.topLeftCorner<6, 6>().ldlt().solve(bl.head<6>())));
                    inc.tail<2>().setZero();
                } else
                    inc = -(wM * (Hl.ldlt().solve(bl)));    //=-H^-1 * b.

                // 更新状态，更新逆深度
                SE3 refToNew_new = SE3::exp(inc.head<6>().cast<double>()) * refToNew_current;
                AffLight refToNew_aff_new = refToNew_aff_current;
                refToNew_aff_new.a += inc[6];
                refToNew_aff_new.b += inc[7];
                doStep(lvl, lambda, inc);

                // 计算更新后的残差，与旧的对比判断是否接受结果
                Mat88f H_new, Hsc_new;
                Vec8f b_new, bsc_new;
                Vec3f resNew = calcResAndGS(lvl, H_new, b_new, Hsc_new, bsc_new, refToNew_new, refToNew_aff_new, false);
                Vec3f regEnergy = calcEC(lvl);

                float eTotalNew = (resNew[0] + resNew[1] + regEnergy[1]);
                float eTotalOld = (resOld[0] + resOld[1] + regEnergy[0]);


                bool accept = eTotalOld > eTotalNew;
                // 接受的话，更新状态
                if (accept) {
                    // 当位移足够大时才开始优化IR
                    if (resNew[1] == alphaK * numPoints[lvl])
                        snapped = true;
                    H = H_new;
                    b = b_new;
                    Hsc = Hsc_new;
                    bsc = bsc_new;
                    resOld = resNew;
                    refToNew_aff_current = refToNew_aff_new;
                    refToNew_current = refToNew_new;
                    applyStep(lvl);
                    optReg(lvl);
                    lambda *= 0.5;
                    fails = 0;
                    if (lambda < 0.0001) lambda = 0.0001;
                } else {// 否则增大Lambda再次迭代计算
                    fails++;
                    lambda *= 4;
                    if (lambda > 10000) lambda = 10000;
                }

                bool quitOpt = false;
                // 迭代停止条件 1、收敛 2、大于最大次数 3、失败两次以上
                if (!(inc.norm() > eps) || iteration >= maxIterations[lvl] || fails >= 2) {
                    Mat88f H, Hsc;
                    Vec8f b, bsc;

                    quitOpt = true;
                }


                if (quitOpt) break;
                iteration++;
            }
            latestRes = resOld;

        }

        thisToNext = refToNew_current;
        thisToNext_aff = refToNew_aff_current;
        // 从底层计算上层点的深度
        for (int i = 0; i < pyrLevelsUsed - 1; i++)
            propagateUp(i);

        frameID++;
        if (!snapped) snappedAt = 0;

        if (snapped && snappedAt == 0)
            snappedAt = frameID;
        // 位移足够大后，再优化5帧才行
        return snapped && frameID > snappedAt + 5;
    }

    // calculates residual, Hessian and Hessian-block neede for re-substituting depth.
    // 计算残差，Hessian矩阵以及舒尔补，sc代表Schur
    Vec3f CoarseInitializer::calcResAndGS(
            int lvl, Mat88f &H_out, Vec8f &b_out,
            Mat88f &H_out_sc, Vec8f &b_out_sc,
            const SE3 &refToNew, AffLight refToNew_aff,
            bool plot) {
        int wl = w[lvl], hl = h[lvl];
        // 当前层图像及梯度
        Eigen::Vector3f *colorRef = firstFrame->dIp[lvl];
        Eigen::Vector3f *colorNew = newFrame->dIp[lvl];
        // 旋转矩阵R * 内参矩阵 K_inv
        Mat33f RKi = (refToNew.rotationMatrix() * Ki[lvl]).cast<float>();
        Vec3f t = refToNew.translation().cast<float>();
        Eigen::Vector2f r2new_aff = Eigen::Vector2f(exp(refToNew_aff.a), refToNew_aff.b);
        // 该层的相机参数
        float fxl = fx[lvl];
        float fyl = fy[lvl];
        float cxl = cx[lvl];
        float cyl = cy[lvl];

        Accumulator11 E;// 1*1 的累加器
        // 初始值分配空间
        acc9.initialize();
        E.initialize();


        int npts = numPoints[lvl];
        Pnt *ptsl = points[lvl];
        for (int i = 0; i < npts; i++) {

            Pnt *point = ptsl + i;

            point->maxstep = 1e10;
            if (!point->isGood) {
                E.updateSingle((float) (point->energy[0]));
                point->energy_new = point->energy;
                point->isGood_new = false;
                continue;
            }

            VecNRf dp0;// 8*1矩阵，每个点附近的残差个数为8个
            VecNRf dp1;
            VecNRf dp2;
            VecNRf dp3;
            VecNRf dp4;
            VecNRf dp5;
            VecNRf dp6;
            VecNRf dp7;
            VecNRf dd;
            VecNRf r;
            JbBuffer_new[i].setZero();

            // sum over all residuals.
            bool isGood = true;
            float energy = 0;
            for (int idx = 0; idx < patternNum; idx++) {
                int dx = patternP[idx][0];
                int dy = patternP[idx][1];


                Vec3f pt = RKi * Vec3f(point->u + dx, point->v + dy, 1) + t * point->idepth_new;
                float u = pt[0] / pt[2];
                float v = pt[1] / pt[2];
                float Ku = fxl * u + cxl;
                float Kv = fyl * v + cyl;
                float new_idepth = point->idepth_new / pt[2];

                if (!(Ku > 1 && Kv > 1 && Ku < wl - 2 && Kv < hl - 2 && new_idepth > 0)) {
                    isGood = false;
                    break;
                }

                Vec3f hitColor = getInterpolatedElement33(colorNew, Ku, Kv, wl);
                //Vec3f hitColor = getInterpolatedElement33BiCub(colorNew, Ku, Kv, wl);

                //float rlR = colorRef[point->u+dx + (point->v+dy) * wl][0];
                float rlR = getInterpolatedElement31(colorRef, point->u + dx, point->v + dy, wl);

                if (!std::isfinite(rlR) || !std::isfinite((float) hitColor[0])) {
                    isGood = false;
                    break;
                }


                float residual = hitColor[0] - r2new_aff[0] * rlR - r2new_aff[1];
                float hw = fabs(residual) < setting_huberTH ? 1 : setting_huberTH / fabs(residual);
                energy += hw * residual * residual * (2 - hw);


                float dxdd = (t[0] - t[2] * u) / pt[2];
                float dydd = (t[1] - t[2] * v) / pt[2];

                if (hw < 1) hw = sqrtf(hw);
                float dxInterp = hw * hitColor[1] * fxl;
                float dyInterp = hw * hitColor[2] * fyl;
                dp0[idx] = new_idepth * dxInterp;
                dp1[idx] = new_idepth * dyInterp;
                dp2[idx] = -new_idepth * (u * dxInterp + v * dyInterp);
                dp3[idx] = -u * v * dxInterp - (1 + v * v) * dyInterp;
                dp4[idx] = (1 + u * u) * dxInterp + u * v * dyInterp;
                dp5[idx] = -v * dxInterp + u * dyInterp;
                dp6[idx] = -hw * r2new_aff[0] * rlR;
                dp7[idx] = -hw * 1;
                dd[idx] = dxInterp * dxdd + dyInterp * dydd;
                r[idx] = hw * residual;

                float maxstep = 1.0f / Vec2f(dxdd * fxl, dydd * fyl).norm();
                if (maxstep < point->maxstep) point->maxstep = maxstep;

                // immediately compute dp*dd' and dd*dd' in JbBuffer1.
                JbBuffer_new[i][0] += dp0[idx] * dd[idx];
                JbBuffer_new[i][1] += dp1[idx] * dd[idx];
                JbBuffer_new[i][2] += dp2[idx] * dd[idx];
                JbBuffer_new[i][3] += dp3[idx] * dd[idx];
                JbBuffer_new[i][4] += dp4[idx] * dd[idx];
                JbBuffer_new[i][5] += dp5[idx] * dd[idx];
                JbBuffer_new[i][6] += dp6[idx] * dd[idx];
                JbBuffer_new[i][7] += dp7[idx] * dd[idx];
                JbBuffer_new[i][8] += r[idx] * dd[idx];
                JbBuffer_new[i][9] += dd[idx] * dd[idx];
            }

            if (!isGood || energy > point->outlierTH * 20) {
                E.updateSingle((float) (point->energy[0]));
                point->isGood_new = false;
                point->energy_new = point->energy;
                continue;
            }


            // add into energy.
            E.updateSingle(energy);
            point->isGood_new = true;
            point->energy_new[0] = energy;

            // update Hessian matrix.
            for (int i = 0; i + 3 < patternNum; i += 4)
                acc9.updateSSE(
                        _mm_load_ps(((float *) (&dp0)) + i),
                        _mm_load_ps(((float *) (&dp1)) + i),
                        _mm_load_ps(((float *) (&dp2)) + i),
                        _mm_load_ps(((float *) (&dp3)) + i),
                        _mm_load_ps(((float *) (&dp4)) + i),
                        _mm_load_ps(((float *) (&dp5)) + i),
                        _mm_load_ps(((float *) (&dp6)) + i),
                        _mm_load_ps(((float *) (&dp7)) + i),
                        _mm_load_ps(((float *) (&r)) + i));


            for (int i = ((patternNum >> 2) << 2); i < patternNum; i++)
                acc9.updateSingle(
                        (float) dp0[i], (float) dp1[i], (float) dp2[i], (float) dp3[i],
                        (float) dp4[i], (float) dp5[i], (float) dp6[i], (float) dp7[i],
                        (float) r[i]);


        }

        E.finish();
        acc9.finish();

        // calculate alpha energy, and decide if we cap it.
        Accumulator11 EAlpha;
        EAlpha.initialize();
        for (int i = 0; i < npts; i++) {
            Pnt *point = ptsl + i;
            if (!point->isGood_new) {
                E.updateSingle((float) (point->energy[1]));
            } else {
                point->energy_new[1] = (point->idepth_new - 1) * (point->idepth_new - 1);
                E.updateSingle((float) (point->energy_new[1]));
            }
        }
        EAlpha.finish();
        float alphaEnergy = alphaW * (EAlpha.A + refToNew.translation().squaredNorm() * npts);

        // compute alpha opt.
        float alphaOpt;
        if (alphaEnergy > alphaK * npts) {
            alphaOpt = 0;
            alphaEnergy = alphaK * npts;
        } else {
            alphaOpt = alphaW;
        }

        acc9SC.initialize();
        for (int i = 0; i < npts; i++) {
            Pnt *point = ptsl + i;
            if (!point->isGood_new)
                continue;

            point->lastHessian_new = JbBuffer_new[i][9];

            JbBuffer_new[i][8] += alphaOpt * (point->idepth_new - 1);
            JbBuffer_new[i][9] += alphaOpt;

            if (alphaOpt == 0) {
                JbBuffer_new[i][8] += couplingWeight * (point->idepth_new - point->iR);
                JbBuffer_new[i][9] += couplingWeight;
            }

            JbBuffer_new[i][9] = 1 / (1 + JbBuffer_new[i][9]);
            acc9SC.updateSingleWeighted(
                    (float) JbBuffer_new[i][0], (float) JbBuffer_new[i][1], (float) JbBuffer_new[i][2],
                    (float) JbBuffer_new[i][3],
                    (float) JbBuffer_new[i][4], (float) JbBuffer_new[i][5], (float) JbBuffer_new[i][6],
                    (float) JbBuffer_new[i][7],
                    (float) JbBuffer_new[i][8], (float) JbBuffer_new[i][9]);
        }
        acc9SC.finish();

        H_out = acc9.H.topLeftCorner<8, 8>();// / acc9.num;
        b_out = acc9.H.topRightCorner<8, 1>();// / acc9.num;
        H_out_sc = acc9SC.H.topLeftCorner<8, 8>();// / acc9.num;
        b_out_sc = acc9SC.H.topRightCorner<8, 1>();// / acc9.num;

        H_out(0, 0) += alphaOpt * npts;
        H_out(1, 1) += alphaOpt * npts;
        H_out(2, 2) += alphaOpt * npts;

        Vec3f tlog = refToNew.log().head<3>().cast<float>();
        b_out[0] += tlog[0] * alphaOpt * npts;
        b_out[1] += tlog[1] * alphaOpt * npts;
        b_out[2] += tlog[2] * alphaOpt * npts;


        return Vec3f(E.A, alphaEnergy, E.num);
    }

    float CoarseInitializer::rescale() {
        float factor = 20 * thisToNext.translation().norm();
        return factor;
    }

    Vec3f CoarseInitializer::calcEC(int lvl) {
        if (!snapped) return Vec3f(0, 0, numPoints[lvl]);
        AccumulatorX<2> E;
        E.initialize();
        int npts = numPoints[lvl];
        for (int i = 0; i < npts; i++) {
            Pnt *point = points[lvl] + i;
            if (!point->isGood_new) continue;
            float rOld = (point->idepth - point->iR);
            float rNew = (point->idepth_new - point->iR);
            E.updateNoWeight(Vec2f(rOld * rOld, rNew * rNew));

        }
        E.finish();

        return Vec3f(couplingWeight * E.A1m[0], couplingWeight * E.A1m[1], E.num);
    }
    //* 计算旧的和新的逆深度与iR的差值, 返回旧的差, 新的差, 数目
    //? iR到底是啥呢     答：IR是逆深度的均值，尺度收敛到IR
    void CoarseInitializer::optReg(int lvl) {
        int npts = numPoints[lvl];
        Pnt *ptsl = points[lvl];
        if (!snapped) {
            for (int i = 0; i < npts; i++)
                ptsl[i].iR = 1;
            return;
        }

        for (int i = 0; i < npts; i++) {
            Pnt *point = ptsl + i;
            if (!point->isGood) continue;

            float idnn[10];
            int nnn = 0;
            for (int j = 0; j < 10; j++) {
                if (point->neighbours[j] == -1) continue;
                Pnt *other = ptsl + point->neighbours[j];
                if (!other->isGood) continue;
                idnn[nnn] = other->iR;
                nnn++;
            }

            if (nnn > 2) {
                std::nth_element(idnn, idnn + nnn / 2, idnn + nnn);
                point->iR = (1 - regWeight) * point->idepth + regWeight * idnn[nnn / 2];
            }
        }

    }

    //* 使用归一化积来更新高层逆深度值
    void CoarseInitializer::propagateUp(int srcLvl) {
        assert(srcLvl + 1 < pyrLevelsUsed);
        // set idepth of target

        int nptss = numPoints[srcLvl];
        int nptst = numPoints[srcLvl + 1];
        Pnt *ptss = points[srcLvl];
        Pnt *ptst = points[srcLvl + 1];

        // set to zero.
        for (int i = 0; i < nptst; i++) {
            Pnt *parent = ptst + i;
            parent->iR = 0;
            parent->iRSumNum = 0;
        }

        for (int i = 0; i < nptss; i++) {
            Pnt *point = ptss + i;
            if (!point->isGood) continue;

            Pnt *parent = ptst + point->parent;
            parent->iR += point->iR * point->lastHessian;
            parent->iRSumNum += point->lastHessian;
        }

        for (int i = 0; i < nptst; i++) {
            Pnt *parent = ptst + i;
            if (parent->iRSumNum > 0) {
                parent->idepth = parent->iR = (parent->iR / parent->iRSumNum);
                parent->isGood = true;
            }
        }

        optReg(srcLvl + 1);
    }
    //@ 使用上层信息来初始化下层
    //@ param: 当前的金字塔层+1
    //@ note: 没法初始化顶层值 
    void CoarseInitializer::propagateDown(int srcLvl) {
        assert(srcLvl > 0);
        // set idepth of target

        int nptst = numPoints[srcLvl - 1];
        Pnt *ptss = points[srcLvl];
        Pnt *ptst = points[srcLvl - 1];

        for (int i = 0; i < nptst; i++) {
            Pnt *point = ptst + i;
            Pnt *parent = ptss + point->parent;

            if (!parent->isGood || parent->lastHessian < 0.1) continue;
            if (!point->isGood) {
                point->iR = point->idepth = point->idepth_new = parent->iR;
                point->isGood = true;
                point->lastHessian = 0;
            } else {
                float newiR = (point->iR * point->lastHessian * 2 + parent->iR * parent->lastHessian) /
                              (point->lastHessian * 2 + parent->lastHessian);
                point->iR = point->idepth = point->idepth_new = newiR;
            }
        }
        optReg(srcLvl - 1);
    }

    //* 低层计算高层, 像素值和梯度
    void CoarseInitializer::makeGradients(Eigen::Vector3f **data) {
        for (int lvl = 1; lvl < pyrLevelsUsed; lvl++) {
            int lvlm1 = lvl - 1;
            int wl = w[lvl], hl = h[lvl], wlm1 = w[lvlm1];

            Eigen::Vector3f *dINew_l = data[lvl];
            Eigen::Vector3f *dINew_lm = data[lvlm1];

            for (int y = 0; y < hl; y++)
                for (int x = 0; x < wl; x++)
                    dINew_l[x + y * wl][0] = 0.25f * (dINew_lm[2 * x + 2 * y * wlm1][0] +
                                                      dINew_lm[2 * x + 1 + 2 * y * wlm1][0] +
                                                      dINew_lm[2 * x + 2 * y * wlm1 + wlm1][0] +
                                                      dINew_lm[2 * x + 1 + 2 * y * wlm1 + wlm1][0]);

            for (int idx = wl; idx < wl * (hl - 1); idx++) {
                dINew_l[idx][1] = 0.5f * (dINew_l[idx + 1][0] - dINew_l[idx - 1][0]);
                dINew_l[idx][2] = 0.5f * (dINew_l[idx + wl][0] - dINew_l[idx - wl][0]);
            }
        }
    }
    /**
     * @brief 设置第一帧数据
     * @param HCalib 相机标定参数
     * @param newFrameHessian 第一帧数据
     * */
    void CoarseInitializer::setFirst(shared_ptr<CalibHessian> HCalib, shared_ptr<FrameHessian> newFrameHessian) {
        // 计算金字塔各层的相机参数 
        makeK(HCalib);
        // 第一帧指向新的一帧
        firstFrame = newFrameHessian;
        // 像素选择类
        PixelSelector sel(w[0], h[0]);
        // 存储各点的状态，statusMap > 0 即为被选中的点
        float *statusMap = new float[w[0] * h[0]];
        bool *statusMapB = new bool[w[0] * h[0]];
        // 不同层取得点密度不同
        float densities[] = {0.03, 0.05, 0.15, 0.5, 1};
        // 遍历所有的金字塔层级
        // 针对不同层数选择大梯度像素，第零层比较复杂1d 2d 4d 大小block来选择三个层次的像素
        for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) 
        {
            sel.currentPotential = 3;// 设置网格的大小 3x3
            int npts;//选择像素点的数目
            // 选择梯度明显的点
            if (lvl == 0) {// 第零层提取特征像素
                npts = sel.makeMaps(firstFrame, statusMap, densities[lvl] * w[0] * h[0], 1, false, 2);
            } else {// 其他层提取goodPoints 
                npts = makePixelStatus(firstFrame->dIp[lvl], statusMapB, w[lvl], h[lvl], densities[lvl] * w[0] * h[0]);
            }
            // 如果非空，删除释放空间，创建新的
            if (points[lvl] != 0) delete[] points[lvl];
            // 创建一个新的点
            points[lvl] = new Pnt[npts];

            // set idepth map to initially 1 everywhere.
            int wl = w[lvl], hl = h[lvl];// 每一层图像的大小
            Pnt *pl = points[lvl];// 每一层图像上的点
            int nl = 0;
            // 留出patternPadding空间
            for (int y = patternPadding + 1; y < hl - patternPadding - 2; y++)
                for (int x = patternPadding + 1; x < wl - patternPadding - 2; x++)
                {
                    // 如果是被选中的点
                    // 第零层和其他选择的方式不一样
                    if ((lvl != 0 && statusMapB[x + y * wl]) || (lvl == 0 && statusMap[x + y * wl] != 0))
                    {
                        //assert(patternNum==9);
                        // 对当前层所有点进行赋值
                        pl[nl].u = x + 0.1;
                        pl[nl].v = y + 0.1;
                        pl[nl].idepth = 1;
                        pl[nl].iR = 1;
                        pl[nl].isGood = true;
                        pl[nl].energy.setZero();
                        pl[nl].lastHessian = 0;
                        pl[nl].lastHessian_new = 0;
                        pl[nl].my_type = (lvl != 0) ? 1 : statusMap[x + y * wl];
                        // 像素点的梯度
                        Eigen::Vector3f *cpt = firstFrame->dIp[lvl] + x + y * w[lvl];
                        float sumGrad2 = 0;
                        // pattern内像素梯度之和
                        for (int idx = 0; idx < patternNum; idx++)
                        {
                            int dx = patternP[idx][0];
                            int dy = patternP[idx][1];
                            float absgrad = cpt[dx + dy * w[lvl]].tail<2>().squaredNorm();
                            sumGrad2 += absgrad;
                        }

                        pl[nl].outlierTH = patternNum * setting_outlierTH;

                        nl++;
                        assert(nl <= npts);
                    }
                }


            numPoints[lvl] = nl;
        }
        delete[] statusMap;
        delete[] statusMapB;
        // 计算点的最邻近点和父点
        makeNN();
        // 一些参数的初始化
        thisToNext = SE3();
        snapped = false;
        frameID = snappedAt = 0;

        for (int i = 0; i < pyrLevelsUsed; i++)
            dGrads[i].setZero();

    }
    //@ 重置点的energy, idepth_new参数
    void CoarseInitializer::resetPoints(int lvl) {
        // lvl层的点
        Pnt *pts = points[lvl];
        // lvl层点的数量
        int npts = numPoints[lvl];
        for (int i = 0; i < npts; i++) {
            // 所有的点的残差设置为零
            pts[i].energy.setZero();
            // 新的逆深度初始化为之前的逆深度
            pts[i].idepth_new = pts[i].idepth;

            if (lvl == pyrLevelsUsed - 1 && !pts[i].isGood) {
                // 如果当前层是最顶层，使用周围点的平均值来重置
                float snd = 0, sn = 0;
                for (int n = 0; n < 10; n++) {
                    if (pts[i].neighbours[n] == -1 || !pts[pts[i].neighbours[n]].isGood) continue;
                    snd += pts[pts[i].neighbours[n]].iR;
                    sn += 1;
                }

                if (sn > 0) {
                    pts[i].isGood = true;
                    pts[i].iR = pts[i].idepth = pts[i].idepth_new = snd / sn;
                }
            }
        }
    }
    //* 求出状态增量后, 计算被边缘化掉的逆深度, 更新逆深度
    void CoarseInitializer::doStep(int lvl, float lambda, Vec8f inc) {

        const float maxPixelStep = 0.25;
        const float idMaxStep = 1e10;
        Pnt *pts = points[lvl];
        int npts = numPoints[lvl];
        for (int i = 0; i < npts; i++) {
            if (!pts[i].isGood) continue;


            float b = JbBuffer[i][8] + JbBuffer[i].head<8>().dot(inc);
            float step = -b * JbBuffer[i][9] / (1 + lambda);


            float maxstep = maxPixelStep * pts[i].maxstep;
            if (maxstep > idMaxStep) maxstep = idMaxStep;

            if (step > maxstep) step = maxstep;
            if (step < -maxstep) step = -maxstep;

            float newIdepth = pts[i].idepth + step;
            if (newIdepth < 1e-3) newIdepth = 1e-3;
            if (newIdepth > 50) newIdepth = 50;
            pts[i].idepth_new = newIdepth;
        }

    }
    //* 新的值赋值给旧的 (能量, 点状态, 逆深度, hessian)
    void CoarseInitializer::applyStep(int lvl) {
        Pnt *pts = points[lvl];
        int npts = numPoints[lvl];
        for (int i = 0; i < npts; i++) {
            if (!pts[i].isGood) {
                pts[i].idepth = pts[i].idepth_new = pts[i].iR;
                continue;
            }
            pts[i].energy = pts[i].energy_new;
            pts[i].isGood = pts[i].isGood_new;
            pts[i].idepth = pts[i].idepth_new;
            pts[i].lastHessian = pts[i].lastHessian_new;
        }
        std::swap<Vec10f *>(JbBuffer, JbBuffer_new);
    }

    /**
     * @brief 在输入第一帧数据和标定参数时调用
     * @param Hcalib 标定参数
     * */
    void CoarseInitializer::makeK(shared_ptr<CalibHessian> HCalib) {
        // w[0] h[0]
        w[0] = wG[0];
        h[0] = hG[0];
        // fx fy cx cy 都等于原始值
        fx[0] = HCalib->fxl();
        fy[0] = HCalib->fyl();
        cx[0] = HCalib->cxl();
        cy[0] = HCalib->cyl();
        // 遍历所有金字塔层
        for (int level = 1; level < pyrLevelsUsed; ++level) {
            // w 和 h缩放响应倍数
            w[level] = w[0] >> level;
            h[level] = h[0] >> level;
            // fx = fx / 2 
            fx[level] = fx[level - 1] * 0.5;
            fy[level] = fy[level - 1] * 0.5;
            // cx cy 
            cx[level] = (cx[0] + 0.5) / ((int) 1 << level) - 0.5;
            cy[level] = (cy[0] + 0.5) / ((int) 1 << level) - 0.5;
        }
        // 内参矩阵
        for (int level = 0; level < pyrLevelsUsed; ++level) {
            K[level] << fx[level], 0.0, cx[level], 0.0, fy[level], cy[level], 0.0, 0.0, 1.0;
            Ki[level] = K[level].inverse();
            fxi[level] = Ki[level](0, 0);
            fyi[level] = Ki[level](1, 1);
            cxi[level] = Ki[level](0, 2);
            cyi[level] = Ki[level](1, 2);
        }
    }
    //@ 生成每一层点的KDTree, 并用其找到邻近点集和父点 
    void CoarseInitializer::makeNN() {
        const float NNDistFactor = 0.05;

        typedef nanoflann::KDTreeSingleIndexAdaptor<
                nanoflann::L2_Simple_Adaptor<float, FLANNPointcloud>,
                FLANNPointcloud, 2> KDTree;

        // build indices
        FLANNPointcloud pcs[PYR_LEVELS];
        KDTree *indexes[PYR_LEVELS];
        for (int i = 0; i < pyrLevelsUsed; i++) {
            pcs[i] = FLANNPointcloud(numPoints[i], points[i]);
            indexes[i] = new KDTree(2, pcs[i], nanoflann::KDTreeSingleIndexAdaptorParams(5));
            indexes[i]->buildIndex();
        }

        const int nn = 10;

        // find NN & parents
        for (int lvl = 0; lvl < pyrLevelsUsed; lvl++) {
            Pnt *pts = points[lvl];
            int npts = numPoints[lvl];

            int ret_index[nn];
            float ret_dist[nn];
            nanoflann::KNNResultSet<float, int, int> resultSet(nn);
            nanoflann::KNNResultSet<float, int, int> resultSet1(1);

            for (int i = 0; i < npts; i++) {
                //resultSet.init(pts[i].neighbours, pts[i].neighboursDist );
                resultSet.init(ret_index, ret_dist);
                Vec2f pt = Vec2f(pts[i].u, pts[i].v);
                indexes[lvl]->findNeighbors(resultSet, (float *) &pt, nanoflann::SearchParams());
                int myidx = 0;
                float sumDF = 0;
                for (int k = 0; k < nn; k++) {
                    pts[i].neighbours[myidx] = ret_index[k];
                    float df = expf(-ret_dist[k] * NNDistFactor);
                    sumDF += df;
                    pts[i].neighboursDist[myidx] = df;
                    assert(ret_index[k] >= 0 && ret_index[k] < npts);
                    myidx++;
                }
                for (int k = 0; k < nn; k++)
                    pts[i].neighboursDist[k] *= 10 / sumDF;


                if (lvl < pyrLevelsUsed - 1) {
                    resultSet1.init(ret_index, ret_dist);
                    pt = pt * 0.5f - Vec2f(0.25f, 0.25f);
                    indexes[lvl + 1]->findNeighbors(resultSet1, (float *) &pt, nanoflann::SearchParams());

                    pts[i].parent = ret_index[0];
                    pts[i].parentDist = expf(-ret_dist[0] * NNDistFactor);

                    assert(ret_index[0] >= 0 && ret_index[0] < numPoints[lvl + 1]);
                } else {
                    pts[i].parent = -1;
                    pts[i].parentDist = -1;
                }
            }
        }
        // done.

        for (int i = 0; i < pyrLevelsUsed; i++)
            delete indexes[i];
    }
}