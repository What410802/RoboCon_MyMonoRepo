// 站立控制律（与 stand.cpp 里的主循环解耦，对标 python/ 的 control.py）。
//
// 模型只有 12 个 <motor>（ctrl 直接是关节力矩，±20 N·m），所以"位控"只能在这里自己算：
//     tau = kp·(q_des − q) − kd·q̇            （--gravity-comp 时再叠加 qfrc_bias）
// q_des 由调用方给：stand.cpp 里搜出来的屈膝站姿（搜索本身不读 keyframe）。--start rest 时
// 调用方再用 RampFrom() 把 q_des 在若干秒内从"趴卧关节角"推到站姿——**控制器还是这一个**，
// 变的只是随时间的目标，所以起身不需要另写一套控制器。
// 关节顺序（FL/FR/RR/RL × hip/thigh/calf）不写死，按 执行器→关节 的映射现查。
#pragma once

#include <mujoco/mujoco.h>

#include <algorithm>
#include <cstdio>
#include <vector>

namespace stand {

class StanceController {
public:
    StanceController(mjModel *m, const std::vector<double> &q_des, double kp, double kd,
                     bool gravity_comp)
        : kp_(kp), kd_(kd), gravity_comp_(gravity_comp), q_des_(q_des) {
        qadr_.resize(m->nu);
        vadr_.resize(m->nu);
        for (int i = 0; i < m->nu; ++i) {
            const int j = m->actuator_trnid[2 * i]; // 该执行器作用的关节 id
            qadr_[i] = m->jnt_qposadr[j];
            vadr_[i] = m->jnt_dofadr[j];
        }
        std::printf("控制：关节空间 PD kp=%g kd=%g%s；目标关节角：", kp_, kd_,
                    gravity_comp_ ? " + qfrc_bias 补偿" : "");
        for (size_t i = 0; i < q_des_.size(); ++i)
            std::printf("%.2f ", q_des_[i]);
        std::printf("\n");
    }

    // 起步斜坡：把 q_des 在 seconds 秒内从 q_from（起点关节角）平滑推到站姿。
    // smoothstep 两端速度为零，所以起步/到位都不会猛拉；用的时钟是 d->time（暂停时不走）。
    // from_label 只用于打印（"原姿态（直腿）" / "趴卧"）。
    void RampFrom(const std::vector<double> &q_from, double seconds, const char *from_label) {
        if (q_from.size() != q_des_.size()) {
            std::fprintf(stderr, "RampFrom：长度不一致（%zu vs %zu），忽略斜坡\n", q_from.size(),
                         q_des_.size());
            return;
        }
        q_from_ = q_from;
        ramp_ = std::max(0.0, seconds);
        std::printf("控制：目标由 %.2f s 斜坡从「%s」的关节角推到上面那组站姿（smoothstep；"
                    "控制器仍是同一条 PD 律，只是 q_des 随时间动）\n",
                    ramp_, from_label);
    }

    // 每步调用一次，写满 d->ctrl（斜坡模式下按 d->time 取当前目标；不改成员，所以是 const）
    void operator()(mjData *d) const {
        const double s = ramp_ > 0.0 ? Smoothstep(std::min(1.0, d->time / ramp_)) : 1.0;
        for (size_t i = 0; i < qadr_.size(); ++i) {
            const double qd =
                q_from_.empty() ? q_des_[i] : q_from_[i] + (q_des_[i] - q_from_[i]) * s;
            double tau = kp_ * (qd - d->qpos[qadr_[i]]) - kd_ * d->qvel[vadr_[i]];
            if (gravity_comp_)
                tau += d->qfrc_bias[vadr_[i]];
            d->ctrl[i] = tau;
        }
    }

private:
    static double Smoothstep(double u) { return u * u * (3.0 - 2.0 * u); }

    double kp_, kd_;
    bool gravity_comp_;
    std::vector<int> qadr_, vadr_;
    std::vector<double> q_des_;
    std::vector<double> q_from_; // 空 = 不做斜坡，直接把目标顶到站姿
    double ramp_ = 0.0;
};

} // namespace stand
