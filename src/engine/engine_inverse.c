// Copyright 2021 DeepMind Technologies Limited
//
// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.

#include "engine/engine_inverse.h"

#include <stddef.h>

#include <mujoco/mjdata.h>
#include <mujoco/mjmacro.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjsan.h>  // IWYU pragma: keep
#include "engine/engine_collision_driver.h"
#include "engine/engine_core_constraint.h"
#include "engine/engine_core_smooth.h"
#include "engine/engine_derivative.h"
#include "engine/engine_io.h"
#include "engine/engine_macro.h"
#include "engine/engine_forward.h"
#include "engine/engine_sensor.h"
#include "engine/engine_support.h"
#include "engine/engine_util_blas.h"
#include "engine/engine_util_errmem.h"
#include "engine/engine_util_sparse.h"

// 位置相关计算
void mj_invPosition(const mjModel* m, mjData* d) {
    TM_START1;
    TM_START;

    // 计算运动学相关量
    mj_kinematics(m, d);  // 1.计算正向运动学
    mj_comPos(m, d);      // 计算质心位置
    mj_camlight(m, d);    // 更新相机和光源
    mj_flex(m, d);        // 处理柔性体
    mj_tendon(m, d);      // 3.计算肌腱长度和力矩
    TM_END(mjTIMER_POS_KINEMATICS);

    // 5.计算复合刚体惯量（内部已计时），同时形成关节空间惯量矩阵
    mj_crb(m, d);        // 内部计时标记为POS_INERTIA
    // 6.稀疏分解分解惯性矩阵（内部已计时）
    mj_factorM(m, d);    // 内部计时标记为POS_INERTIA

    // 处理碰撞检测（内部已计时）
    mj_collision(m, d);  // 内部计时标记为POS_COLLISION

    // 8.创建约束系统
    TM_RESTART;
    mj_makeConstraint(m, d);
    TM_END(mjTIMER_POS_MAKE);

    // 处理传动系统
    TM_RESTART;
    mj_transmission(m, d);
    TM_ADD(mjTIMER_POS_KINEMATICS);

    TM_END1(mjTIMER_POSITION);
}



// 速度相关计算
void mj_invVelocity(const mjModel* m, mjData* d) {
    // 前向速度计算
    mj_fwdVelocity(m, d);
}



// 将离散时间qacc转换为连续时间qacc
static void mj_discreteAcc(const mjModel* m, mjData* d) {
    int nv = m->nv, nM = m->nM, nD = m->nD, dof_damping;
    mjtNum* qacc = d->qacc;

    mj_markStack(d);
    mjtNum* qfrc = mjSTACKALLOC(d, nv, mjtNum);

    // 根据选择的积分器类型进行处理
    switch ((mjtIntegrator)m->opt.integrator) {
    case mjINT_RK4:
        // RK4不支持离散逆动力学
        mjERROR("RK4积分器不支持离散逆动力学");
        return;

    case mjINT_EULER:
        // 检查自由度阻尼（当禁用标志未设置时）
        dof_damping = 0;
        if (!mjDISABLED(mjDSBL_EULERDAMP)) {
            for (int i = 0; i < nv; i++) {
                if (m->dof_damping[i] > 0) {
                    dof_damping = 1;
                    break;
                }
            }
        }

        // 如果禁用或没有自由度阻尼则直接返回
        if (!dof_damping) {
            mj_freeStack(d);
            return;
        }

        // 计算 qfrc = (M + h*diag(B)) * qacc
        mj_mulM(m, d, qfrc, qacc);
        for (int i = 0; i < nv; i++) {
            qfrc[i] += m->opt.timestep * m->dof_damping[i] * d->qacc[i];
        }
        break;

    case mjINT_IMPLICIT:
        // 计算速度导数项
        mjd_smooth_vel(m, d, /* 计算偏置项标志 */ 1);

        // 将qLU初始化为qM
        for (int i = 0; i < nD; i++) {
            d->qLU[i] = d->qM[d->mapM2D[i]];
        }

        // 更新qLU = qM - dt*qDeriv2
        mju_addToScl(d->qLU, d->qDeriv, -m->opt.timestep, m->nD);

        // 计算qfrc = qLU * qacc
        mju_mulMatVecSparse(qfrc, d->qLU, qacc, nv,
            d->D_rownnz, d->D_rowadr, d->D_colind, /*行超节点*/NULL);
        break;

    case mjINT_IMPLICITFAST:
        // 计算解析导数qDeriv（跳过rne导数）
        mjd_smooth_vel(m, d, /* 计算偏置项标志 */ 0);

        // 保存质量矩阵
        mjtNum* qMsave = mjSTACKALLOC(d, m->nM, mjtNum);
        mju_copy(qMsave, d->qM, m->nM);

        // 更新M = M - dt*qDeriv（仅处理非零元素）
        mjtNum* qDerivReduced = mjSTACKALLOC(d, m->nM, mjtNum);
        for (int i = 0; i < nM; i++) {
            qDerivReduced[i] = d->qDeriv[d->mapD2M[i]];
        }
        mju_addToScl(d->qM, qDerivReduced, -m->opt.timestep, m->nM);

        // 计算qfrc = (M - dt*qDeriv) * qacc
        mj_mulM(m, d, qfrc, qacc);

        // 恢复原始质量矩阵
        mju_copy(d->qM, qMsave, m->nM);
        break;
    }

    // 求解qacc：qfrc = M * qacc
    mj_solveM(m, d, qacc, qfrc, 1);

    mj_freeStack(d);
}



// 逆约束求解器
void mj_invConstraint(const mjModel* m, mjData* d) {
    TM_START;
    int nefc = d->nefc;

    // 无约束时清空并返回
    if (!nefc) {
        mju_zero(d->qfrc_constraint, m->nv);
        TM_END(mjTIMER_CONSTRAINT);
        return;
    }

    mj_markStack(d);
    mjtNum* jar = mjSTACKALLOC(d, nefc, mjtNum);

    // 计算 jar = Jac*qacc - aref
    mj_mulJacVec(m, d, jar, d->qacc);
    mju_subFrom(jar, d->efc_aref, nefc);

    // 调用约束更新函数
    mj_constraintUpdate(m, d, jar, NULL, 0);

    mj_freeStack(d);
    TM_END(mjTIMER_CONSTRAINT);
}



// 带跳过的逆动力学计算，skipstage参数为mjtStage类型
void mj_inverseSkip(const mjModel* m, mjData* d,
    int skipstage, int skipsensor) {
    TM_START;
    mj_markStack(d);
    mjtNum* qacc;
    int nv = m->nv;

    // 位置相关阶段
    if (skipstage < mjSTAGE_POS) {
        mj_invPosition(m, d);
        if (!skipsensor) {
            mj_sensorPos(m, d);  // 更新位置传感器
        }
        if (mjENABLED(mjENBL_ENERGY)) {
            mj_energyPos(m, d);  // 9.计算位置能量
        }
    }

    // 速度相关阶段
    if (skipstage < mjSTAGE_VEL) {
        mj_invVelocity(m, d);
        if (!skipsensor) {
            mj_sensorVel(m, d);  // 更新速度传感器
        }
        if (mjENABLED(mjENBL_ENERGY)) {
            mj_energyVel(m, d);  // 12.计算速度能量
        }
    }

    // 处理离散时间积分
    if (mjENABLED(mjENBL_INVDISCRETE)) {
        // 保存当前qacc
        qacc = mjSTACKALLOC(d, nv, mjtNum);
        mju_copy(qacc, d->qacc, nv);

        // 15.原地修改qacc
        mj_discreteAcc(m, d);
    }

    // 加速度相关阶段
    // 如果无约束就直接返回了
    mj_invConstraint(m, d);
    mj_rne(m, d, 1, d->qfrc_inverse);  // 递归牛顿-欧拉算法
    if (!skipsensor) {
        mj_sensorAcc(m, d);  // 18.更新加速度传感器
    }

    // RNE 计算结束后，qfrc_inverse 中已经包含科里奥利力、广义惯性力
    // 19.计算最终逆向动力学力：qfrc_inverse += 惯性力 - 被动力 - 约束力
    // 被动力在计算速度的阶段完成计算，正向动力学中没有直接分步计算约束力
    // 也就是说在这一管线中，被动力在正向的时候算好了，逆向时没有重新计算；而约束力是在逆向动力学中计算的
    // 考虑到正向动力学和逆向动力学在计算中存在相同的部分，这并不奇怪
    for (int i = 0; i < nv; i++) {
        d->qfrc_inverse[i] += m->dof_armature[i] * d->qacc[i]
            - d->qfrc_passive[i] - d->qfrc_constraint[i];
    }

    // 恢复原始qacc（如果启用了离散时间积分）
    if (mjENABLED(mjENBL_INVDISCRETE)) {
        mju_copy(d->qacc, qacc, nv);
    }

    mj_freeStack(d);
    TM_END(mjTIMER_INVERSE);
}



// 逆动力学主入口函数
void mj_inverse(const mjModel* m, mjData* d) {
    // 调用完整逆动力学计算，不跳过任何阶段
    mj_inverseSkip(m, d, mjSTAGE_NONE, 0);
}


// 比较正向动力学和逆向动力学，不改变正向动力学的结果
//    fwdinv[0] = 约束力的范数（正向的qfrc_constraint - 逆向的qfrc_constraint）
//    fwdinv[1] = 应用力的范数（正向的qrc_applied - qfrc_inverse）
void mj_compareFwdInv(const mjModel* m, mjData* d) {
    int nv = m->nv, nefc = d->nefc;
    mjtNum* qforce, * dif, * save_qfrc_constraint, * save_efc_force;

    // 清除结果，如果没有约束则直接返回
    d->solver_fwdinv[0] = d->solver_fwdinv[1] = 0;
    if (!nefc) {
        return;
    }

    // 分配内存
    mj_markStack(d);
    qforce = mjSTACKALLOC(d, nv, mjtNum);
    dif = mjSTACKALLOC(d, nv, mjtNum);
    save_qfrc_constraint = mjSTACKALLOC(d, nv, mjtNum);
    save_efc_force = mjSTACKALLOC(d, nefc, mjtNum);

    // 计算 qforce = qfrc_applied + J'*xfrc_applied + qfrc_actuator
    // 该结果应等于逆向动力学的输出
    mju_add(qforce, d->qfrc_applied, d->qfrc_actuator, nv);
    mj_xfrcAccumulate(m, d, qforce);

    // 保存即将被修改的正向动力学结果
    mju_copy(save_qfrc_constraint, d->qfrc_constraint, nv);
    mju_copy(save_efc_force, d->efc_force, nefc);

    // 执行逆向动力学，不更新位置和速度
    // 参数1: 不重新计算传感器和能量
    mj_inverseSkip(m, d, mjSTAGE_VEL, 1);

    // 计算统计量
    mju_sub(dif, save_qfrc_constraint, d->qfrc_constraint, nv);
    d->solver_fwdinv[0] = mju_norm(dif, nv);
    mju_sub(dif, qforce, d->qfrc_inverse, nv);
    d->solver_fwdinv[1] = mju_norm(dif, nv);

    // 恢复正向动力学结果
    mju_copy(d->qfrc_constraint, save_qfrc_constraint, nv);
    mju_copy(d->efc_force, save_efc_force, nefc);

    mj_freeStack(d);
}
