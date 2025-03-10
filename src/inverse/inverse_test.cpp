#include <iostream>
#include <array>
#include <cmath>
#include <cstdlib>
#include <cstring>
#include <ctime>
#include "mujoco/mujoco.h"


int main() {
    std::array<char, 1024> error;
    mjModel* model = mj_loadXML("test.xml", nullptr, error.data(), error.size());
    if (!model) {
        std::cerr << "Model loading error: " << error.data() << std::endl;
        return 1;
    }

    mjData* data = mj_makeData(model);
    const int nv = model->nv;
    const int nbody = model->nbody;
    const double tolerance = 1e-6;

    // ================== 参数设置 ==================
    const double total_time = 1.0;
    const double dt = model->opt.timestep;
    const int n_steps = static_cast<int>(total_time / dt);

    // 初始化随机广义力
    double* applied_force = new double[nv];
    double* xfrc_force = new double[nbody * 6];
    double* qfrc_actuator = new double[nv];
    std::srand(static_cast<unsigned>(time(nullptr)));

    // ================== 主仿真循环 ==================
    double max_force_err = 0.0;
    double max_constraint_err = 0.0;

    // 分配内存
    mjtNum* expected_force = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * nv));
    mjtNum* save_qfrc_constraint = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * nv));
    mjtNum* dif = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * nv));

    for (int i = 0; i < n_steps; ++i) {

        // 我们让所有涉及的外力每个仿真时刻都有变化
        for (int k = 0; k < nv; ++k) {
            applied_force[k] = 0.4 * (std::rand() / static_cast<double>(RAND_MAX) - 0.5);
        }
        for (int k = 0; k < nbody * 6; ++k) {
            xfrc_force[k] = 0.8 * (std::rand() / static_cast<double>(RAND_MAX) - 0.5);
        }
        for (int k = 0; k < nv; ++k) {
            qfrc_actuator[k] = 0.4 * (std::rand() / static_cast<double>(RAND_MAX) - 0.5);
        }

        // 应用广义力，这个力应当是额外附加的，因此不用处理其他的力
        std::copy(applied_force, applied_force + nv, data->qfrc_applied);
        std::copy(xfrc_force, xfrc_force + nbody * 6, data->xfrc_applied);
        std::copy(qfrc_actuator, qfrc_actuator + nv, data->qfrc_actuator);

        // ---------- 正向计算阶段 ----------

        mj_checkPos(model, data);
        mj_checkVel(model, data);
        mj_forward(model, data);
        mj_checkAcc(model, data);
        data->solver_fwdinv[0] = data->solver_fwdinv[1] = 0;

        // ---------- 逆动力学验证阶段 ----------
        
        // 计算预期力，确保和逆向动力学得到的表达式完全一致
        mju_add(expected_force, data->qfrc_applied, data->qfrc_actuator, nv);
        mj_xfrcAccumulate(model, data, expected_force);

        // 下面的代码等效于函数 mj_xfrcAccumulate(model, data, expected_force)
        // 不直接调用是为了避免 LNK1120 和 LNK2019 发生
        // 测试发现，只要将调用的函数在声明时前缀 MJAPI ，并且在 mujoco.h 中添加定义即可
        // 这种方法应当可以应用于自己新定义的任何函数
        //for (int j = 1; j < model->nbody; j++) {
        //    if (!mju_isZero(data->xfrc_applied + 6 * j, 6)) {
        //        mj_applyFT(model, data, data->xfrc_applied + 6 * j, 
        //            data->xfrc_applied + 6 * j + 3, data->xipos + 3 * j, j, expected_force);
        //    }
        //}
        
        // 两个注意点，一个是 nefc 不能实现定义，避免 nefc 非常量导致的错误
        // 一个是内存分配必须注意位置，否则可能因为约束在前向计算种发生变化导致错误
        mjtNum* save_efc_force = static_cast<mjtNum*>(malloc(sizeof(mjtNum) * data->nefc));

        mju_copy(save_qfrc_constraint, data->qfrc_constraint, nv);
        mju_copy(save_efc_force, data->efc_force, data->nefc);

        mj_inverseSkip(model, data, mjSTAGE_VEL, 1);

        // 计算统计量
        mju_sub(dif, save_qfrc_constraint, data->qfrc_constraint, nv);
        data->solver_fwdinv[0] = mju_norm(dif, nv);
        mju_sub(dif, expected_force, data->qfrc_inverse, nv);
        data->solver_fwdinv[1] = mju_norm(dif, nv);

        // 恢复正向动力学结果
        mju_copy(data->qfrc_constraint, save_qfrc_constraint, nv);
        mju_copy(data->efc_force, save_efc_force, data->nefc);

        // 验证结果
        max_force_err = fmax(max_force_err, data->solver_fwdinv[1]);
        max_constraint_err = fmax(max_constraint_err, data->solver_fwdinv[0]);

        mj_RungeKutta(model, data, 4);

        free(save_efc_force);
    }

    // 释放资源
    free(save_qfrc_constraint);
    free(dif);
    free(expected_force);

    // ================== 结果输出 ==================
    std::cout << "==== 逆动力学验证 ====\n";
    std::cout << "最大外力误差: " << max_force_err
        << (max_force_err < tolerance ? " (通过)" : " (失败)") << std::endl ;
    std::cout << "最大约束力误差: " << max_constraint_err
        << (max_constraint_err < tolerance ? " (通过)" : " (失败)") << std::endl;

    // 资源释放
    delete[] applied_force;
    delete[] xfrc_force;
    delete[] qfrc_actuator;
    mj_deleteData(data);
    mj_deleteModel(model);
    return 0;
}
