// 版权和许可证信息
// 版权所有 2021 DeepMind Technologies Limited
//
// 根据Apache许可证2.0版本授权
// 除非符合许可证要求，否则不得使用本文件
// 您可以在以下网址获取许可证副本
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，本软件按"原样"分发
// 没有任何明示或暗示的保证或条件，详见许可证

#ifndef MUJOCO_MJDATA_H_
#define MUJOCO_MJDATA_H_

#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjtnum.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjthread.h>

//---------------------------------- 基本类型定义 (mjt) -----------------------------------------

// 状态元素枚举
typedef enum mjtState_ {          // 状态元素标志位
    mjSTATE_TIME = 1 << 0,   // 时间
    mjSTATE_QPOS = 1 << 1,   // 位置
    mjSTATE_QVEL = 1 << 2,   // 速度
    mjSTATE_ACT = 1 << 3,   // 执行器激活状态
    mjSTATE_WARMSTART = 1 << 4,   // 用于热启动的加速度
    mjSTATE_CTRL = 1 << 5,   // 控制信号
    mjSTATE_QFRC_APPLIED = 1 << 6,   // 应用的广义力
    mjSTATE_XFRC_APPLIED = 1 << 7,   // 应用的笛卡尔力/扭矩
    mjSTATE_EQ_ACTIVE = 1 << 8,   // 约束启用/禁用状态
    mjSTATE_MOCAP_POS = 1 << 9,   // 运动捕捉体位置
    mjSTATE_MOCAP_QUAT = 1 << 10,  // 运动捕捉体方向
    mjSTATE_USERDATA = 1 << 11,  // 用户数据
    mjSTATE_PLUGIN = 1 << 12,  // 插件状态

    mjNSTATE = 13,     // 状态元素总数

    // 常用状态组合
    mjSTATE_PHYSICS = mjSTATE_QPOS | mjSTATE_QVEL | mjSTATE_ACT,  // 物理状态
    mjSTATE_FULLPHYSICS = mjSTATE_TIME | mjSTATE_PHYSICS | mjSTATE_PLUGIN,  // 完整物理状态
    mjSTATE_USER = mjSTATE_CTRL | mjSTATE_QFRC_APPLIED | mjSTATE_XFRC_APPLIED |
    mjSTATE_EQ_ACTIVE | mjSTATE_MOCAP_POS | mjSTATE_MOCAP_QUAT |
    mjSTATE_USERDATA,  // 用户相关状态
    mjSTATE_INTEGRATION = mjSTATE_FULLPHYSICS | mjSTATE_USER | mjSTATE_WARMSTART  // 积分相关状态
} mjtState;

// 警告类型枚举
typedef enum mjtWarning_ {   // 警告类型
    mjWARN_INERTIA = 0,   // 惯性矩阵接近奇异
    mjWARN_CONTACTFULL,        // 接触列表过多接触
    mjWARN_CNSTRFULL,          // 约束过多
    mjWARN_VGEOMFULL,          // 可视化几何体过多
    mjWARN_BADQPOS,            // qpos中存在非法数值
    mjWARN_BADQVEL,            // qvel中存在非法数值
    mjWARN_BADQACC,            // qacc中存在非法数值
    mjWARN_BADCTRL,            // ctrl中存在非法数值

    mjNWARNING                 // 警告类型总数
} mjtWarning;

// 内部计时器枚举
typedef enum mjtTimer_ {     // 内部计时器类型
    // 主要API计时
    mjTIMER_STEP = 0,   // 完整步骤
    mjTIMER_FORWARD,           // 前向动力学
    mjTIMER_INVERSE,           // 逆向动力学

    // 步骤分解计时
    mjTIMER_POSITION,          // 位置计算
    mjTIMER_VELOCITY,          // 速度计算
    mjTIMER_ACTUATION,         // 执行器计算
    mjTIMER_CONSTRAINT,        // 约束计算
    mjTIMER_ADVANCE,           // 积分方法（欧拉/隐式）

    // 位置计算分解
    mjTIMER_POS_KINEMATICS,    // 运动学计算（包含COM、肌腱、传动）
    mjTIMER_POS_INERTIA,       // 惯性计算
    mjTIMER_POS_COLLISION,     // 碰撞检测
    mjTIMER_POS_MAKE,          // 约束生成
    mjTIMER_POS_PROJECT,       // 约束投影

    // 碰撞检测分解
    mjTIMER_COL_BROAD,         // 粗略碰撞检测
    mjTIMER_COL_NARROW,        // 精确碰撞检测

    mjNTIMER                   // 计时器总数
} mjtTimer;

//---------------------------------- 接触结构定义 -------------------------------------------------

// 碰撞检测结果结构体
struct mjContact_ {                // 碰撞检测结果
    // 近场碰撞函数设置的参数
    mjtNum  dist;                    // 最近点间距离（负值表示穿透）
    mjtNum  pos[3];                  // 接触点位置（两几何体中间点）
    mjtNum  frame[9];                // 坐标系（0-2为法线方向，从geom[0]指向geom[1]）

    // mj_collideGeoms设置的参数
    mjtNum  includemargin;           // 包含阈值（当dist < margin - gap时包含）
    mjtNum  friction[5];             // 摩擦参数（切向1, 2, 自旋, 滚动1, 2）
    mjtNum  solref[mjNREF];          // 约束求解器参考（法线方向）
    mjtNum  solreffriction[mjNREF];  // 约束求解器参考（摩擦方向）
    mjtNum  solimp[mjNIMP];          // 约束求解器阻抗参数

    // 求解器内部存储
    mjtNum  mu;                      // 正则化锥摩擦系数（由mj_makeConstraint设置）
    mjtNum  H[36];                   // 锥Hessian矩阵（由mj_constraintUpdate设置）

    // mj_collideXXX设置的接触描述符
    int     dim;                     // 接触空间维度：1, 3, 4或6
    int     geom1;                   // 几何体1 ID（已弃用，使用geom[0]）
    int     geom2;                   // 几何体2 ID（已弃用，使用geom[1]）
    int     geom[2];                 // 几何体ID（-1表示柔性体）
    int     flex[2];                 // 柔性体ID（-1表示几何体）
    int     elem[2];                 // 元素ID（-1表示几何体或柔性体顶点）
    int     vert[2];                 // 顶点ID（-1表示几何体或柔性体元素）

    // 由mj_setContact或mj_instantiateContact设置的标志
    int     exclude;                 // 0: 包含，1: 间隙中，2: 融合，3: 无自由度

    // 由mj_instantiateContact计算的地址
    int     efc_address;             // 在efc中的地址（-1表示未包含）
};
typedef struct mjContact_ mjContact;

//---------------------------------- 诊断数据结构 -------------------------------------------------

// 警告统计结构体
struct mjWarningStat_ {      // 警告统计信息
    int     lastinfo;          // 最后一次警告的详细信息
    int     number;            // 警告触发次数
};
typedef struct mjWarningStat_ mjWarningStat;

// 计时器统计结构体
struct mjTimerStat_ {        // 计时器统计信息
    mjtNum  duration;          // 累计持续时间
    int     number;            // 调用次数
};
typedef struct mjTimerStat_ mjTimerStat;

// 求解器统计结构体
struct mjSolverStat_ {       // 每迭代求解器统计
    mjtNum  improvement;       // 成本降低量（按trace(M(qpos0))缩放）
    mjtNum  gradient;          // 梯度范数（仅原始问题，已缩放）
    mjtNum  lineslope;         // 线搜索中的斜率
    int     nactive;           // 活动约束数量
    int     nchange;           // 约束状态变化次数
    int     neval;             // 线搜索中的成本评估次数
    int     nupdate;           // 线搜索中的Cholesky更新次数
};
typedef struct mjSolverStat_ mjSolverStat;

//---------------------------------- 主数据结构 mjData -------------------------------------------------

struct mjData_ {
    // 常量大小
    size_t  narena;            // 内存池总大小（包含栈）
    size_t  nbuffer;           // 主缓冲区大小（字节）
    int     nplugin;           // 插件实例数量

    // 栈指针管理
    size_t  pstack;            // 栈中第一个可用字节
    size_t  pbase;             // 上次调用mj_markStack时的栈指针位置

    // 内存池指针
    size_t  parena;            // 内存池中第一个可用字节

    // 内存使用统计
    size_t  maxuse_stack;                       // 栈最大使用量（字节）
    size_t  maxuse_threadstack[mjMAXTHREAD];    // 每线程栈最大使用量（字节）
    size_t  maxuse_arena;                       // 内存池最大使用量（字节）
    int     maxuse_con;                         // 最大接触数量
    int     maxuse_efc;                         // 最大标量约束数量

    // 求解器统计信息
    mjSolverStat  solver[mjNISLAND * mjNSOLVER];  // 各岛各迭代的求解统计
    int           solver_nisland;               // 处理的岛数量
    int           solver_niter[mjNISLAND];      // 各岛的迭代次数
    int           solver_nnz[mjNISLAND];        // 各岛Hessian或efc_AR的非零元数量
    mjtNum        solver_fwdinv[2];             // 前向-逆向对比数据（qfrc, efc）

    // 诊断信息
    mjWarningStat warning[mjNWARNING];          // 警告统计数组
    mjTimerStat   timer[mjNTIMER];              // 计时统计数组

    // 变量大小
    int     ncon;              // 检测到的接触数量
    int     ne;                // 等式约束数量
    int     nf;                // 摩擦约束数量
    int     nl;                // 限制约束数量
    int     nefc;              // 总约束数量
    int     nJ;                // 约束雅可比矩阵非零元数量
    int     nA;                // 约束逆惯性矩阵非零元数量
    int     nisland;           // 检测到的约束岛数量

    // 全局属性
    mjtNum  time;              // 仿真时间
    mjtNum  energy[2];         // 势能，动能

    //-------------------- 信息头结束

    // 内存缓冲区
    void* buffer;            // 主缓冲区（所有指针都指向该区域）          (nbuffer字节)
    void* arena;             // 内存池+栈缓冲区                          (narena字节)

    //-------------------- 主要输入输出

    // 状态量
    mjtNum* qpos;              // 关节位置                                 (nq x 1)
    mjtNum* qvel;              // 关节速度                                 (nv x 1)
    mjtNum* act;               // 执行器激活状态                          (na x 1)
    mjtNum* qacc_warmstart;    // 热启动使用的加速度                      (nv x 1)
    mjtNum* plugin_state;      // 插件状态                                (npluginstate x 1)

    // 控制量
    mjtNum* ctrl;              // 控制输入                                (nu x 1)
    mjtNum* qfrc_applied;      // 应用的广义力                            (nv x 1)
    mjtNum* xfrc_applied;      // 应用的笛卡尔力/扭矩                     (nbody x 6)
    mjtByte* eq_active;        // 约束启用状态                            (neq x 1)

    // 运动捕捉数据
    mjtNum* mocap_pos;         // 运动捕捉体位置                          (nmocap x 3)
    mjtNum* mocap_quat;        // 运动捕捉体四元数                        (nmocap x 4)

    // 动力学量
    mjtNum* qacc;              // 关节加速度                              (nv x 1)
    mjtNum* act_dot;           // 执行器激活状态的时间导数                (na x 1)

    // 用户数据
    mjtNum* userdata;          // 用户数据（引擎不修改）                  (nuserdata x 1)

    // 传感器数据
    mjtNum* sensordata;        // 传感器数据数组                          (nsensordata x 1)

    // 插件相关
    int* plugin;         // m->plugin的副本（用于删除）             (nplugin x 1)
    uintptr_t* plugin_data;    // 指向插件管理的数据结构                  (nplugin x 1)

    //-------------------- 位置相关量

    // 由mj_fwdPosition/mj_kinematics计算
    mjtNum* xpos;              // 物体坐标系位置                          (nbody x 3)
    mjtNum* xquat;             // 物体坐标系四元数                        (nbody x 4)
    mjtNum* xmat;              // 物体坐标系旋转矩阵                      (nbody x 9)
    mjtNum* xipos;             // 物体质心位置                            (nbody x 3)
    mjtNum* ximat;             // 物体惯性矩阵旋转                        (nbody x 9)
    mjtNum* xanchor;           // 关节锚点位置                            (njnt x 3)
    mjtNum* xaxis;             // 关节轴方向                              (njnt x 3)
    mjtNum* geom_xpos;         // 几何体位置                              (ngeom x 3)
    mjtNum* geom_xmat;         // 几何体旋转矩阵                          (ngeom x 9)
    mjtNum* site_xpos;         // 站点位置                                (nsite x 3)
    mjtNum* site_xmat;         // 站点旋转矩阵                            (nsite x 9)
    mjtNum* cam_xpos;          // 相机位置                                (ncam x 3)
    mjtNum* cam_xmat;          // 相机旋转矩阵                            (ncam x 9)
    mjtNum* light_xpos;        // 光源位置                                (nlight x 3)
    mjtNum* light_xdir;        // 光源方向                                (nlight x 3)

    // 由mj_fwdPosition/mj_comPos计算
    mjtNum* subtree_com;       // 各子树的质心                            (nbody x 3)
    mjtNum* cdof;              // 基于质心的自由度运动轴（旋转/平移）     (nv x 6)
    mjtNum* cinert;            // 基于质心的惯性和质量                    (nbody x 10)

    // 由mj_fwdPosition/mj_flex计算
    mjtNum* flexvert_xpos;     // 柔性体顶点位置                          (nflexvert x 3)
    mjtNum* flexelem_aabb;     // 柔性体元素包围盒（中心，尺寸）          (nflexelem x 6)
    int* flexedge_J_rownnz; // 雅可比矩阵行非零元数量                 (nflexedge x 1)
    int* flexedge_J_rowadr; // 行起始地址（在colind数组中）           (nflexedge x 1)
    int* flexedge_J_colind; // 雅可比矩阵列索引                       (nflexedge x nv)
    mjtNum* flexedge_J;        // 柔性体边雅可比矩阵                     (nflexedge x nv)
    mjtNum* flexedge_length;   // 柔性体边长度                           (nflexedge x 1)

    // 由mj_fwdPosition/mj_tendon计算
    int* ten_wrapadr;       // 肌腱路径起始地址                       (ntendon x 1)
    int* ten_wrapnum;       // 肌腱路径中的包裹点数                   (ntendon x 1)
    int* ten_J_rownnz;      // 雅可比矩阵行非零元数量                 (ntendon x 1)
    int* ten_J_rowadr;      // 行起始地址（在colind数组中）           (ntendon x 1)
    int* ten_J_colind;      // 雅可比矩阵列索引                       (ntendon x nv)
    mjtNum* ten_J;             // 肌腱雅可比矩阵                         (ntendon x nv)
    mjtNum* ten_length;        // 肌腱长度                               (ntendon x 1)
    int* wrap_obj;          // 包裹对象ID（-1: 站点，-2: 滑轮）       (nwrap x 2)
    mjtNum* wrap_xpos;         // 所有路径中的笛卡尔点                   (nwrap x 6)

    // 由mj_fwdPosition/mj_transmission计算
    mjtNum* actuator_length;   // 执行器长度                             (nu x 1)
    int* moment_rownnz;     // actuator_moment行非零元数量            (nu x 1)
    int* moment_rowadr;     // 行起始地址（在colind数组中）           (nu x 1)
    int* moment_colind;     // 雅可比矩阵列索引                       (nJmom x 1)
    mjtNum* actuator_moment;   // 执行器力矩                             (nJmom x 1)

    // 由mj_fwdPosition/mj_crb计算
    mjtNum* crb;               // 复合刚体惯性和质量                     (nbody x 10)
    mjtNum* qM;                // 总惯性矩阵（稀疏存储）                 (nM x 1)

    // 由mj_fwdPosition/mj_factorM计算
    mjtNum* qLD;               // M矩阵的L'*D*L分解（稀疏存储）          (nM x 1)
    mjtNum* qLDiagInv;         // 1/diag(D)                             (nv x 1)

    // 由mj_collisionTree计算
    mjtNum* bvh_aabb_dyn;     // 全局动态包围盒（中心，尺寸）           (nbvhdynamic x 6)
    mjtByte* bvh_active;       // 碰撞检测标记                          (nbvh x 1)

    //-------------------- 位置和速度相关量

    // 由mj_fwdVelocity计算
    mjtNum* flexedge_velocity; // 柔性体边速度                          (nflexedge x 1)
    mjtNum* ten_velocity;      // 肌腱速度                               (ntendon x 1)
    mjtNum* actuator_velocity; // 执行器速度                             (nu x 1)

    // 由mj_fwdVelocity/mj_comVel计算
    mjtNum* cvel;              // 基于质心的速度（旋转/平移）            (nbody x 6)
    mjtNum* cdof_dot;          // cdof的时间导数（旋转/平移）            (nv x 6)

    // 由mj_fwdVelocity/mj_rne计算（无加速度）
    mjtNum* qfrc_bias;         // 科里奥利力和离心力                     (nv x 1)

    // 由mj_fwdVelocity/mj_passive计算
    mjtNum* qfrc_spring;       // 弹簧力                                 (nv x 1)
    mjtNum* qfrc_damper;       // 阻尼力                                 (nv x 1)
    mjtNum* qfrc_gravcomp;     // 重力补偿力                             (nv x 1)
    mjtNum* qfrc_fluid;        // 流体力                                 (nv x 1)
    mjtNum* qfrc_passive;      // 总被动力                               (nv x 1)

    // 由mj_sensorVel/mj_subtreeVel计算（需要时）
    mjtNum* subtree_linvel;    // 子树质心线速度                         (nbody x 3)
    mjtNum* subtree_angmom;    // 子树质心角动量                         (nbody x 3)

    // 由mj_Euler或mj_implicit计算
    mjtNum* qH;                // 修改后的M矩阵分解（L'*D*L）            (nM x 1)
    mjtNum* qHDiagInv;         // 修改后的M矩阵的1/diag(D)              (nv x 1)

    // 由mj_resetData计算
    int* B_rownnz;          // 体-自由度：每行非零元数量              (nbody x 1)
    int* B_rowadr;          // 体-自由度：行起始地址（在B_colind中） (nbody x 1)
    int* B_colind;          // 体-自由度：列索引                     (nB x 1)
    int* M_rownnz;          // 惯性矩阵：每行非零元数量               (nv x 1)
    int* M_rowadr;          // 惯性矩阵：行起始地址（在M_colind中）  (nv x 1)
    int* M_colind;          // 惯性矩阵：列索引                      (nM x 1)
    int* mapM2M;            // 从M（旧格式）到M（CSR）的索引映射      (nM x 1)
    int* C_rownnz;          // 约简自由度：每行非零元数量            (nv x 1)
    int* C_rowadr;          // 约简自由度：行起始地址（在C_colind中）(nv x 1)
    int* C_colind;          // 约简自由度：列索引                    (nC x 1)
    int* mapM2C;            // 从M到C的索引映射                      (nC x 1)
    int* D_rownnz;          // 自由度：每行非零元数量                (nv x 1)
    int* D_rowadr;          // 自由度：行起始地址（在D_colind中）    (nv x 1)
    int* D_diag;            // 自由度：对角线元素索引                (nv x 1)
    int* D_colind;          // 自由度：列索引                       (nD x 1)
    int* mapM2D;            // 从M到D的索引映射                     (nD x 1)
    int* mapD2M;            // 从D到M的索引映射                     (nM x 1)

    // 由mj_implicit/mj_derivative计算
    mjtNum* qDeriv;            // 被动力+执行器力-偏置力对速度的导数    (nD x 1)

    // 由mj_implicit/mju_factorLUSparse计算
    mjtNum* qLU;               // (qM - dt*qDeriv)的稀疏LU分解          (nD x 1)

    //-------------------- 位置、速度、控制/加速度相关量

    // 由mj_fwdActuation计算
    mjtNum* actuator_force;    // 执行器空间中的力                      (nu x 1)
    mjtNum* qfrc_actuator;     // 执行器产生的广义力                    (nv x 1)

    // 由mj_fwdAcceleration计算
    mjtNum* qfrc_smooth;       // 总无约束力                           (nv x 1)
    mjtNum* qacc_smooth;       // 无约束加速度                         (nv x 1)

    // 由mj_fwdConstraint/mj_inverse计算
    mjtNum* qfrc_constraint;   // 约束力                               (nv x 1)

    // 由mj_inverse计算
    mjtNum* qfrc_inverse;      // 净外力（应等于qfrc_applied + J'*xfrc_applied + qfrc_actuator）(nv x 1)

    // 由mj_sensorAcc/mj_rnePostConstraint计算（需要时）
    mjtNum* cacc;              // 基于质心的加速度                      (nbody x 6)
    mjtNum* cfrc_int;          // 与父体的相互作用力                   (nbody x 6)
    mjtNum* cfrc_ext;          // 外部作用力                           (nbody x 6)

    //-------------------- 内存池分配：位置相关

    // 由mj_collision生成
    mjContact* contact;        // 所有检测到的接触数组                  (ncon x 1)

    // 由mj_makeConstraint生成
    int* efc_type;          // 约束类型（mjtConstraint枚举）         (nefc x 1)
    int* efc_id;            // 对应对象的ID                         (nefc x 1)
    int* efc_J_rownnz;      // 约束雅可比行非零元数量               (nefc x 1)
    int* efc_J_rowadr;      // 行起始地址（在colind数组中）         (nefc x 1)
    int* efc_J_rowsuper;    // 超节点中的后续行数                   (nefc x 1)
    int* efc_J_colind;      // 约束雅可比列索引                     (nJ x 1)
    int* efc_JT_rownnz;     // 转置雅可比行非零元数量               (nv x 1)
    int* efc_JT_rowadr;     // 转置雅可比行起始地址                 (nv x 1)
    int* efc_JT_rowsuper;   // 转置超节点中的后续行数               (nv x 1)
    int* efc_JT_colind;     // 转置雅可比列索引                     (nJ x 1)
    mjtNum* efc_J;             // 约束雅可比矩阵                       (nJ x 1)
    mjtNum* efc_JT;            // 转置约束雅可比矩阵                   (nJ x 1)
    mjtNum* efc_pos;           // 约束位置（等式约束，接触）            (nefc x 1)
    mjtNum* efc_margin;        // 包含边距（接触约束）                  (nefc x 1)
    mjtNum* efc_frictionloss;  // 摩擦损耗                             (nefc x 1)
    mjtNum* efc_diagApprox;    // 矩阵A对角线的近似值                  (nefc x 1)
    mjtNum* efc_KBIP;          // 刚度、阻尼、阻抗参数                  (nefc x 4)
    mjtNum* efc_D;             // 约束质量                             (nefc x 1)
    mjtNum* efc_R;             // 约束质量逆                           (nefc x 1)
    int* tendon_efcadr;     // 涉及肌腱的首个efc地址（-1表示无）    (ntendon x 1)

    // 由mj_island生成
    int* dof_island;        // 自由度所属岛ID（-1表示无）           (nv x 1)
    int* island_dofnum;     // 岛内自由度数量                       (nisland x 1)
    int* island_dofadr;     // 岛内自由度索引起始地址               (nisland x 1)
    int* island_dofind;     // 岛内自由度索引（-1表示无）           (nv x 1)
    int* dof_islandind;     // 自由度在岛内的索引                   (nv x 1)
    int* efc_island;        // 约束所属岛ID                         (nefc x 1)
    int* island_efcnum;     // 岛内约束数量                         (nisland x 1)
    int* island_efcadr;     // 岛内约束索引起始地址                 (nisland x 1)
    int* island_efcind;     // 岛内约束索引                         (nefc x 1)

    // 由mj_projectConstraint生成（PGS求解器）
    int* efc_AR_rownnz;     // AR矩阵行非零元数量                   (nefc x 1)
    int* efc_AR_rowadr;     // AR行起始地址（在colind数组中）       (nefc x 1)
    int* efc_AR_colind;     // AR矩阵列索引                         (nA x 1)
    mjtNum* efc_AR;            // J*inv(M)*J' + R 矩阵                 (nA x 1)

    //-------------------- 内存池分配：位置、速度相关

    // 由mj_fwdVelocity/mj_referenceConstraint生成
    mjtNum* efc_vel;           // 约束空间速度：J*qvel                 (nefc x 1)
    mjtNum* efc_aref;          // 参考伪加速度                        (nefc x 1)

    //-------------------- 内存池分配：位置、速度、控制/加速度相关

    // 由mj_fwdConstraint/mj_inverse生成
    mjtNum* efc_b;             // 线性成本项：J*qacc_smooth - aref    (nefc x 1)
    mjtNum* efc_force;         // 约束空间中的力                      (nefc x 1)
    int* efc_state;         // 约束状态（mjtConstraintState枚举）  (nefc x 1)

    // 线程池指针
    uintptr_t threadpool;      // 线程池指针
};
typedef struct mjData_ mjData;

//---------------------------------- 回调函数类型定义 ---------------------------------------

// 通用MuJoCo函数类型
typedef void (*mjfGeneric)(const mjModel* m, mjData* d);

// 接触过滤器类型（返回1表示忽略，0表示碰撞）
typedef int (*mjfConFilt)(const mjModel* m, mjData* d, int geom1, int geom2);

// 传感器模拟函数类型
typedef void (*mjfSensor)(const mjModel* m, mjData* d, int stage);

// 计时器函数类型
typedef mjtNum(*mjfTime)(void);

// 执行器动态、增益、偏置函数类型
typedef mjtNum(*mjfAct)(const mjModel* m, const mjData* d, int id);

// 碰撞检测函数类型
typedef int (*mjfCollision)(const mjModel* m, const mjData* d,
    mjContact* con, int g1, int g2, mjtNum margin);

#endif  // MUJOCO_MJDATA_H_
