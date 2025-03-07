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

#ifndef MUJOCO_MJMODEL_H_
#define MUJOCO_MJMODEL_H_

#include <stddef.h>
#include <stdint.h>

#include <mujoco/mjtnum.h>

// 全局常量
#define mjPI            3.14159265358979323846                // 圆周率π
#define mjMAXVAL        1E+10     // qpos、qvel、qacc中的最大值
#define mjMINMU         1E-5      // 摩擦系数的最小值
#define mjMINIMP        0.0001    // 约束阻抗的最小值
#define mjMAXIMP        0.9999    // 约束阻抗的最大值
#define mjMAXCONPAIR    50        // 每对几何体的最大接触数
#define mjMAXTREEDEPTH  50        // 包围体层次结构的最大深度
#define mjMAXFLEXNODES  27        // 柔性体的最大节点数


//---------------------------------- 尺寸定义 ---------------------------------------------------------

#define mjNEQDATA       11        // eq_data字段的数量
#define mjNDYN          10        // 执行器动力学参数的数量
#define mjNGAIN         10        // 执行器增益参数的数量
#define mjNBIAS         10        // 执行器偏置参数的数量
#define mjNFLUID        12        // 流体交互参数的数量
#define mjNREF          2         // 求解器参考参数的数量
#define mjNIMP          5         // 求解器阻抗参数的数量
#define mjNSOLVER       200       // mjData.solver数组的大小
#define mjNISLAND       20        // mjData.solver数组的数量


//---------------------------------- 枚举类型 (mjt) ----------------------------------------------

typedef enum mjtDisableBit_ {     // 禁用默认功能的位标志
    mjDSBL_CONSTRAINT = 1 << 0,     // 禁用整个约束求解器
    mjDSBL_EQUALITY = 1 << 1,     // 禁用等式约束
    mjDSBL_FRICTIONLOSS = 1 << 2,     // 禁用关节和肌腱的摩擦损失约束
    mjDSBL_LIMIT = 1 << 3,     // 禁用关节和肌腱的极限约束
    mjDSBL_CONTACT = 1 << 4,     // 禁用接触约束
    mjDSBL_PASSIVE = 1 << 5,     // 禁用被动力
    mjDSBL_GRAVITY = 1 << 6,     // 禁用重力
    mjDSBL_CLAMPCTRL = 1 << 7,     // 将控制信号限制在指定范围内
    mjDSBL_WARMSTART = 1 << 8,     // 禁用约束求解器的热启动
    mjDSBL_FILTERPARENT = 1 << 9,     // 移除与父物体的碰撞
    mjDSBL_ACTUATION = 1 << 10,    // 禁用执行器力
    mjDSBL_REFSAFE = 1 << 11,    // 积分器安全：确保ref[0] >= 2*时间步长
    mjDSBL_SENSOR = 1 << 12,    // 禁用传感器
    mjDSBL_MIDPHASE = 1 << 13,    // 禁用中阶段碰撞过滤
    mjDSBL_EULERDAMP = 1 << 14,    // 在欧拉积分器中隐式积分关节阻尼
    mjDSBL_AUTORESET = 1 << 15,    // 检测到数值问题时自动重置
    mjDSBL_NATIVECCD = 1 << 16,    // 禁用原生凸碰撞检测

    mjNDISABLE = 17        // 禁用标志的数量
} mjtDisableBit;


typedef enum mjtEnableBit_ {      // 启用可选功能的位标志
    mjENBL_OVERRIDE = 1 << 0,     // 覆盖接触参数
    mjENBL_ENERGY = 1 << 1,     // 启用能量计算
    mjENBL_FWDINV = 1 << 2,     // 记录求解器统计信息
    mjENBL_INVDISCRETE = 1 << 3,     // 离散时间逆向动力学
    // 实验性功能：
    mjENBL_MULTICCD = 1 << 4,     // 多点凸碰撞检测
    mjENBL_ISLAND = 1 << 5,     // 约束岛屿发现

    mjNENABLE = 6         // 启用标志的数量
} mjtEnableBit;


typedef enum mjtJoint_ {          // 自由度类型
    mjJNT_FREE = 0,        // 全局位置和方向（四元数，7自由度）
    mjJNT_BALL,                     // 相对于父物体的方向（四元数，4自由度）
    mjJNT_SLIDE,                    // 沿物体固定轴的滑动距离（1自由度）
    mjJNT_HINGE                     // 绕物体固定轴的旋转角度（弧度，1自由度）
} mjtJoint;


typedef enum mjtGeom_ {           // 几何形状类型
    // 常规几何类型
    mjGEOM_PLANE = 0,        // 平面
    mjGEOM_HFIELD,                  // 高度场
    mjGEOM_SPHERE,                  // 球体
    mjGEOM_CAPSULE,                 // 胶囊体
    mjGEOM_ELLIPSOID,               // 椭球体
    mjGEOM_CYLINDER,                // 圆柱体
    mjGEOM_BOX,                     // 立方体
    mjGEOM_MESH,                    // 网格
    mjGEOM_SDF,                     // 有符号距离场

    mjNGEOMTYPES,                   // 常规几何类型的数量

    // 仅用于渲染的几何类型：不在mjModel中使用，不计入mjNGEOMTYPES
    mjGEOM_ARROW = 100,      // 箭头
    mjGEOM_ARROW1,                  // 无楔形物的箭头
    mjGEOM_ARROW2,                  // 双向箭头
    mjGEOM_LINE,                    // 线段
    mjGEOM_LINEBOX,                 // 线框立方体
    mjGEOM_FLEX,                    // 柔性体
    mjGEOM_SKIN,                    // 皮肤
    mjGEOM_LABEL,                   // 文本标签
    mjGEOM_TRIANGLE,                // 三角形

    mjGEOM_NONE = 1001      // 缺失的几何类型
} mjtGeom;


typedef enum mjtCamLight_ {       // 相机和光源的跟踪模式
    mjCAMLIGHT_FIXED = 0,        // 位置和旋转固定在物体上
    mjCAMLIGHT_TRACK,               // 位置跟踪物体，旋转固定在全局坐标系
    mjCAMLIGHT_TRACKCOM,            // 位置跟踪子树的质心，旋转固定在物体上
    mjCAMLIGHT_TARGETBODY,          // 位置固定，旋转跟踪目标物体
    mjCAMLIGHT_TARGETBODYCOM        // 位置固定，旋转跟踪目标子树的质心
} mjtCamLight;


typedef enum mjtTexture_ {        // 纹理类型
    mjTEXTURE_2D = 0,        // 2D纹理，适用于平面和高度场
    mjTEXTURE_CUBE,                 // 立方体贴图，适用于其他几何类型
    mjTEXTURE_SKYBOX                // 用作天空盒的立方体贴图
} mjtTexture;


typedef enum mjtTextureRole_ {    // 纹理在渲染中的角色
    mjTEXROLE_USER = 0,        // 未指定
    mjTEXROLE_RGB,                  // 基础颜色（反照率）
    mjTEXROLE_OCCLUSION,            // 环境光遮蔽
    mjTEXROLE_ROUGHNESS,            // 粗糙度
    mjTEXROLE_METALLIC,             // 金属度
    mjTEXROLE_NORMAL,               // 法线（凹凸贴图）
    mjTEXROLE_OPACITY,              // 透明度
    mjTEXROLE_EMISSIVE,             // 自发光
    mjTEXROLE_RGBA,                 // 基础颜色和透明度
    mjTEXROLE_ORM,                  // 环境遮蔽、粗糙度、金属度
    mjNTEXROLE
} mjtTextureRole;


typedef enum mjtIntegrator_ {     // 积分器模式
    mjINT_EULER = 0,        // 半隐式欧拉法
    mjINT_RK4,                      // 四阶龙格-库塔法
    mjINT_IMPLICIT,                 // 速度隐式积分
    mjINT_IMPLICITFAST              // 速度隐式积分，无RNE导数
} mjtIntegrator;


typedef enum mjtCone_ {           // 摩擦锥类型
    mjCONE_PYRAMIDAL = 0,       // 金字塔形
    mjCONE_ELLIPTIC                 // 椭圆形
} mjtCone;


typedef enum mjtJacobian_ {       // 约束雅可比矩阵类型
    mjJAC_DENSE = 0,       // 密集矩阵
    mjJAC_SPARSE,                   // 稀疏矩阵
    mjJAC_AUTO                      // 自动选择（nv<60时用密集，否则稀疏）
} mjtJacobian;


typedef enum mjtSolver_ {         // 约束求解算法
    mjSOL_PGS = 0,       // 投影高斯-赛德尔（对偶）
    mjSOL_CG,                       // 共轭梯度（原始）
    mjSOL_NEWTON                    // 牛顿法（原始）
} mjtSolver;


typedef enum mjtEq_ {             // 等式约束类型
    mjEQ_CONNECT = 0,        // 在点处连接两个物体（球关节）
    mjEQ_WELD,                      // 固定两个物体的相对位置和方向
    mjEQ_JOINT,                     // 用三次曲线耦合两个标量关节的值
    mjEQ_TENDON,                    // 用三次曲线耦合两个肌腱的长度
    mjEQ_FLEX,                      // 固定柔性体的所有边长度
    mjEQ_DISTANCE                   // 不支持，使用会报错
} mjtEq;


typedef enum mjtWrap_ {           // 肌腱包裹对象类型
    mjWRAP_NONE = 0,        // 空对象
    mjWRAP_JOINT,                   // 恒定力矩臂
    mjWRAP_PULLEY,                  // 用于分割肌腱的滑轮
    mjWRAP_SITE,                    // 通过站点
    mjWRAP_SPHERE,                  // 绕球体包裹
    mjWRAP_CYLINDER                 // 绕无限圆柱体包裹
} mjtWrap;


typedef enum mjtTrn_ {            // 执行器传动类型
    mjTRN_JOINT = 0,        // 作用于关节的力
    mjTRN_JOINTINPARENT,            // 在父坐标系中作用于关节的力
    mjTRN_SLIDERCRANK,              // 通过滑块曲柄机构传递力
    mjTRN_TENDON,                   // 作用于肌腱的力
    mjTRN_SITE,                     // 作用于站点的力
    mjTRN_BODY,                     // 作用于物体几何体的粘附力

    mjTRN_UNDEFINED = 1000      // 未定义的传动类型
} mjtTrn;


typedef enum mjtDyn_ {            // 执行器动态类型
    mjDYN_NONE = 0,        // 无内部动态；ctrl直接指定力
    mjDYN_INTEGRATOR,               // 积分器：da/dt = u
    mjDYN_FILTER,                   // 线性滤波器：da/dt = (u-a) / tau
    mjDYN_FILTEREXACT,              // 精确积分的线性滤波器：da/dt = (u-a) / tau
    mjDYN_MUSCLE,                   // 分段线性滤波器，两个时间常数
    mjDYN_USER                      // 用户定义的动态类型
} mjtDyn;


typedef enum mjtGain_ {           // 执行器增益类型
    mjGAIN_FIXED = 0,        // 固定增益
    mjGAIN_AFFINE,                  // 常数 + kp*长度 + kv*速度
    mjGAIN_MUSCLE,                  // 由mju_muscleGain()计算的肌肉FLV曲线
    mjGAIN_USER                     // 用户定义的增益类型
} mjtGain;


typedef enum mjtBias_ {           // 执行器偏置类型
    mjBIAS_NONE = 0,        // 无偏置
    mjBIAS_AFFINE,                  // 常数 + kp*长度 + kv*速度
    mjBIAS_MUSCLE,                  // 由mju_muscleBias()计算的肌肉被动力
    mjBIAS_USER                     // 用户定义的偏置类型
} mjtBias;


typedef enum mjtObj_ {            // MuJoCo对象类型
    mjOBJ_UNKNOWN = 0,        // 未知对象类型
    mjOBJ_BODY,                     // 物体
    mjOBJ_XBODY,                    // 物体，用于访问常规坐标系而非惯性坐标系
    mjOBJ_JOINT,                    // 关节
    mjOBJ_DOF,                      // 自由度
    mjOBJ_GEOM,                     // 几何体
    mjOBJ_SITE,                     // 站点
    mjOBJ_CAMERA,                   // 相机
    mjOBJ_LIGHT,                    // 光源
    mjOBJ_FLEX,                     // 柔性体
    mjOBJ_MESH,                     // 网格
    mjOBJ_SKIN,                     // 皮肤
    mjOBJ_HFIELD,                   // 高度场
    mjOBJ_TEXTURE,                  // 纹理
    mjOBJ_MATERIAL,                 // 渲染材质
    mjOBJ_PAIR,                     // 包含的几何体对
    mjOBJ_EXCLUDE,                  // 排除的物体对
    mjOBJ_EQUALITY,                 // 等式约束
    mjOBJ_TENDON,                   // 肌腱
    mjOBJ_ACTUATOR,                 // 执行器
    mjOBJ_SENSOR,                   // 传感器
    mjOBJ_NUMERIC,                  // 数值型自定义字段
    mjOBJ_TEXT,                     // 文本型自定义字段
    mjOBJ_TUPLE,                    // 元组型自定义字段
    mjOBJ_KEY,                      // 关键帧
    mjOBJ_PLUGIN,                   // 插件实例

    mjNOBJECT,                      // 对象类型的数量

    // 元元素，不出现在mjModel中
    mjOBJ_FRAME = 100       // 坐标系
} mjtObj;


typedef enum mjtConstraint_ {     // 约束类型
    mjCNSTR_EQUALITY = 0,        // 等式约束
    mjCNSTR_FRICTION_DOF,           // 自由度摩擦
    mjCNSTR_FRICTION_TENDON,        // 肌腱摩擦
    mjCNSTR_LIMIT_JOINT,            // 关节极限
    mjCNSTR_LIMIT_TENDON,           // 肌腱极限
    mjCNSTR_CONTACT_FRICTIONLESS,   // 无摩擦接触
    mjCNSTR_CONTACT_PYRAMIDAL,      // 金字塔摩擦锥的接触
    mjCNSTR_CONTACT_ELLIPTIC        // 椭圆摩擦锥的接触
} mjtConstraint;


typedef enum mjtConstraintState_ {  // 约束状态
    mjCNSTRSTATE_SATISFIED = 0,       // 约束满足，零成本（极限、接触）
    mjCNSTRSTATE_QUADRATIC,           // 二次成本（等式、摩擦、极限、接触）
    mjCNSTRSTATE_LINEARNEG,           // 线性成本，负侧（摩擦）
    mjCNSTRSTATE_LINEARPOS,           // 线性成本，正侧（摩擦）
    mjCNSTRSTATE_CONE                 // 到锥体的平方距离成本（椭圆接触）
} mjtConstraintState;


typedef enum mjtSensor_ {         // 传感器类型
    // 常见机器人传感器，附加到站点
    mjSENS_TOUCH = 0,        // 传感器区域接触法向力的标量和
    mjSENS_ACCELEROMETER,           // 局部坐标系中的3D线性加速度
    mjSENS_VELOCIMETER,             // 局部坐标系中的3D线速度
    mjSENS_GYRO,                    // 局部坐标系中的3D角速度
    mjSENS_FORCE,                   // 站点物体与其父物体之间的3D力
    mjSENS_TORQUE,                  // 站点物体与其父物体之间的3D扭矩
    mjSENS_MAGNETOMETER,            // 3D磁力计
    mjSENS_RANGEFINDER,             // 沿z轴到最近几何体或站点的标量距离
    mjSENS_CAMPROJECTION,           // 站点在相机图像中的像素坐标

    // 与标量关节、肌腱、执行器相关的传感器
    mjSENS_JOINTPOS,                // 标量关节位置（仅铰链和滑动）
    mjSENS_JOINTVEL,                // 标量关节速度（仅铰链和滑动）
    mjSENS_TENDONPOS,               // 标量肌腱位置
    mjSENS_TENDONVEL,               // 标量肌腱速度
    mjSENS_ACTUATORPOS,             // 标量执行器位置
    mjSENS_ACTUATORVEL,             // 标量执行器速度
    mjSENS_ACTUATORFRC,             // 标量执行器力
    mjSENS_JOINTACTFRC,             // 在关节处测量的执行器力

    // 球关节相关传感器
    mjSENS_BALLQUAT,                // 球关节四元数（4D）
    mjSENS_BALLANGVEL,              // 球关节角速度（3D）

    // 关节和肌腱极限传感器，约束空间
    mjSENS_JOINTLIMITPOS,           // 关节极限距离余量
    mjSENS_JOINTLIMITVEL,           // 关节极限速度
    mjSENS_JOINTLIMITFRC,           // 关节极限力
    mjSENS_TENDONLIMITPOS,          // 肌腱极限距离余量
    mjSENS_TENDONLIMITVEL,          // 肌腱极限速度
    mjSENS_TENDONLIMITFRC,          // 肌腱极限力

    // 附加到具有空间帧的对象的传感器：物体、几何体、站点、相机
    mjSENS_FRAMEPOS,                // 3D位置
    mjSENS_FRAMEQUAT,               // 4D单位四元数方向
    mjSENS_FRAMEXAXIS,              // 物体坐标系的x轴单位向量
    mjSENS_FRAMEYAXIS,              // 物体坐标系的y轴单位向量
    mjSENS_FRAMEZAXIS,              // 物体坐标系的z轴单位向量
    mjSENS_FRAMELINVEL,             // 3D线速度
    mjSENS_FRAMEANGVEL,             // 3D角速度
    mjSENS_FRAMELINACC,             // 3D线加速度
    mjSENS_FRAMEANGACC,             // 3D角加速度

    // 与运动学子树相关的传感器；附加到物体（子树根）
    mjSENS_SUBTREECOM,              // 子树的质心（3D）
    mjSENS_SUBTREELINVEL,           // 子树的线速度（3D）
    mjSENS_SUBTREEANGMOM,           // 子树的角动量（3D）

    // 几何距离传感器；附加到几何体或物体
    mjSENS_GEOMDIST,                // 两个几何体之间的符号距离
    mjSENS_GEOMNORMAL,              // 两个几何体之间的法线方向
    mjSENS_GEOMFROMTO,              // 两个几何体之间的线段

    // 全局传感器
    mjSENS_E_POTENTIAL,             // 势能
    mjSENS_E_KINETIC,               // 动能
    mjSENS_CLOCK,                   // 仿真时间

    // 插件控制的传感器
    mjSENS_PLUGIN,                  // 插件控制

    // 用户定义传感器
    mjSENS_USER                     // 由mjcb_sensor回调提供数据
} mjtSensor;


typedef enum mjtStage_ {          // 计算阶段
    mjSTAGE_NONE = 0,        // 无计算
    mjSTAGE_POS,                    // 位置相关计算
    mjSTAGE_VEL,                    // 速度相关计算
    mjSTAGE_ACC                     // 加速度/力相关计算
} mjtStage;


typedef enum mjtDataType_ {       // 传感器数据类型
    mjDATATYPE_REAL = 0,        // 实数值，无约束
    mjDATATYPE_POSITIVE,            // 正值；0或负值表示不活动
    mjDATATYPE_AXIS,                // 3D单位向量
    mjDATATYPE_QUATERNION           // 单位四元数
} mjtDataType;


typedef enum mjtSameFrame_ {      // 物体与其子物体的坐标系对齐方式
    mjSAMEFRAME_NONE = 0,        // 无对齐
    mjSAMEFRAME_BODY,               // 与物体坐标系对齐
    mjSAMEFRAME_INERTIA,            // 与惯性坐标系对齐
    mjSAMEFRAME_BODYROT,            // 方向与物体坐标系对齐
    mjSAMEFRAME_INERTIAROT          // 方向与惯性坐标系对齐
} mjtSameFrame;


typedef enum mjtLRMode_ {         // 执行器长度范围计算模式
    mjLRMODE_NONE = 0,            // 不处理任何执行器
    mjLRMODE_MUSCLE,                // 处理肌肉执行器
    mjLRMODE_MUSCLEUSER,            // 处理肌肉和用户执行器
    mjLRMODE_ALL                    // 处理所有执行器
} mjtLRMode;


typedef enum mjtFlexSelf_ {       // 柔性体自碰撞模式
    mjFLEXSELF_NONE = 0,          // 无自碰撞
    mjFLEXSELF_NARROW,              // 跳过中阶段，直接进入窄阶段
    mjFLEXSELF_BVH,                 // 中阶段使用BVH（如果启用中阶段）
    mjFLEXSELF_SAP,                 // 中阶段使用SAP
    mjFLEXSELF_AUTO                 // 自动选择BVH或SAP
} mjtFlexSelf;


//---------------------------------- mjLROpt -------------------------------------------------------

struct mjLROpt_ {                 // mj_setLengthRange()的选项
    // 标志
    int mode;                       // 处理的执行器类型（mjtLRMode）
    int useexisting;                // 如果可用则使用现有长度范围
    int uselimit;                   // 如果可用则使用关节和肌腱极限

    // 算法参数
    mjtNum accel;                   // 用于计算力的目标加速度
    mjtNum maxforce;                // 最大力；0：无限制
    mjtNum timeconst;               // 速度减少的时间常数；最小0.01
    mjtNum timestep;                // 仿真时间步长；0：使用mjOption.timestep
    mjtNum inttotal;                // 总仿真时间间隔
    mjtNum interval;                // 评估时间间隔（结束时）
    mjtNum tolrange;                // 收敛容差（相对于范围）
};
typedef struct mjLROpt_ mjLROpt;


//---------------------------------- mjVFS ---------------------------------------------------------

struct mjVFS_ {                               // 用于从内存加载的虚拟文件系统
    void* impl_;                                // 指向VFS内存的内部指针
};
typedef struct mjVFS_ mjVFS;

//---------------------------------- mjOption ------------------------------------------------------

struct mjOption_ {                // 物理选项
    // 时间参数
    mjtNum timestep;                // 仿真时间步长
    mjtNum apirate;                 // 远程API的更新频率（Hz）

    // 求解器参数
    mjtNum impratio;                // 摩擦与法向接触阻抗的比率
    mjtNum tolerance;               // 主求解器容差
    mjtNum ls_tolerance;            // CG/牛顿线搜索容差
    mjtNum noslip_tolerance;        // 无滑动求解器容差
    mjtNum ccd_tolerance;           // 凸碰撞求解器容差

    // 物理常量
    mjtNum gravity[3];              // 重力加速度
    mjtNum wind[3];                 // 风力（用于升力、阻力和粘性）
    mjtNum magnetic[3];             // 全局磁通量
    mjtNum density;                 // 介质密度
    mjtNum viscosity;               // 介质粘性

    // 覆盖接触求解器参数（如果启用）
    mjtNum o_margin;                // 边距
    mjtNum o_solref[mjNREF];        // 求解器参考参数
    mjtNum o_solimp[mjNIMP];        // 求解器阻抗参数
    mjtNum o_friction[5];           // 摩擦参数

    // 离散设置
    int integrator;                 // 积分模式（mjtIntegrator）
    int cone;                       // 摩擦锥类型（mjtCone）
    int jacobian;                   // 雅可比矩阵类型（mjtJacobian）
    int solver;                     // 求解器算法（mjtSolver）
    int iterations;                 // 主求解器的最大迭代次数
    int ls_iterations;              // CG/牛顿线搜索的最大迭代次数
    int noslip_iterations;          // 无滑动求解器的最大迭代次数
    int ccd_iterations;             // 凸碰撞求解器的最大迭代次数
    int disableflags;               // 禁用标准功能的位标志
    int enableflags;                // 启用可选功能的位标志
    int disableactuator;            // 按组ID禁用执行器的位标志

    // SDF碰撞设置
    int sdf_initpoints;             // 梯度下降的起始点数
    int sdf_iterations;             // 梯度下降的最大迭代次数
};
typedef struct mjOption_ mjOption;


//---------------------------------- mjVisual ------------------------------------------------------

struct mjVisual_ {                // 可视化选项
    struct {                        // 全局参数
        int orthographic;             // 自由相机是否正交投影（0：否，1：是）
        float fovy;                   // 自由相机的垂直视场角（正交投影时为长度，否则为角度）
        float ipd;                    // 自由相机的瞳孔间距
        float azimuth;                // 自由相机的初始方位角（度）
        float elevation;              // 自由相机的初始仰角（度）
        float linewidth;              // 线框和射线渲染的线宽
        float glow;                   // 选中物体的发光系数
        float realtime;               // 初始实时因子（1：实时）
        int   offwidth;               // 离屏缓冲区的宽度
        int   offheight;              // 离屏缓冲区的高度
        int   ellipsoidinertia;       // 惯性可视化几何体（0：立方体，1：椭球体）
        int   bvactive;               // 可视化活动包围体（0：否，1：是）
    } global;

    struct {                        // 渲染质量
        int   shadowsize;             // 阴影贴图纹理的大小
        int   offsamples;             // 离屏渲染的多重采样数
        int   numslices;              // 内置几何体绘制的切片数
        int   numstacks;              // 内置几何体绘制的堆叠数
        int   numquads;               // 立方体渲染的四边形数
    } quality;

    struct {                        // 头灯
        float ambient[3];             // 环境光RGB（alpha=1）
        float diffuse[3];             // 漫反射RGB（alpha=1）
        float specular[3];            // 镜面反射RGB（alpha=1）
        int   active;                 // 头灯是否激活
    } headlight;

    struct {                        // 映射
        float stiffness;              // 鼠标扰动的刚度（空间到力）
        float stiffnessrot;           // 鼠标扰动的旋转刚度（空间到扭矩）
        float force;                  // 力单位到空间单位的转换系数
        float torque;                 // 扭矩单位到空间单位的转换系数
        float alpha;                  // 启用透明度时几何体alpha的缩放
        float fogstart;               // OpenGL雾效起始距离（mjModel.stat.extent的倍数）
        float fogend;                 // OpenGL雾效结束距离（mjModel.stat.extent的倍数）
        float znear;                  // 近裁剪平面（mjModel.stat.extent的倍数）
        float zfar;                   // 远裁剪平面（mjModel.stat.extent的倍数）
        float haze;                   // 雾效比率
        float shadowclip;             // 定向光源的阴影裁剪距离（mjModel.stat.extent的倍数）
        float shadowscale;            // 聚光灯的阴影缩放（light.cutoff的倍数）
        float actuatortendon;         // 肌腱宽度的缩放
    } map;

    struct {                        // 装饰元素的缩放比例（相对于平均物体尺寸）
        float forcewidth;             // 力箭头的宽度
        float contactwidth;           // 接触宽度
        float contactheight;          // 接触高度
        float connect;                // 自动连接胶囊体的宽度
        float com;                    // 质心半径
        float camera;                 // 相机物体
        float light;                  // 光源物体
        float selectpoint;            // 选择点
        float jointlength;            // 关节长度
        float jointwidth;             // 关节宽度
        float actuatorlength;         // 执行器长度
        float actuatorwidth;          // 执行器宽度
        float framelength;            // 物体坐标系轴的长度
        float framewidth;             // 物体坐标系轴的宽度
        float constraint;             // 约束宽度
        float slidercrank;            // 滑块曲柄宽度
        float frustum;                // 视锥体的远平面
    } scale;

    struct {                        // 装饰元素的颜色
        float fog[4];                 // 雾效颜色
        float haze[4];                // 雾霾颜色
        float force[4];             // 外部力颜色
        float inertia[4];           // 惯性框颜色
        float joint[4];             // 关节颜色
        float actuator[4];          // 执行器中性颜色
        float actuatornegative[4];  // 执行器负极限颜色
        float actuatorpositive[4];  // 执行器正极限颜色
        float com[4];               // 质心颜色
        float camera[4];            // 相机物体颜色
        float light[4];             // 光源物体颜色
        float selectpoint[4];       // 选择点颜色
        float connect[4];           // 自动连接颜色
        float contactpoint[4];      // 接触点颜色
        float contactforce[4];      // 接触力颜色
        float contactfriction[4];   // 接触摩擦力颜色
        float contacttorque[4];     // 接触扭矩颜色
        float contactgap[4];        // 间隙中的接触点颜色
        float rangefinder[4];       // 测距仪射线颜色
        float constraint[4];        // 约束颜色
        float slidercrank[4];       // 滑块曲柄颜色
        float crankbroken[4];       // 曲柄需拉伸/断裂时的颜色
        float frustum[4];           // 相机视锥体颜色
        float bv[4];                // 包围体颜色
        float bvactive[4];          // 活动包围体颜色
    } rgba;
};
typedef struct mjVisual_ mjVisual;


//---------------------------------- mjStatistic ---------------------------------------------------

struct mjStatistic_ {             // 模型统计信息（基于qpos0）
    mjtNum meaninertia;             // 平均对角惯性
    mjtNum meanmass;                // 平均物体质量
    mjtNum meansize;                // 平均物体尺寸
    mjtNum extent;                  // 空间范围
    mjtNum center[3];               // 模型中心
};
typedef struct mjStatistic_ mjStatistic;


//---------------------------------- mjModel -------------------------------------------------------

struct mjModel_ {
    // ------------------------------- 尺寸信息

    // mjModel构建时需要的尺寸
    int nq;                         // 广义坐标数量 = qpos的维度
    int nv;                         // 自由度数量 = qvel的维度
    int nu;                         // 执行器/控制数量 = ctrl的维度
    int na;                         // 激活状态数量 = act的维度
    int nbody;                      // 物体数量
    int nbvh;                       // 所有物体的总包围体数量
    int nbvhstatic;                 // 静态包围体数量（aabb存储在mjModel中）
    int nbvhdynamic;                // 动态包围体数量（aabb存储在mjData中）
    int njnt;                       // 关节数量
    int ngeom;                      // 几何体数量
    int nsite;                      // 站点数量
    int ncam;                       // 相机数量
    int nlight;                     // 光源数量
    int nflex;                      // 柔性体数量
    int nflexnode;                  // 所有柔性体的自由度数量
    int nflexvert;                  // 所有柔性体的顶点数量
    int nflexedge;                  // 所有柔性体的边数量
    int nflexelem;                  // 所有柔性体的元素数量
    int nflexelemdata;              // 所有柔性体元素的顶点ID数量
    int nflexelemedge;              // 所有柔性体元素的边ID数量
    int nflexshelldata;             // 所有柔性体壳片段的顶点ID数量
    int nflexevpair;                // 所有柔性体的元素-顶点对数量
    int nflextexcoord;              // 带纹理坐标的顶点数量
    int nmesh;                      // 网格数量
    int nmeshvert;                  // 所有网格的顶点数量
    int nmeshnormal;                // 所有网格的法线数量
    int nmeshtexcoord;              // 所有网格的纹理坐标数量
    int nmeshface;                  // 所有网格的三角面数量
    int nmeshgraph;                 // 网格辅助数据中的整数数量
    int nmeshpoly;                  // 所有网格的多边形数量
    int nmeshpolyvert;              // 所有多边形的顶点数量
    int nmeshpolymap;               // 顶点映射中的多边形数量
    int nskin;                      // 皮肤数量
    int nskinvert;                  // 所有皮肤的顶点数量
    int nskintexvert;               // 所有皮肤中带纹理坐标的顶点数量
    int nskinface;                  // 所有皮肤的三角面数量
    int nskinbone;                  // 所有皮肤的骨骼数量
    int nskinbonevert;              // 所有皮肤骨骼的顶点数量
    int nhfield;                    // 高度场数量
    int nhfielddata;                // 所有高度场的数据点数
    int ntex;                       // 纹理数量
    int ntexdata;                   // 纹理RGB数据的字节数
    int nmat;                       // 材质数量
    int npair;                      // 预定义的几何体对数量
    int nexclude;                   // 排除的几何体对数量
    int neq;                        // 等式约束数量
    int ntendon;                    // 肌腱数量
    int nwrap;                      // 所有肌腱路径中的包裹对象数量
    int nsensor;                    // 传感器数量
    int nnumeric;                   // 数值型自定义字段数量
    int nnumericdata;               // 所有数值字段中的mjtNum数量
    int ntext;                      // 文本型自定义字段数量
    int ntextdata;                  // 所有文本字段中的mjtByte数量
    int ntuple;                     // 元组型自定义字段数量
    int ntupledata;                 // 所有元组字段中的对象数量
    int nkey;                       // 关键帧数量
    int nmocap;                     // 运动捕捉物体数量
    int nplugin;                    // 插件实例数量
    int npluginattr;                // 所有插件配置属性的字符数
    int nuser_body;                 // body_user中的mjtNum数量
    int nuser_jnt;                  // jnt_user中的mjtNum数量
    int nuser_geom;                 // geom_user中的mjtNum数量
    int nuser_site;                 // site_user中的mjtNum数量
    int nuser_cam;                  // cam_user中的mjtNum数量
    int nuser_tendon;               // tendon_user中的mjtNum数量
    int nuser_actuator;             // actuator_user中的mjtNum数量
    int nuser_sensor;               // sensor_user中的mjtNum数量
    int nnames;                     // 所有名称的字符数
    int npaths;                     // 所有路径的字符数

    // mjModel构建后设置的尺寸
    int nnames_map;                 // 名称哈希映射的槽数
    int nM;                         // 稀疏惯性矩阵的非零元素数量
    int nB;                         // 稀疏物体-自由度矩阵的非零元素数量
    int nC;                         // 约简自由度-自由度矩阵的非零元素数量
    int nD;                         // 自由度-自由度矩阵的非零元素数量
    int nJmom;                      // 执行器力矩矩阵的非零元素数量
    int ntree;                      // 世界物体下的运动学树数量
    int ngravcomp;                  // 具有非零重力补偿的物体数量
    int nemax;                      // 潜在等式约束行数
    int njmax;                      // 约束雅可比矩阵的可用行数（旧版）
    int nconmax;                    // 接触列表中的潜在接触数（旧版）
    int nuserdata;                  // 为用户保留的mjtNum数量
    int nsensordata;                // 传感器数据向量中的mjtNum数量
    int npluginstate;               // 插件状态向量中的mjtNum数量

    size_t narena;                  // mjData竞技场中的字节数（包含堆栈）
    size_t nbuffer;                 // 缓冲区中的字节数

    // ------------------------------- 选项和统计信息

    mjOption opt;                   // 物理选项
    mjVisual vis;                   // 可视化选项
    mjStatistic stat;               // 模型统计信息

    // ------------------------------- 缓冲区

      // 主缓冲区
    void* buffer;               // 主缓冲区；所有指针都指向该缓冲区内 (nbuffer)

    // 默认广义坐标
    mjtNum* qpos0;                // 默认姿态下的广义坐标值 (nq x 1)
    mjtNum* qpos_spring;          // 弹簧的参考姿态 (nq x 1)

    // 体属性
    int* body_parentid;        // 父体的ID (nbody x 1)
    int* body_rootid;          // 体所在根节点的ID (nbody x 1)
    int* body_weldid;          // 当前体焊接到的目标体ID (nbody x 1)
    int* body_mocapid;         // 运动捕捉数据ID（-1表示无） (nbody x 1)
    int* body_jntnum;          // 该体关联的关节数量 (nbody x 1)
    int* body_jntadr;          // 关节数据起始地址（-1表示无关节） (nbody x 1)
    int* body_dofnum;          // 该体的自由度数量 (nbody x 1)
    int* body_dofadr;          // 自由度数据起始地址（-1表示无自由度） (nbody x 1)
    int* body_treeid;          // 体所属运动树的ID（-1表示静态） (nbody x 1)
    int* body_geomnum;         // 该体关联的几何体数量 (nbody x 1)
    int* body_geomadr;         // 几何体数据起始地址（-1表示无几何体） (nbody x 1)
    mjtByte* body_simple;          // 质量矩阵简化标志：1-对角矩阵，2-仅滑动自由度对角 (nbody x 1)
    mjtByte* body_sameframe;       // 是否与惯性系同坐标系（mjtSameframe枚举） (nbody x 1)
    mjtNum* body_pos;             // 相对于父体的位置偏移 (nbody x 3)
    mjtNum* body_quat;            // 相对于父体的四元数朝向偏移 (nbody x 4)
    mjtNum* body_ipos;            // 局部坐标系下的质心位置 (nbody x 3)
    mjtNum* body_iquat;           // 局部坐标系下的惯性椭球朝向 (nbody x 4)
    mjtNum* body_mass;            // 质量 (nbody x 1)
    mjtNum* body_subtreemass;     // 以该体为根的子树的累计质量 (nbody x 1)
    mjtNum* body_inertia;         // 惯性张量（在ipos/iquat坐标系下的对角线） (nbody x 3)
    mjtNum* body_invweight0;      // 初始姿态下的平均逆惯性（平移，旋转） (nbody x 2)
    mjtNum* body_gravcomp;        // 抗重力系数（以体重为单位） (nbody x 1)
    mjtNum* body_margin;          // 所有几何体边距的最大值 (nbody x 1)
    mjtNum* body_user;            // 用户自定义数据 (nbody x nuser_body)
    int* body_plugin;          // 插件实例ID（-1表示未使用） (nbody x 1)
    int* body_contype;         // 所有几何体接触类型的按位或 (nbody x 1)
    int* body_conaffinity;     // 所有几何体接触亲和性的按位或 (nbody x 1)
    int* body_bvhadr;          // BVH（包围体层次）根节点地址 (nbody x 1)
    int* body_bvhnum;          // 包围体数量 (nbody x 1)

    // 包围体层次结构（BVH）
    int* bvh_depth;            // 在BVH中的深度 (nbvh x 1)
    int* bvh_child;            // 左右子节点索引 (nbvh x 2)
    int* bvh_nodeid;           // 节点对应的几何体或元素ID（-1表示非叶节点） (nbvh x 1)
    mjtNum* bvh_aabb;             // 局部坐标下的轴对齐包围盒（中心，尺寸） (nbvhstatic x 6)

    // 关节属性
    int* jnt_type;             // 关节类型（mjtJoint枚举） (njnt x 1)
    int* jnt_qposadr;          // 在qpos中的起始地址 (njnt x 1)
    int* jnt_dofadr;           // 在qvel中的起始地址 (njnt x 1)
    int* jnt_bodyid;           // 关节所属体的ID (njnt x 1)
    int* jnt_group;            // 可见性分组 (njnt x 1)
    mjtByte* jnt_limited;          // 是否有关节限制 (njnt x 1)
    mjtByte* jnt_actfrclimited;    // 是否有执行器力限制 (njnt x 1)
    mjtByte* jnt_actgravcomp;      // 是否通过执行器施加抗重力 (njnt x 1)
    mjtNum* jnt_solref;           // 约束求解器参考参数（限制） (njnt x mjNREF)
    mjtNum* jnt_solimp;           // 约束求解器阻抗参数（限制） (njnt x mjNIMP)
    mjtNum* jnt_pos;              // 局部锚点位置 (njnt x 3)
    mjtNum* jnt_axis;             // 局部关节轴方向 (njnt x 3)
    mjtNum* jnt_stiffness;        // 刚度系数 (njnt x 1)
    mjtNum* jnt_range;            // 关节运动范围限制 (njnt x 2)
    mjtNum* jnt_actfrcrange;      // 执行器总力范围 (njnt x 2)
    mjtNum* jnt_margin;           // 限制检测的最小距离 (njnt x 1)
    mjtNum* jnt_user;             // 用户自定义数据 (njnt x nuser_jnt)

    // 自由度属性
    int* dof_bodyid;           // 自由度所属体的ID (nv x 1)
    int* dof_jntid;            // 自由度所属关节的ID (nv x 1)
    int* dof_parentid;         // 父自由度ID（-1表示无） (nv x 1)
    int* dof_treeid;           // 自由度所属运动树的ID (nv x 1)
    int* dof_Madr;             // 在M矩阵对角线中的地址 (nv x 1)
    int* dof_simplenum;        // 连续简单自由度的数量 (nv x 1)
    mjtNum* dof_solref;           // 约束求解器参考参数（摩擦损耗） (nv x mjNREF)
    mjtNum* dof_solimp;           // 约束求解器阻抗参数（摩擦损耗） (nv x mjNIMP)
    mjtNum* dof_frictionloss;     // 自由度摩擦损耗 (nv x 1)
    mjtNum* dof_armature;         // 自由度附加惯量/质量 (nv x 1)
    mjtNum* dof_damping;          // 阻尼系数 (nv x 1)
    mjtNum* dof_invweight0;       // 初始姿态下的逆惯量对角线 (nv x 1)
    mjtNum* dof_M0;               // 初始姿态下的惯量对角线 (nv x 1)

    // 几何体属性
    int* geom_type;            // 几何类型（mjtGeom枚举） (ngeom x 1)
    int* geom_contype;         // 接触类型 (ngeom x 1)
    int* geom_conaffinity;     // 接触亲和性 (ngeom x 1)
    int* geom_condim;          // 接触维度（1,3,4,6） (ngeom x 1)
    int* geom_bodyid;          // 几何体所属体的ID (ngeom x 1)
    int* geom_dataid;          // 网格/高度场数据ID（-1表示无） (ngeom x 1)
    int* geom_matid;           // 渲染材质ID（-1表示无） (ngeom x 1)
    int* geom_group;           // 可见性分组 (ngeom x 1)
    int* geom_priority;        // 接触优先级 (ngeom x 1)
    int* geom_plugin;          // 插件实例ID（-1表示未使用） (ngeom x 1)
    mjtByte* geom_sameframe;       // 是否与体同坐标系（mjtSameframe枚举） (ngeom x 1)
    mjtNum* geom_solmix;          // 接触对中solref/solimp的混合系数 (ngeom x 1)
    mjtNum* geom_solref;          // 约束求解器参考参数（接触） (ngeom x mjNREF)
    mjtNum* geom_solimp;          // 约束求解器阻抗参数（接触） (ngeom x mjNIMP)
    mjtNum* geom_size;            // 几何体尺寸参数 (ngeom x 3)
    mjtNum* geom_aabb;            // 轴对齐包围盒（中心，尺寸） (ngeom x 6)
    mjtNum* geom_rbound;          // 包围球半径 (ngeom x 1)
    mjtNum* geom_pos;             // 相对于体的位置偏移 (ngeom x 3)
    mjtNum* geom_quat;            // 相对于体的四元数朝向偏移 (ngeom x 4)
    mjtNum* geom_friction;        // 摩擦系数（滑动，自旋，滚动） (ngeom x 3)
    mjtNum* geom_margin;          // 接触检测边距 (ngeom x 1)
    mjtNum* geom_gap;             // 求解器包含距离（边距-间隙） (ngeom x 1)
    mjtNum* geom_fluid;           // 流体交互参数 (ngeom x mjNFLUID)
    mjtNum* geom_user;            // 用户自定义数据 (ngeom x nuser_geom)
    float* geom_rgba;            // 未指定材质时的RGBA颜色 (ngeom x 4)

    // 站点属性
    int* site_type;            // 渲染几何类型（mjtGeom枚举） (nsite x 1)
    int* site_bodyid;          // 站点所属体的ID (nsite x 1)
    int* site_matid;           // 渲染材质ID（-1表示无） (nsite x 1)
    int* site_group;           // 可见性分组 (nsite x 1)
    mjtByte* site_sameframe;       // 是否与体同坐标系（mjtSameframe枚举） (nsite x 1)
    mjtNum* site_size;            // 渲染尺寸 (nsite x 3)
    mjtNum* site_pos;             // 相对于体的位置偏移 (nsite x 3)
    mjtNum* site_quat;            // 相对于体的四元数朝向偏移 (nsite x 4)
    mjtNum* site_user;            // 用户自定义数据 (nsite x nuser_site)
    float* site_rgba;            // 未指定材质时的RGBA颜色 (nsite x 4)

    // 相机属性
    int* cam_mode;             // 跟踪模式（mjtCamLight枚举） (ncam x 1)
    int* cam_bodyid;           // 相机所属体的ID (ncam x 1)
    int* cam_targetbodyid;     // 目标体ID（-1表示无） (ncam x 1)
    mjtNum* cam_pos;              // 相对于体坐标系的位置 (ncam x 3)
    mjtNum* cam_quat;             // 相对于体坐标系的朝向 (ncam x 4)
    mjtNum* cam_poscom0;          // 初始姿态下相对于子质心的全局位置 (ncam x 3)
    mjtNum* cam_pos0;             // 初始姿态下相对于体的全局位置 (ncam x 3)
    mjtNum* cam_mat0;             // 初始姿态下的全局朝向矩阵 (ncam x 9)
    int* cam_orthographic;     // 是否正交投影（0-透视，1-正交） (ncam x 1)
    mjtNum* cam_fovy;             // 垂直视场角（正交投影时为长度） (ncam x 1)
    mjtNum* cam_ipd;              // 瞳孔间距 (ncam x 1)
    int* cam_resolution;       // 分辨率[宽, 高] (ncam x 2)
    float* cam_sensorsize;       // 传感器尺寸[宽, 高] (ncam x 2)
    float* cam_intrinsic;        // 内参矩阵[焦距，主点坐标] (ncam x 4)
    mjtNum* cam_user;             // 用户自定义数据 (ncam x nuser_cam)

    // 光源属性
    int* light_mode;           // 光源跟踪模式（mjtCamLight枚举） (nlight x 1)
    int* light_bodyid;         // 光源所属体的ID (nlight x 1)
    int* light_targetbodyid;   // 目标体ID（-1表示无） (nlight x 1)
    mjtByte* light_directional;    // 是否为定向光源 (nlight x 1)
    mjtByte* light_castshadow;     // 是否投射阴影 (nlight x 1)
    float* light_bulbradius;     // 软阴影光源半径 (nlight x 1)
    mjtByte* light_active;         // 光源是否启用 (nlight x 1)
    mjtNum* light_pos;            // 相对于体坐标系的位置 (nlight x 3)
    mjtNum* light_dir;            // 相对于体坐标系的方向 (nlight x 3)
    mjtNum* light_poscom0;        // 初始姿态下相对于子质心的全局位置 (nlight x 3)
    mjtNum* light_pos0;           // 初始姿态下相对于体的全局位置 (nlight x 3)
    mjtNum* light_dir0;           // 初始姿态下的全局方向 (nlight x 3)
    float* light_attenuation;    // OpenGL衰减系数（二次模型） (nlight x 3)
    float* light_cutoff;         // OpenGL截止角度 (nlight x 1)
    float* light_exponent;       // OpenGL聚光指数 (nlight x 1)
    float* light_ambient;        // 环境光颜色（RGB，Alpha=1） (nlight x 3)
    float* light_diffuse;        // 漫反射颜色（RGB，Alpha=1） (nlight x 3)
    float* light_specular;       // 镜面反射颜色（RGB，Alpha=1） (nlight x 3)

    // 柔性体接触属性
    int* flex_contype;         // 接触类型 (nflex x 1)
    int* flex_conaffinity;     // 接触亲和性 (nflex x 1)
    int* flex_condim;          // 接触维度（1,3,4,6） (nflex x 1)
    int* flex_priority;        // 接触优先级 (nflex x 1)
    mjtNum* flex_solmix;          // 接触对中solref/solimp的混合系数 (nflex x 1)
    mjtNum* flex_solref;          // 约束求解器参考参数（接触） (nflex x mjNREF)
    mjtNum* flex_solimp;          // 约束求解器阻抗参数（接触） (nflex x mjNIMP)
    mjtNum* flex_friction;        // 摩擦系数（滑动，自旋，滚动） (nflex x 3)
    mjtNum* flex_margin;          // 接触检测边距 (nflex x 1)
    mjtNum* flex_gap;             // 求解器包含距离（边距-间隙） (nflex x 1)
    mjtByte* flex_internal;        // 是否启用内部柔性体碰撞 (nflex x 1)
    int* flex_selfcollide;     // 自碰撞模式（mjtFlexSelf枚举） (nflex x 1)
    int* flex_activelayers;    // 活动元素层数（仅3D） (nflex x 1)

    // 柔性体其他属性
    int* flex_dim;             // 维度（1-线，2-三角形，3-四面体） (nflex x 1)
    int* flex_matid;           // 渲染材质ID (nflex x 1)
    int* flex_group;           // 可见性分组 (nflex x 1)
    int* flex_interp;          // 插值方式（0-顶点，1-节点） (nflex x 1)
    int* flex_nodeadr;         // 节点数据起始地址 (nflex x 1)
    int* flex_nodenum;         // 节点数量 (nflex x 1)
    int* flex_vertadr;         // 顶点数据起始地址 (nflex x 1)
    int* flex_vertnum;         // 顶点数量 (nflex x 1)
    int* flex_edgeadr;         // 边数据起始地址 (nflex x 1)
    int* flex_edgenum;         // 边数量 (nflex x 1)
    int* flex_elemadr;         // 元素数据起始地址 (nflex x 1)
    int* flex_elemnum;         // 元素数量 (nflex x 1)
    int* flex_elemdataadr;     // 元素顶点ID起始地址 (nflex x 1)
    int* flex_elemedgeadr;     // 元素边ID起始地址 (nflex x 1)
    int* flex_shellnum;        // 壳层数量 (nflex x 1)
    int* flex_shelldataadr;    // 壳层数据起始地址 (nflex x 1)
    int* flex_evpairadr;       // 元素-顶点对起始地址 (nflex x 1)
    int* flex_evpairnum;       // 元素-顶点对数量 (nflex x 1)
    int* flex_texcoordadr;     // 纹理坐标数据地址（-1表示无） (nflex x 1)
    int* flex_nodebodyid;      // 节点所属体ID (nflexnode x 1)
    int* flex_vertbodyid;      // 顶点所属体ID (nflexvert x 1)
    int* flex_edge;            // 边的顶点ID（每边2个） (nflexedge x 2)
    int* flex_elem;            // 元素顶点ID（每元素dim+1个） (nflexelemdata x 1)
    int* flex_elemtexcoord;    // 元素纹理坐标（每元素dim+1个） (nflexelemdata x 1)
    int* flex_elemedge;        // 元素边ID (nflexelemedge x 1)
    int* flex_elemlayer;       // 元素到表面的距离（仅3D） (nflexelem x 1)
    int* flex_shell;           // 壳层顶点ID（每片段dim个） (nflexshelldata x 1)
    int* flex_evpair;          // （元素，顶点）碰撞对 (nflexevpair x 2)
    mjtNum* flex_vert;            // 局部坐标系下的顶点位置 (nflexvert x 3)
    mjtNum* flex_vert0;           // 初始姿态下归一化的顶点位置 (nflexvert x 3)
    mjtNum* flex_node;            // 局部坐标系下的节点位置 (nflexnode x 3)
    mjtNum* flex_node0;           // 初始姿态下的笛卡尔节点位置 (nflexnode x 3)
    mjtNum* flexedge_length0;     // 初始姿态下的边长度 (nflexedge x 1)
    mjtNum* flexedge_invweight0;  // 初始姿态下的边逆权重 (nflexedge x 1)
    mjtNum* flex_radius;          // 基本元素周围的半径 (nflex x 1)
    mjtNum* flex_stiffness;       // 有限元刚度矩阵 (nflexelem x 21)
    mjtNum* flex_damping;         // 瑞利阻尼系数 (nflex x 1)
    mjtNum* flex_edgestiffness;   // 边刚度 (nflex x 1)
    mjtNum* flex_edgedamping;     // 边阻尼 (nflex x 1)
    mjtByte* flex_edgeequality;    // 是否定义边等式约束 (nflex x 1)
    mjtByte* flex_rigid;           // 是否所有顶点在同一体上 (nflex x 1)
    mjtByte* flexedge_rigid;       // 是否边的两端顶点在同一体上 (nflexedge x 1)
    mjtByte* flex_centered;        // 是否所有顶点坐标为(0,0,0) (nflex x 1)
    mjtByte* flex_flatskin;        // 使用平面着色渲染柔性体表面 (nflex x 1)
    int* flex_bvhadr;          // BVH根节点地址（-1表示无） (nflex x 1)
    int* flex_bvhnum;          // 包围体数量 (nflex x 1)
    float* flex_rgba;            // 未指定材质时的RGBA颜色 (nflex x 4)
    float* flex_texcoord;        // 顶点纹理坐标 (nflextexcoord x 2)

    // 网格属性
    int* mesh_vertadr;         // 顶点数据起始地址 (nmesh x 1)
    int* mesh_vertnum;         // 顶点数量 (nmesh x 1)
    int* mesh_faceadr;         // 面数据起始地址 (nmesh x 1)
    int* mesh_facenum;         // 面数量 (nmesh x 1)
    int* mesh_bvhadr;          // BVH根节点地址 (nmesh x 1)
    int* mesh_bvhnum;          // BVH节点数量 (nmesh x 1)
    int* mesh_normaladr;       // 法线数据起始地址 (nmesh x 1)
    int* mesh_normalnum;       // 法线数量 (nmesh x 1)
    int* mesh_texcoordadr;     // 纹理坐标数据地址（-1表示无） (nmesh x 1)
    int* mesh_texcoordnum;     // 纹理坐标数量 (nmesh x 1)
    int* mesh_graphadr;        // 图数据地址（-1表示无） (nmesh x 1)
    float* mesh_vert;            // 所有网格的顶点坐标 (nmeshvert x 3)
    float* mesh_normal;          // 所有网格的法线 (nmeshnormal x 3)
    float* mesh_texcoord;        // 所有网格的纹理坐标 (nmeshtexcoord x 2)
    int* mesh_face;            // 面顶点索引 (nmeshface x 3)
    int* mesh_facenormal;      // 面法线索引 (nmeshface x 3)
    int* mesh_facetexcoord;    // 面纹理坐标索引 (nmeshface x 3)
    int* mesh_graph;           // 凸图数据 (nmeshgraph x 1)
    mjtNum* mesh_scale;           // 对资源顶点的缩放 (nmesh x 3)
    mjtNum* mesh_pos;             // 对资源顶点的平移 (nmesh x 3)
    mjtNum* mesh_quat;            // 对资源顶点的旋转 (nmesh x 4)
    int* mesh_pathadr;         // 网格资源路径地址（-1表示无） (nmesh x 1)
    int* mesh_polynum;         // 多边形数量 (nmesh x 1)
    int* mesh_polyadr;         // 多边形数据起始地址 (nmesh x 1)
    mjtNum* mesh_polynormal;      // 所有多边形法线 (nmeshpoly x 3)
    int* mesh_polyvertadr;     // 多边形顶点起始地址 (nmeshpoly x 1)
    int* mesh_polyvertnum;     // 多边形顶点数量 (nmeshpoly x 1)
    int* mesh_polyvert;        // 所有多边形顶点索引 (nmeshpolyvert x 1)
    int* mesh_polymapadr;      // 顶点对应的多边形起始地址 (nmeshvert x 1)
    int* mesh_polymapnum;      // 顶点对应的多边形数量 (nmeshvert x 1)
    int* mesh_polymap;         // 顶点到多边形的映射 (nmeshpolymap x 1)

    // 皮肤属性
    int* skin_matid;           // 皮肤材质ID（-1表示无） (nskin x 1)
    int* skin_group;           // 可见性分组 (nskin x 1)
    float* skin_rgba;            // 皮肤颜色 (nskin x 4)
    float* skin_inflate;         // 沿法线方向的膨胀量 (nskin x 1)
    int* skin_vertadr;         // 顶点数据起始地址 (nskin x 1)
    int* skin_vertnum;         // 顶点数量 (nskin x 1)
    int* skin_texcoordadr;     // 纹理坐标数据地址（-1表示无） (nskin x 1)
    int* skin_faceadr;         // 面数据起始地址 (nskin x 1)
    int* skin_facenum;         // 面数量 (nskin x 1)
    int* skin_boneadr;         // 骨骼数据起始地址 (nskin x 1)
    int* skin_bonenum;         // 骨骼数量 (nskin x 1)
    float* skin_vert;            // 所有皮肤的顶点坐标 (nskinvert x 3)
    float* skin_texcoord;        // 所有皮肤的纹理坐标 (nskintexvert x 2)
    int* skin_face;            // 所有皮肤的面索引 (nskinface x 3)
    int* skin_bonevertadr;     // 骨骼顶点起始地址 (nskinbone x 1)
    int* skin_bonevertnum;     // 骨骼顶点数量 (nskinbone x 1)
    float* skin_bonebindpos;     // 骨骼绑定位置 (nskinbone x 3)
    float* skin_bonebindquat;    // 骨骼绑定四元数 (nskinbone x 4)
    int* skin_bonebodyid;      // 骨骼所属体ID (nskinbone x 1)
    int* skin_bonevertid;      // 骨骼顶点ID (nskinbonevert x 1)
    float* skin_bonevertweight;  // 骨骼顶点权重 (nskinbonevert x 1)
    int* skin_pathadr;         // 皮肤资源路径地址（-1表示无） (nskin x 1)

    // 高度场属性
    mjtNum* hfield_size;          // 尺寸（x, y, 顶部z, 底部z） (nhfield x 4)
    int* hfield_nrow;          // 网格行数 (nhfield x 1)
    int* hfield_ncol;          // 网格列数 (nhfield x 1)
    int* hfield_adr;           // 高度场数据起始地址 (nhfield x 1)
    float* hfield_data;          // 高程数据 (nhfielddata x 1)
    int* hfield_pathadr;       // 高度场资源路径地址（-1表示无） (nhfield x 1)

    // 纹理属性
    int* tex_type;             // 纹理类型（mjtTexture枚举） (ntex x 1)
    int* tex_height;           // 纹理图像行数 (ntex x 1)
    int* tex_width;            // 纹理图像列数 (ntex x 1)
    int* tex_nchannel;         // 纹理通道数 (ntex x 1)
    int* tex_adr;              // 纹理数据起始地址 (ntex x 1)
    mjtByte* tex_data;             // 像素值 (ntexdata x 1)
    int* tex_pathadr;          // 纹理资源路径地址（-1表示无） (ntex x 1)

    // 材质属性
    int* mat_texid;            // 纹理ID数组（-1表示无） (nmat x mjNTEXROLE)
    mjtByte* mat_texuniform;       // 是否使纹理立方体均匀 (nmat x 1)
    float* mat_texrepeat;        // 二维纹理重复次数 (nmat x 2)
    float* mat_emission;         // 自发光强度（RGB缩放） (nmat x 1)
    float* mat_specular;         // 镜面反射强度（白色缩放） (nmat x 1)
    float* mat_shininess;        // 光泽度系数 (nmat x 1)
    float* mat_reflectance;      // 反射率（0表示禁用） (nmat x 1)
    float* mat_metallic;         // 金属度系数 (nmat x 1)
    float* mat_roughness;        // 粗糙度系数 (nmat x 1)
    float* mat_rgba;             // RGBA颜色 (nmat x 4)

    // 预定义几何体碰撞对
    int* pair_dim;             // 接触维度 (npair x 1)
    int* pair_geom1;           // 几何体1的ID (npair x 1)
    int* pair_geom2;           // 几何体2的ID (npair x 1)
    int* pair_signature;       // 体ID组合（body1 << 16 + body2） (npair x 1)
    mjtNum* pair_solref;          // 约束求解器参考参数（法向接触） (npair x mjNREF)
    mjtNum* pair_solreffriction;  // 约束求解器参考参数（摩擦接触） (npair x mjNREF)
    mjtNum* pair_solimp;          // 约束求解器阻抗参数（接触） (npair x mjNIMP)
    mjtNum* pair_margin;          // 接触检测边距 (npair x 1)
    mjtNum* pair_gap;             // 求解器包含距离（边距-间隙） (npair x 1)
    mjtNum* pair_friction;        // 摩擦系数（切向1, 切向2, 自旋, 滚动1, 滚动2） (npair x 5)

    // 排除碰撞的体对
    int* exclude_signature;    // 体ID组合（body1 << 16 + body2） (nexclude x 1)

    // 等式约束
    int* eq_type;              // 约束类型（mjtEq枚举） (neq x 1)
    int* eq_obj1id;            // 对象1的ID (neq x 1)
    int* eq_obj2id;            // 对象2的ID (neq x 1)
    int* eq_objtype;           // 对象类型（mjtObj枚举） (neq x 1)
    mjtByte* eq_active0;           // 初始启用/禁用状态 (neq x 1)
    mjtNum* eq_solref;            // 约束求解器参考参数 (neq x mjNREF)
    mjtNum* eq_solimp;            // 约束求解器阻抗参数 (neq x mjNIMP)
    mjtNum* eq_data;              // 约束数值数据 (neq x mjNEQDATA)

    // 肌腱属性
    int* tendon_adr;           // 肌腱路径中第一个对象的地址 (ntendon x 1)
    int* tendon_num;           // 肌腱路径中的对象数量 (ntendon x 1)
    int* tendon_matid;         // 渲染材质ID (ntendon x 1)
    int* tendon_group;         // 可见性分组 (ntendon x 1)
    mjtByte* tendon_limited;       // 是否有长度限制 (ntendon x 1)
    mjtNum* tendon_width;         // 渲染宽度 (ntendon x 1)
    mjtNum* tendon_solref_lim;    // 约束求解器参考参数（限制） (ntendon x mjNREF)
    mjtNum* tendon_solimp_lim;    // 约束求解器阻抗参数（限制） (ntendon x mjNIMP)
    mjtNum* tendon_solref_fri;    // 约束求解器参考参数（摩擦） (ntendon x mjNREF)
    mjtNum* tendon_solimp_fri;    // 约束求解器阻抗参数（摩擦） (ntendon x mjNIMP)
    mjtNum* tendon_range;         // 肌腱长度限制 (ntendon x 2)
    mjtNum* tendon_margin;        // 限制检测的最小距离 (ntendon x 1)
    mjtNum* tendon_stiffness;     // 刚度系数 (ntendon x 1)
    mjtNum* tendon_damping;       // 阻尼系数 (ntendon x 1)
    mjtNum* tendon_frictionloss;  // 摩擦损耗 (ntendon x 1)
    mjtNum* tendon_lengthspring;  // 弹簧静止长度范围 (ntendon x 2)
    mjtNum* tendon_length0;       // 初始姿态下的肌腱长度 (ntendon x 1)
    mjtNum* tendon_invweight0;    // 初始姿态下的逆权重 (ntendon x 1)
    mjtNum* tendon_user;          // 用户自定义数据 (ntendon x nuser_tendon)
    float* tendon_rgba;          // 未指定材质时的RGBA颜色 (ntendon x 4)

    // 肌腱路径中的包裹对象
    int* wrap_type;            // 包裹对象类型（mjtWrap枚举） (nwrap x 1)
    int* wrap_objid;           // 对象ID（几何体、站点、关节） (nwrap x 1)
    mjtNum* wrap_prm;             // 参数（除数、关节系数或站点ID） (nwrap x 1)

    // 执行器属性
    int* actuator_trntype;     // 传动类型（mjtTrn枚举） (nu x 1)
    int* actuator_dyntype;     // 动力学类型（mjtDyn枚举） (nu x 1)
    int* actuator_gaintype;    // 增益类型（mjtGain枚举） (nu x 1)
    int* actuator_biastype;    // 偏置类型（mjtBias枚举） (nu x 1)
    int* actuator_trnid;       // 传动目标ID（关节、肌腱、站点） (nu x 2)
    int* actuator_actadr;      // 激活变量起始地址（-1表示无状态） (nu x 1)
    int* actuator_actnum;      // 激活变量数量 (nu x 1)
    int* actuator_group;       // 可见性分组 (nu x 1)
    mjtByte* actuator_ctrllimited; // 控制量是否受限 (nu x 1)
    mjtByte* actuator_forcelimited;// 力是否受限 (nu x 1)
    mjtByte* actuator_actlimited;  // 激活量是否受限 (nu x 1)
    mjtNum* actuator_dynprm;      // 动力学参数 (nu x mjNDYN)
    mjtNum* actuator_gainprm;     // 增益参数 (nu x mjNGAIN)
    mjtNum* actuator_biasprm;     // 偏置参数 (nu x mjNBIAS)
    mjtByte* actuator_actearly;    // 是否在力计算前更新激活 (nu x 1)
    mjtNum* actuator_ctrlrange;   // 控制量范围 (nu x 2)
    mjtNum* actuator_forcerange;  // 力范围 (nu x 2)
    mjtNum* actuator_actrange;    // 激活量范围 (nu x 2)
    mjtNum* actuator_gear;        // 传动比（长度和力的缩放） (nu x 6)
    mjtNum* actuator_cranklength; // 曲柄长度（滑块-曲柄机构） (nu x 1)
    mjtNum* actuator_acc0;        // 初始姿态下单位力的加速度 (nu x 1)
    mjtNum* actuator_length0;     // 初始姿态下的执行器长度 (nu x 1)
    mjtNum* actuator_lengthrange; // 可行执行器长度范围 (nu x 2)
    mjtNum* actuator_user;        // 用户自定义数据 (nu x nuser_actuator)
    int* actuator_plugin;      // 插件实例ID（-1表示非插件） (nu x 1)

    // 传感器属性
    int* sensor_type;          // 传感器类型（mjtSensor枚举） (nsensor x 1)
    int* sensor_datatype;      // 数据类型（mjtDataType枚举） (nsensor x 1)
    int* sensor_needstage;     // 所需计算阶段（mjtStage枚举） (nsensor x 1)
    int* sensor_objtype;       // 传感器对象类型（mjtObj枚举） (nsensor x 1)
    int* sensor_objid;         // 传感器对象ID (nsensor x 1)
    int* sensor_reftype;       // 参考帧类型（mjtObj枚举） (nsensor x 1)
    int* sensor_refid;         // 参考帧ID（-1表示全局坐标系） (nsensor x 1)
    int* sensor_dim;           // 输出标量数量 (nsensor x 1)
    int* sensor_adr;           // 传感器数据在数组中的地址 (nsensor x 1)
    mjtNum* sensor_cutoff;        // 截止值（0表示忽略） (nsensor x 1)
    mjtNum* sensor_noise;         // 噪声标准差 (nsensor x 1)
    mjtNum* sensor_user;          // 用户自定义数据 (nsensor x nuser_sensor)
    int* sensor_plugin;        // 插件实例ID（-1表示非插件） (nsensor x 1)

    // 插件实例
    int* plugin;               // 全局注册的插件槽位号 (nplugin x 1)
    int* plugin_stateadr;      // 插件状态数组中的地址 (nplugin x 1)
    int* plugin_statenum;      // 插件实例的状态数量 (nplugin x 1)
    char* plugin_attr;          // 插件实例的配置属性 (npluginattr x 1)
    int* plugin_attradr;       // 每个实例配置属性的起始地址 (nplugin x 1)

    // 自定义数值字段
    int* numeric_adr;          // 数值字段在numeric_data中的地址 (nnumeric x 1)
    int* numeric_size;         // 数值字段的大小 (nnumeric x 1)
    mjtNum* numeric_data;         // 所有数值字段的数据 (nnumericdata x 1)

    // 自定义文本字段
    int* text_adr;             // 文本在text_data中的地址 (ntext x 1)
    int* text_size;            // 文本字段大小（包含终止符） (ntext x 1)
    char* text_data;            // 所有文本字段数据（0结尾） (ntextdata x 1)

    // 自定义元组字段
    int* tuple_adr;            // 元组在text_data中的地址 (ntuple x 1)
    int* tuple_size;           // 元组中的对象数量 (ntuple x 1)
    int* tuple_objtype;        // 元组中所有对象的类型 (ntupledata x 1)
    int* tuple_objid;          // 元组中所有对象的ID (ntupledata x 1)
    mjtNum* tuple_objprm;         // 元组中所有对象的参数 (ntupledata x 1)

    // 关键帧数据
    mjtNum* key_time;             // 关键帧时间 (nkey x 1)
    mjtNum* key_qpos;             // 关键帧位置 (nkey x nq)
    mjtNum* key_qvel;             // 关键帧速度 (nkey x nv)
    mjtNum* key_act;              // 关键帧激活状态 (nkey x na)
    mjtNum* key_mpos;             // 关键帧运动捕捉位置 (nkey x nmocap*3)
    mjtNum* key_mquat;            // 关键帧运动捕捉四元数 (nkey x nmocap*4)
    mjtNum* key_ctrl;             // 关键帧控制量 (nkey x nu)

    // 名称索引
    int* name_bodyadr;         // 体名称指针 (nbody x 1)
    int* name_jntadr;          // 关节名称指针 (njnt x 1)
    int* name_geomadr;         // 几何体名称指针 (ngeom x 1)
    int* name_siteadr;         // 站点名称指针 (nsite x 1)
    int* name_camadr;          // 相机名称指针 (ncam x 1)
    int* name_lightadr;        // 光源名称指针 (nlight x 1)
    int* name_flexadr;         // 柔性体名称指针 (nflex x 1)
    int* name_meshadr;         // 网格名称指针 (nmesh x 1)
    int* name_skinadr;         // 皮肤名称指针 (nskin x 1)
    int* name_hfieldadr;       // 高度场名称指针 (nhfield x 1)
    int* name_texadr;          // 纹理名称指针 (ntex x 1)
    int* name_matadr;          // 材质名称指针 (nmat x 1)
    int* name_pairadr;         // 几何体对名称指针 (npair x 1)
    int* name_excludeadr;      // 排除对名称指针 (nexclude x 1)
    int* name_eqadr;           // 等式约束名称指针 (neq x 1)
    int* name_tendonadr;       // 肌腱名称指针 (ntendon x 1)
    int* name_actuatoradr;     // 执行器名称指针 (nu x 1)
    int* name_sensoradr;       // 传感器名称指针 (nsensor x 1)
    int* name_numericadr;      // 数值字段名称指针 (nnumeric x 1)
    int* name_textadr;         // 文本字段名称指针 (ntext x 1)
    int* name_tupleadr;        // 元组字段名称指针 (ntuple x 1)
    int* name_keyadr;          // 关键帧名称指针 (nkey x 1)
    int* name_pluginadr;       // 插件实例名称指针 (nplugin x 1)
    char* names;                // 所有对象的名称（0结尾） (nnames x 1)
    int* names_map;            // 名称的内部哈希映射 (nnames_map x 1)

    // 资源路径
    char* paths;                // 资源路径（0结尾） (npaths x 1)
};
typedef struct mjModel_ mjModel;

#endif  // MUJOCO_MJMODEL_H_
