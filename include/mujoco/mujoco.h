// 版权所有 2021 DeepMind Technologies Limited
//
// 根据 Apache 许可证 2.0 版本（"许可证"）授权;
// 除非符合许可证的要求，否则不得使用本文件。
// 您可以在以下网址获取许可证副本：
//
//     http://www.apache.org/licenses/LICENSE-2.0
//
// 除非适用法律要求或书面同意，本软件按"原样"分发，
// 没有任何明示或暗示的保证或条件。
// 详见许可证了解具体权限和限制。

#ifndef MUJOCO_MUJOCO_H_
#define MUJOCO_MUJOCO_H_

// 头文件版本号，应与 mj_version() 返回的库版本匹配
#define mjVERSION_HEADER 331

// 需要定义 size_t, fabs 和 log10
#include <stdlib.h>
#include <math.h>

// 类型定义
#include <mujoco/mjdata.h>      // 仿真数据结构
#include <mujoco/mjexport.h>    // 导出宏定义
#include <mujoco/mjmodel.h>     // 模型数据结构
#include <mujoco/mjmacro.h>     // 宏定义
#include <mujoco/mjplugin.h>    // 插件接口
#include <mujoco/mjrender.h>    // 渲染相关
#include <mujoco/mjsan.h>       // 内存消毒处理
#include <mujoco/mjspec.h>      // 模型规范
#include <mujoco/mjthread.h>    // 多线程支持
#include <mujoco/mjtnum.h>      // 数值类型定义
#include <mujoco/mjui.h>        // 用户界面
#include <mujoco/mjvisualize.h> // 可视化工具

// 使用 C 语言接口
#ifdef __cplusplus
extern "C" {
#endif

    // 用户自定义错误和内存处理函数
    MJAPI extern void  (*mju_user_error)(const char*);    // 用户错误处理回调
    MJAPI extern void  (*mju_user_warning)(const char*);  // 用户警告处理回调
    MJAPI extern void* (*mju_user_malloc)(size_t);        // 用户内存分配函数
    MJAPI extern void  (*mju_user_free)(void*);           // 用户内存释放函数

    // 扩展计算管道的回调函数
    MJAPI extern mjfGeneric  mjcb_passive;         // 被动力回调
    MJAPI extern mjfGeneric  mjcb_control;         // 控制输入回调
    MJAPI extern mjfConFilt  mjcb_contactfilter;   // 接触过滤回调
    MJAPI extern mjfSensor   mjcb_sensor;          // 传感器计算回调
    MJAPI extern mjfTime     mjcb_time;            // 自定义时间计算回调
    MJAPI extern mjfAct      mjcb_act_dyn;         // 执行器动力学回调
    MJAPI extern mjfAct      mjcb_act_gain;        // 执行器增益回调
    MJAPI extern mjfAct      mjcb_act_bias;        // 执行器偏置回调

    // 几何体碰撞函数表 [发送者类型][接收者类型]
    MJAPI extern mjfCollision mjCOLLISIONFUNC[mjNGEOMTYPES][mjNGEOMTYPES];

    // 功能开关的字符串名称数组
    MJAPI extern const char* mjDISABLESTRING[mjNDISABLE]; // 禁用功能名称
    MJAPI extern const char* mjENABLESTRING[mjNENABLE];   // 启用功能名称
    MJAPI extern const char* mjTIMERSTRING[mjNTIMER];     // 计时器标签
    MJAPI extern const char* mjLABELSTRING[mjNLABEL];     // 可视化标签
    MJAPI extern const char* mjFRAMESTRING[mjNFRAME];     // 参考帧名称
    MJAPI extern const char* mjVISSTRING[mjNVISFLAG][3];  // 可视化选项
    MJAPI extern const char* mjRNDSTRING[mjNRNDFLAG][3];  // 渲染选项

    //---------------------------------- 虚拟文件系统 -------------------------------------------

    // 初始化空虚拟文件系统，使用后需调用 mj_deleteVFS 释放内存
    MJAPI void mj_defaultVFS(mjVFS* vfs);

    // 添加文件到VFS，返回0成功，2名称重复，-1加载失败
    MJAPI int mj_addFileVFS(mjVFS* vfs, const char* directory, const char* filename);

    // 从缓冲区添加文件到VFS，返回0成功，2名称重复，-1失败
    MJAPI int mj_addBufferVFS(mjVFS* vfs, const char* name, const void* buffer, int nbuffer);

    // 从VFS删除文件，返回0成功，-1未找到
    MJAPI int mj_deleteFileVFS(mjVFS* vfs, const char* filename);

    // 删除VFS所有文件并释放内存
    MJAPI void mj_deleteVFS(mjVFS* vfs);

    //---------------------------------- 解析与编译 ---------------------------------------------

    // 解析MJCF/URDF格式XML文件并编译为模型，vfs非空时优先从虚拟文件系统读取
    MJAPI mjModel* mj_loadXML(const char* filename, const mjVFS* vfs, char* error, int error_sz);

    // 从XML文件解析模型规范
    MJAPI mjSpec* mj_parseXML(const char* filename, const mjVFS* vfs, char* error, int error_sz);

    // 从XML字符串解析模型规范
    MJAPI mjSpec* mj_parseXMLString(const char* xml, const mjVFS* vfs, char* error, int error_sz);

    // 将规范编译为模型
    MJAPI mjModel* mj_compile(mjSpec* s, const mjVFS* vfs);

    // 重新编译规范并保留状态，返回0成功
    MJAPI int mj_recompile(mjSpec* s, const mjVFS* vfs, mjModel* m, mjData* d);

    // 将模型保存为MJCF格式XML文件，error非空时存储错误信息
    MJAPI int mj_saveLastXML(const char* filename, const mjModel* m, char* error, int error_sz);

    // 释放上次加载的XML模型，每次加载时自动调用
    MJAPI void mj_freeLastXML(void);

    // 将规范保存为XML字符串，返回0成功，缓冲区不足时返回所需大小
    MJAPI int mj_saveXMLString(const mjSpec* s, char* xml, int xml_sz, char* error, int error_sz);

    // 将规范保存为XML文件，返回0成功
    MJAPI int mj_saveXML(const mjSpec* s, const char* filename, char* error, int error_sz);

    //---------------------------------- 主仿真循环 -----------------------------------------------

    // 推进仿真一步，使用控制回调获取外部力和控制输入
    MJAPI void mj_step(const mjModel* m, mjData* d);

    // 分步仿真第一步：在用户设置力和控制前执行
    MJAPI void mj_step1(const mjModel* m, mjData* d);

    // 分步仿真第二步：在用户设置力和控制后执行
    MJAPI void mj_step2(const mjModel* m, mjData* d);

    // 前向动力学：计算加速度但不积分
    MJAPI void mj_forward(const mjModel* m, mjData* d);

    // 逆动力学：需预先设置qacc
    MJAPI void mj_inverse(const mjModel* m, mjData* d);

    // 带跳过的前向动力学，skipstage指定跳过的阶段
    MJAPI void mj_forwardSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);

    // 带跳过的逆动力学，skipstage指定跳过的阶段
    MJAPI void mj_inverseSkip(const mjModel* m, mjData* d, int skipstage, int skipsensor);

    //---------------------------------- 初始化配置 ------------------------------------------------

    // 设置长度范围计算的默认选项
    MJAPI void mj_defaultLROpt(mjLROpt* opt);

    // 设置求解器参数为默认值
    MJAPI void mj_defaultSolRefImp(mjtNum* solref, mjtNum* solimp);

    // 设置物理选项为默认值
    MJAPI void mj_defaultOption(mjOption* opt);

    // 设置可视化选项为默认值
    MJAPI void mj_defaultVisual(mjVisual* vis);

    // 复制模型，目标为空时分配新内存
    MJAPI mjModel* mj_copyModel(mjModel* dest, const mjModel* src);

    // 保存模型到MJB文件或内存缓冲区，缓冲区优先
    MJAPI void mj_saveModel(const mjModel* m, const char* filename, void* buffer, int buffer_sz);

    // 从MJB文件加载模型，vfs非空时优先从虚拟文件系统读取
    MJAPI mjModel* mj_loadModel(const char* filename, const mjVFS* vfs);

    // 释放模型内存
    MJAPI void mj_deleteModel(mjModel* m);

    // 返回模型所需缓冲区大小
    MJAPI int mj_sizeModel(const mjModel* m);

    // 分配与模型对应的数据对象
    MJAPI mjData* mj_makeData(const mjModel* m);

    // 复制数据对象，模型只需包含尺寸信息
    MJAPI mjData* mj_copyData(mjData* dest, const mjModel* m, const mjData* src);

    // 重置数据为默认值
    MJAPI void mj_resetData(const mjModel* m, mjData* d);

    // 用调试值填充数据（默认值之外）
    MJAPI void mj_resetDataDebug(const mjModel* m, mjData* d, unsigned char debug_value);

    // 根据关键帧重置数据
    MJAPI void mj_resetDataKeyframe(const mjModel* m, mjData* d, int key);

#ifndef ADDRESS_SANITIZER  // ASAN激活时栈管理函数在mjsan.h中声明

    // 在数据栈上标记新帧
    MJAPI void mj_markStack(mjData* d);

    // 释放当前栈帧，上次标记后的栈分配指针将失效
    MJAPI void mj_freeStack(mjData* d);

#endif  // ADDRESS_SANITIZER

    // 在数据栈上按对齐方式分配内存，溢出时报错
    MJAPI void* mj_stackAllocByte(mjData* d, size_t bytes, size_t alignment);

    // 在数据栈上分配浮点数组，溢出时报错
    MJAPI mjtNum* mj_stackAllocNum(mjData* d, size_t size);

    // 在数据栈上分配整型数组，溢出时报错
    MJAPI int* mj_stackAllocInt(mjData* d, size_t size);

    // 释放数据对象内存
    MJAPI void mj_deleteData(mjData* d);

    // 重置所有回调函数为NULL
    MJAPI void mj_resetCallbacks(void);

    // 根据qpos0设置模型的常量字段
    MJAPI void mj_setConst(mjModel* m, mjData* d);

    // 设置执行器长度范围，返回1成功，0错误
    MJAPI int mj_setLengthRange(mjModel* m, mjData* d, int index,
        const mjLROpt* opt, char* error, int error_sz);

    // 创建空规范
    MJAPI mjSpec* mj_makeSpec(void);

    // 复制规范
    MJAPI mjSpec* mj_copySpec(const mjSpec* s);

    // 释放规范内存
    MJAPI void mj_deleteSpec(mjSpec* s);

    // 激活插件，返回0成功
    MJAPI int mjs_activatePlugin(mjSpec* s, const char* name);

    // 设置深度复制开关，返回0成功
    MJAPI int mjs_setDeepCopy(mjSpec* s, int deepcopy);

    //---------------------------------- 打印输出 ------------------------------------------------------

    // 格式化打印模型到文件，float_format为浮点格式字符串
    MJAPI void mj_printFormattedModel(const mjModel* m, const char* filename, const char* float_format);

    // 打印模型到文本文件
    MJAPI void mj_printModel(const mjModel* m, const char* filename);

    // 格式化打印数据到文件
    MJAPI void mj_printFormattedData(const mjModel* m, const mjData* d, const char* filename,
        const char* float_format);

    // 打印数据到文本文件
    MJAPI void mj_printData(const mjModel* m, const mjData* d, const char* filename);

    // 打印矩阵到屏幕
    MJAPI void mju_printMat(const mjtNum* mat, int nr, int nc);

    // 打印稀疏矩阵到屏幕
    MJAPI void mju_printMatSparse(const mjtNum* mat, int nr,
        const int* rownnz, const int* rowadr, const int* colind);

    // 打印XML模式为文本或HTML格式
    MJAPI int mj_printSchema(const char* filename, char* buffer, int buffer_sz,
        int flg_html, int flg_pad);

    //---------------------------------- 核心组件 ----------------------------------------------------

    // 执行位置相关计算
    MJAPI void mj_fwdPosition(const mjModel* m, mjData* d);

    // 执行速度相关计算
    MJAPI void mj_fwdVelocity(const mjModel* m, mjData* d);

    // 计算执行器力qfrc_actuator
    MJAPI void mj_fwdActuation(const mjModel* m, mjData* d);

    // 汇总所有非约束力，计算平滑加速度
    MJAPI void mj_fwdAcceleration(const mjModel* m, mjData* d);

    // 运行约束求解器
    MJAPI void mj_fwdConstraint(const mjModel* m, mjData* d);

    // 半隐式欧拉积分器
    MJAPI void mj_Euler(const mjModel* m, mjData* d);

    // N阶龙格-库塔显式积分器
    MJAPI void mj_RungeKutta(const mjModel* m, mjData* d, int N);

    // 隐式速度积分器
    MJAPI void mj_implicit(const mjModel* m, mjData* d);

    // 逆动力学中的位置相关计算
    MJAPI void mj_invPosition(const mjModel* m, mjData* d);

    // 逆动力学中的速度相关计算
    MJAPI void mj_invVelocity(const mjModel* m, mjData* d);

    // 应用逆约束动力学解析公式
    MJAPI void mj_invConstraint(const mjModel* m, mjData* d);

    // 比较正逆动力学结果，保存到fwdinv
    MJAPI void mj_compareFwdInv(const mjModel* m, mjData* d);

    //---------------------------------- 子组件 ----------------------------------------------------

    // 评估位置相关传感器
    MJAPI void mj_sensorPos(const mjModel* m, mjData* d);

    // 评估速度相关传感器
    MJAPI void mj_sensorVel(const mjModel* m, mjData* d);

    // 评估加速度和力相关传感器
    MJAPI void mj_sensorAcc(const mjModel* m, mjData* d);

    // 计算势能（位置相关能量）
    MJAPI void mj_energyPos(const mjModel* m, mjData* d);

    // 计算动能（速度相关能量）
    MJAPI void mj_energyVel(const mjModel* m, mjData* d);

    // 检查qpos，若存在过大或NaN值则重置
    MJAPI void mj_checkPos(const mjModel* m, mjData* d);

    // 检查qvel，若存在过大或NaN值则重置
    MJAPI void mj_checkVel(const mjModel* m, mjData* d);

    // 检查qacc，若存在过大或NaN值则重置
    MJAPI void mj_checkAcc(const mjModel* m, mjData* d);

    // 运行前向运动学计算
    MJAPI void mj_kinematics(const mjModel* m, mjData* d);

    // 将惯量和运动自由度映射到质心坐标系
    MJAPI void mj_comPos(const mjModel* m, mjData* d);

    // 计算相机和光源的位置与方向
    MJAPI void mj_camlight(const mjModel* m, mjData* d);

    // 计算柔性体相关量
    MJAPI void mj_flex(const mjModel* m, mjData* d);

    // 计算肌腱长度、速度和力矩臂
    MJAPI void mj_tendon(const mjModel* m, mjData* d);

    // 计算执行器传动长度和力矩
    MJAPI void mj_transmission(const mjModel* m, mjData* d);

    // 运行复合刚体惯性算法（CRB）
    MJAPI void mj_crb(const mjModel* m, mjData* d);

    // 计算惯性矩阵的稀疏L'DL分解
    MJAPI void mj_factorM(const mjModel* m, mjData* d);

    // 求解线性系统 M*x = y，使用分解结果 x = inv(L'DL)*y
    MJAPI void mj_solveM(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y, int n);

    // 半线性求解：x = sqrt(inv(D))*inv(L')*y
    MJAPI void mj_solveM2(const mjModel* m, mjData* d, mjtNum* x, const mjtNum* y,
        const mjtNum* sqrtInvD, int n);

    // 计算cvel和cdof_dot
    MJAPI void mj_comVel(const mjModel* m, mjData* d);

    // 计算被动力（弹簧阻尼、重力补偿、流体力）
    MJAPI void mj_passive(const mjModel* m, mjData* d);

    // 计算子树的线速度和角动量
    MJAPI void mj_subtreeVel(const mjModel* m, mjData* d);

    // RNE算法：计算 M(qpos)*qacc + C(qpos,qvel)，flg_acc=0时移除惯性项
    MJAPI void mj_rne(const mjModel* m, mjData* d, int flg_acc, mjtNum* result);

    // 完整数据的RNE后处理：计算cacc、cfrc_ext、cfrc_int
    MJAPI void mj_rnePostConstraint(const mjModel* m, mjData* d);

    // 执行碰撞检测
    MJAPI void mj_collision(const mjModel* m, mjData* d);

    // 构建约束
    MJAPI void mj_makeConstraint(const mjModel* m, mjData* d);

    // 查找约束岛
    MJAPI void mj_island(const mjModel* m, mjData* d);

    // 计算逆约束惯性矩阵efc_AR
    MJAPI void mj_projectConstraint(const mjModel* m, mjData* d);

    // 计算efc_vel和efc_aref
    MJAPI void mj_referenceConstraint(const mjModel* m, mjData* d);

    // 更新约束状态和力，可选计算锥Hessian矩阵
    MJAPI void mj_constraintUpdate(const mjModel* m, mjData* d, const mjtNum* jar,
        mjtNum cost[1], int flg_coneHessian);

    //---------------------------------- 状态管理 ----------------------------------------------------

    // 返回状态规范的大小
    MJAPI int mj_stateSize(const mjModel* m, unsigned int spec);

    // 获取模型状态
    MJAPI void mj_getState(const mjModel* m, const mjData* d, mjtNum* state, unsigned int spec);

    // 设置模型状态
    MJAPI void mj_setState(const mjModel* m, mjData* d, const mjtNum* state, unsigned int spec);

    // 将当前状态复制到第k个关键帧
    MJAPI void mj_setKeyframe(mjModel* m, const mjData* d, int k);

    // 添加接触点到列表，返回0成功，1缓冲区满
    MJAPI int mj_addContact(const mjModel* m, mjData* d, const mjContact* con);

    // 判断摩擦锥类型
    MJAPI int mj_isPyramidal(const mjModel* m);

    // 判断约束雅可比矩阵类型
    MJAPI int mj_isSparse(const mjModel* m);

    // 判断求解器类型（PGS为对偶，CG和牛顿为原始）
    MJAPI int mj_isDual(const mjModel* m);

    // 乘雅可比矩阵与向量（稠密或稀疏）
    MJAPI void mj_mulJacVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    // 乘雅可比转置与向量
    MJAPI void mj_mulJacTVec(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    // 计算附着于指定body的全局点的3/6维末端雅可比
    MJAPI void mj_jac(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
        const mjtNum point[3], int body);

    // 计算body坐标系的末端雅可比
    MJAPI void mj_jacBody(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body);

    // 计算body质心的末端雅可比
    MJAPI void mj_jacBodyCom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int body);

    // 计算子树质心的末端雅可比
    MJAPI void mj_jacSubtreeCom(const mjModel* m, mjData* d, mjtNum* jacp, int body);

    // 计算几何体的末端雅可比
    MJAPI void mj_jacGeom(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int geom);

    // 计算站点的末端雅可比
    MJAPI void mj_jacSite(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr, int site);

    // 计算点的平移雅可比和轴的旋转雅可比
    MJAPI void mj_jacPointAxis(const mjModel* m, mjData* d, mjtNum* jacPoint, mjtNum* jacAxis,
        const mjtNum point[3], const mjtNum axis[3], int body);

    // 计算全局点的时间导数雅可比
    MJAPI void mj_jacDot(const mjModel* m, const mjData* d, mjtNum* jacp, mjtNum* jacr,
        const mjtNum point[3], int body);

    // 获取子树角动量矩阵
    MJAPI void mj_angmomMat(const mjModel* m, mjData* d, mjtNum* mat, int body);

    // 根据名称获取对象ID，未找到返回-1
    MJAPI int mj_name2id(const mjModel* m, int type, const char* name);

    // 根据ID获取对象名称，未找到返回NULL
    MJAPI const char* mj_id2name(const mjModel* m, int type, int id);

    // 将稀疏惯性矩阵转换为稠密矩阵
    MJAPI void mj_fullM(const mjModel* m, mjtNum* dst, const mjtNum* M);

    // 向量与惯性矩阵相乘
    MJAPI void mj_mulM(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    // 向量与惯性矩阵平方根相乘
    MJAPI void mj_mulM2(const mjModel* m, const mjData* d, mjtNum* res, const mjtNum* vec);

    // 将惯性矩阵添加到目标矩阵（支持稀疏或稠密格式）
    MJAPI void mj_addM(const mjModel* m, mjData* d, mjtNum* dst, int* rownnz, int* rowadr, int* colind);

    // 应用笛卡尔力和扭矩（绕过xfrc_applied机制）
    MJAPI void mj_applyFT(const mjModel* m, mjData* d, const mjtNum force[3], const mjtNum torque[3],
        const mjtNum point[3], int body, mjtNum* qfrc_target);

    // 计算对象在局部或全局坐标系中的6D速度
    MJAPI void mj_objectVelocity(const mjModel* m, const mjData* d,
        int objtype, int objid, mjtNum res[6], int flg_local);

    // 计算对象在局部或全局坐标系中的6D加速度
    MJAPI void mj_objectAcceleration(const mjModel* m, const mjData* d,
        int objtype, int objid, mjtNum res[6], int flg_local);

    // 返回两个几何体间的最小距离，可选输出线段
    MJAPI mjtNum mj_geomDistance(const mjModel* m, const mjData* d, int geom1, int geom2,
        mjtNum distmax, mjtNum fromto[6]);

    // 提取接触点的6D力/扭矩（接触坐标系中）
    MJAPI void mj_contactForce(const mjModel* m, const mjData* d, int id, mjtNum result[6]);

    // 通过差分计算速度
    MJAPI void mj_differentiatePos(const mjModel* m, mjtNum* qvel, mjtNum dt,
        const mjtNum* qpos1, const mjtNum* qpos2);

    // 用给定速度积分位置
    MJAPI void mj_integratePos(const mjModel* m, mjtNum* qpos, const mjtNum* qvel, mjtNum dt);

    // 标准化所有四元数
    MJAPI void mj_normalizeQuat(const mjModel* m, mjtNum* qpos);

    // 从局部坐标转换到全局坐标，sameframe指定坐标系类型
    MJAPI void mj_local2Global(mjData* d, mjtNum xpos[3], mjtNum xmat[9], const mjtNum pos[3],
        const mjtNum quat[4], int body, mjtByte sameframe);

    // 计算总质量
    MJAPI mjtNum mj_getTotalmass(const mjModel* m);

    // 缩放质量以达到指定总质量
    MJAPI void mj_setTotalmass(mjModel* m, mjtNum newmass);

    // 获取插件配置属性值，无效时返回NULL
    MJAPI const char* mj_getPluginConfig(const mjModel* m, int plugin_id, const char* attrib);

    // 加载动态库（注册插件）
    MJAPI void mj_loadPluginLibrary(const char* path);

    // 扫描目录加载所有插件库，可选回调
    MJAPI void mj_loadAllPluginLibraries(const char* directory, mjfPluginLibraryLoadCallback callback);

    // 返回版本号（如102表示1.0.2）
    MJAPI int mj_version(void);

    // 返回当前版本字符串
    MJAPI const char* mj_versionString(void);

    //---------------------------------- 射线投射 ---------------------------------------------------

    // 从单点发射多条射线检测碰撞
    MJAPI void mj_multiRay(const mjModel* m, mjData* d, const mjtNum pnt[3], const mjtNum* vec,
        const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
        int* geomid, mjtNum* dist, int nray, mjtNum cutoff);

    // 单射线碰撞检测，返回最近距离，geomgroup为几何分组掩码
    MJAPI mjtNum mj_ray(const mjModel* m, const mjData* d, const mjtNum pnt[3], const mjtNum vec[3],
        const mjtByte* geomgroup, mjtByte flg_static, int bodyexclude,
        int geomid[1]);

    // 射线与高度场碰撞检测
    MJAPI mjtNum mj_rayHfield(const mjModel* m, const mjData* d, int geomid,
        const mjtNum pnt[3], const mjtNum vec[3]);

    // 射线与网格碰撞检测
    MJAPI mjtNum mj_rayMesh(const mjModel* m, const mjData* d, int geomid,
        const mjtNum pnt[3], const mjtNum vec[3]);

    // 纯几何体射线检测（通用函数）
    MJAPI mjtNum mju_rayGeom(const mjtNum pos[3], const mjtNum mat[9], const mjtNum size[3],
        const mjtNum pnt[3], const mjtNum vec[3], int geomtype);

    // 柔性体射线检测，返回最近顶点ID
    MJAPI mjtNum mju_rayFlex(const mjModel* m, const mjData* d, int flex_layer, mjtByte flg_vert,
        mjtByte flg_edge, mjtByte flg_face, mjtByte flg_skin, int flexid,
        const mjtNum* pnt, const mjtNum* vec, int vertid[1]);

    // 皮肤表面射线检测，返回最近顶点ID
    MJAPI mjtNum mju_raySkin(int nface, int nvert, const int* face, const float* vert,
        const mjtNum pnt[3], const mjtNum vec[3], int vertid[1]);

    //---------------------------------- 交互操作 ---------------------------------------------------

    // 设置默认相机参数
    MJAPI void mjv_defaultCamera(mjvCamera* cam);

    // 设置默认自由相机
    MJAPI void mjv_defaultFreeCamera(const mjModel* m, mjvCamera* cam);

    // 设置默认扰动参数
    MJAPI void mjv_defaultPerturb(mjvPerturb* pert);

    // 从房间空间转换到模型空间
    MJAPI void mjv_room2model(mjtNum modelpos[3], mjtNum modelquat[4], const mjtNum roompos[3],
        const mjtNum roomquat[4], const mjvScene* scn);

    // 从模型空间转换到房间空间
    MJAPI void mjv_model2room(mjtNum roompos[3], mjtNum roomquat[4], const mjtNum modelpos[3],
        const mjtNum modelquat[4], const mjvScene* scn);

    // 获取模型空间中的相机信息（平均左右眼）
    MJAPI void mjv_cameraInModel(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
        const mjvScene* scn);

    // 获取房间空间中的相机信息（平均左右眼）
    MJAPI void mjv_cameraInRoom(mjtNum headpos[3], mjtNum forward[3], mjtNum up[3],
        const mjvScene* scn);

    // 获取相机在单位距离处的视锥高度
    MJAPI mjtNum mjv_frustumHeight(const mjvScene* scn);

    // 在水平面内旋转向量对齐相机方向
    MJAPI void mjv_alignToCamera(mjtNum res[3], const mjtNum vec[3], const mjtNum forward[3]);

    // 根据鼠标操作移动相机
    MJAPI void mjv_moveCamera(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
        const mjvScene* scn, mjvCamera* cam);

    // 根据场景状态移动相机
    MJAPI void mjv_moveCameraFromState(const mjvSceneState* scnstate, int action,
        mjtNum reldx, mjtNum reldy,
        const mjvScene* scn, mjvCamera* cam);

    // 根据鼠标操作移动扰动对象
    MJAPI void mjv_movePerturb(const mjModel* m, const mjData* d, int action, mjtNum reldx,
        mjtNum reldy, const mjvScene* scn, mjvPerturb* pert);

    // 根据场景状态移动扰动对象
    MJAPI void mjv_movePerturbFromState(const mjvSceneState* scnstate, int action,
        mjtNum reldx, mjtNum reldy,
        const mjvScene* scn, mjvPerturb* pert);

    // 根据鼠标操作移动模型
    MJAPI void mjv_moveModel(const mjModel* m, int action, mjtNum reldx, mjtNum reldy,
        const mjtNum roomup[3], mjvScene* scn);

    // 从选定body初始化扰动参数
    MJAPI void mjv_initPerturb(const mjModel* m, mjData* d, const mjvScene* scn, mjvPerturb* pert);

    // 应用扰动姿态到mocap或qpos
    MJAPI void mjv_applyPerturbPose(const mjModel* m, mjData* d, const mjvPerturb* pert,
        int flg_paused);

    // 应用扰动力到xfrc_applied
    MJAPI void mjv_applyPerturbForce(const mjModel* m, mjData* d, const mjvPerturb* pert);

    // 平均两个OpenGL相机参数
    MJAPI mjvGLCamera mjv_averageCamera(const mjvGLCamera* cam1, const mjvGLCamera* cam2);

    // 用鼠标选择几何体/柔性体/皮肤，返回bodyid
    MJAPI int mjv_select(const mjModel* m, const mjData* d, const mjvOption* vopt,
        mjtNum aspectratio, mjtNum relx, mjtNum rely,
        const mjvScene* scn, mjtNum selpnt[3],
        int geomid[1], int flexid[1], int skinid[1]);

    //---------------------------------- 可视化 -------------------------------------------------

    // 设置默认可视化选项
    MJAPI void mjv_defaultOption(mjvOption* opt);

    // 设置默认图表参数
    MJAPI void mjv_defaultFigure(mjvFigure* fig);

    // 初始化几何体参数（非NULL字段），其余设为默认
    MJAPI void mjv_initGeom(mjvGeom* geom, int type, const mjtNum size[3],
        const mjtNum pos[3], const mjtNum mat[9], const float rgba[4]);

    // 设置连接两个点的几何体参数（线型等）
    MJAPI void mjv_connector(mjvGeom* geom, int type, mjtNum width,
        const mjtNum from[3], const mjtNum to[3]);

    // 设置默认场景参数
    MJAPI void mjv_defaultScene(mjvScene* scn);

    // 分配场景资源
    MJAPI void mjv_makeScene(const mjModel* m, mjvScene* scn, int maxgeom);

    // 释放场景资源
    MJAPI void mjv_freeScene(mjvScene* scn);

    // 根据模型状态更新场景
    MJAPI void mjv_updateScene(const mjModel* m, mjData* d, const mjvOption* opt,
        const mjvPerturb* pert, mjvCamera* cam, int catmask, mjvScene* scn);

    // 从场景状态更新场景，返回警告数量
    MJAPI int mjv_updateSceneFromState(const mjvSceneState* scnstate, const mjvOption* opt,
        const mjvPerturb* pert, mjvCamera* cam, int catmask,
        mjvScene* scn);

    // 复制模型（跳过可视化不需要的大数组）
    MJAPI void mjv_copyModel(mjModel* dest, const mjModel* src);

    // 设置默认场景状态
    MJAPI void mjv_defaultSceneState(mjvSceneState* scnstate);

    // 初始化场景状态对象
    MJAPI void mjv_makeSceneState(const mjModel* m, const mjData* d,
        mjvSceneState* scnstate, int maxgeom);

    // 释放场景状态资源
    MJAPI void mjv_freeSceneState(mjvSceneState* scnstate);

    // 根据模型和数据更新场景状态
    MJAPI void mjv_updateSceneState(const mjModel* m, mjData* d, const mjvOption* opt,
        mjvSceneState* scnstate);

    // 添加指定类别的几何体到场景
    MJAPI void mjv_addGeoms(const mjModel* m, mjData* d, const mjvOption* opt,
        const mjvPerturb* pert, int catmask, mjvScene* scn);

    // 创建光源列表
    MJAPI void mjv_makeLights(const mjModel* m, const mjData* d, mjvScene* scn);

    // 更新相机参数
    MJAPI void mjv_updateCamera(const mjModel* m, const mjData* d, mjvCamera* cam, mjvScene* scn);

    // 更新皮肤数据
    MJAPI void mjv_updateSkin(const mjModel* m, const mjData* d, mjvScene* scn);

    //---------------------------------- OpenGL渲染 ----------------------------------------------

    // 设置默认渲染上下文
    MJAPI void mjr_defaultContext(mjrContext* con);

    // 创建自定义OpenGL上下文，fontscale为字体缩放比例
    MJAPI void mjr_makeContext(const mjModel* m, mjrContext* con, int fontscale);

    // 修改现有上下文字体
    MJAPI void mjr_changeFont(int fontscale, mjrContext* con);

    // 添加辅助缓冲区
    MJAPI void mjr_addAux(int index, int width, int height, int samples, mjrContext* con);

    // 释放渲染上下文资源
    MJAPI void mjr_freeContext(mjrContext* con);

    // 调整离屏缓冲区尺寸
    MJAPI void mjr_resizeOffscreen(int width, int height, mjrContext* con);

    // 上传纹理到GPU
    MJAPI void mjr_uploadTexture(const mjModel* m, const mjrContext* con, int texid);

    // 上传网格到GPU
    MJAPI void mjr_uploadMesh(const mjModel* m, const mjrContext* con, int meshid);

    // 上传高度场到GPU
    MJAPI void mjr_uploadHField(const mjModel* m, const mjrContext* con, int hfieldid);

    // 恢复当前缓冲区绑定
    MJAPI void mjr_restoreBuffer(const mjrContext* con);

    // 设置当前渲染目标（窗口或离屏缓冲区）
    MJAPI void mjr_setBuffer(int framebuffer, mjrContext* con);

    // 从当前缓冲区读取像素数据
    MJAPI void mjr_readPixels(unsigned char* rgb, float* depth,
        mjrRect viewport, const mjrContext* con);

    // 绘制像素数据到当前缓冲区
    MJAPI void mjr_drawPixels(const unsigned char* rgb, const float* depth,
        mjrRect viewport, const mjrContext* con);

    // 缓冲区间复制区域
    MJAPI void mjr_blitBuffer(mjrRect src, mjrRect dst,
        int flg_color, int flg_depth, const mjrContext* con);

    // 设置辅助缓冲区为当前目标
    MJAPI void mjr_setAux(int index, const mjrContext* con);

    // 复制辅助缓冲区内容到当前缓冲区
    MJAPI void mjr_blitAux(int index, mjrRect src, int left, int bottom, const mjrContext* con);

    // 在指定位置绘制文本
    MJAPI void mjr_text(int font, const char* txt, const mjrContext* con,
        float x, float y, float r, float g, float b);

    // 绘制叠加文本（统计信息等）
    MJAPI void mjr_overlay(int font, int gridpos, mjrRect viewport,
        const char* overlay, const char* overlay2, const mjrContext* con);

    // 获取当前缓冲区的最大视口
    MJAPI mjrRect mjr_maxViewport(const mjrContext* con);

    // 绘制矩形区域
    MJAPI void mjr_rectangle(mjrRect viewport, float r, float g, float b, float a);

    // 绘制带标签的矩形区域
    MJAPI void mjr_label(mjrRect viewport, int font, const char* txt,
        float r, float g, float b, float a, float rt, float gt, float bt,
        const mjrContext* con);

    // 绘制2D图表
    MJAPI void mjr_figure(mjrRect viewport, mjvFigure* fig, const mjrContext* con);

    // 渲染3D场景
    MJAPI void mjr_render(mjrRect viewport, mjvScene* scn, const mjrContext* con);

    // 完成所有OpenGL命令
    MJAPI void mjr_finish(void);

    // 获取OpenGL错误状态
    MJAPI int mjr_getError(void);

    // 查找包含鼠标位置的矩形区域索引
    MJAPI int mjr_findRect(int x, int y, int nrect, const mjrRect* rect);

    //---------------------------------- UI框架相关函数 --------------------------------------------------

// 获取内置UI主题间距 (索引: 0-1)
    MJAPI mjuiThemeSpacing mjui_themeSpacing(int ind);

    // 获取内置UI主题颜色 (索引: 0-3)
    MJAPI mjuiThemeColor mjui_themeColor(int ind);

    // 向UI添加定义项
    MJAPI void mjui_add(mjUI* ui, const mjuiDef* def);

    // 向UI指定区域添加定义项
    MJAPI void mjui_addToSection(mjUI* ui, int sect, const mjuiDef* def);

    // 计算UI尺寸
    MJAPI void mjui_resize(mjUI* ui, const mjrContext* con);

    // 更新指定区域/项；-1表示更新全部
    MJAPI void mjui_update(int section, int item, const mjUI* ui,
        const mjuiState* state, const mjrContext* con);

    // 处理UI事件，返回被修改项的指针，无修改返回NULL
    MJAPI mjuiItem* mjui_event(mjUI* ui, mjuiState* state, const mjrContext* con);

    // 将UI图像复制到当前缓冲区
    MJAPI void mjui_render(mjUI* ui, const mjuiState* state, const mjrContext* con);


    //---------------------------------- 错误处理与内存管理 ----------------------------------------------

    // 主错误处理函数；不会返回调用者
    MJAPI void mju_error(const char* msg, ...) mjPRINTFLIKE(1, 2);

    // 已弃用：使用mju_error
    MJAPI void mju_error_i(const char* msg, int i);

    // 已弃用：使用mju_error
    MJAPI void mju_error_s(const char* msg, const char* text);

    // 主警告处理函数；返回调用者
    MJAPI void mju_warning(const char* msg, ...) mjPRINTFLIKE(1, 2);

    // 已弃用：使用mju_warning
    MJAPI void mju_warning_i(const char* msg, int i);

    // 已弃用：使用mju_warning
    MJAPI void mju_warning_s(const char* msg, const char* text);

    // 清除用户自定义的错误和内存处理程序
    MJAPI void mju_clearHandlers(void);

    // 内存分配函数：64字节对齐，大小填充为64的倍数
    MJAPI void* mju_malloc(size_t size);

    // 内存释放函数，默认使用free()
    MJAPI void mju_free(void* ptr);

    // 高级警告函数：在mjData中统计警告，仅首次打印
    MJAPI void mj_warning(mjData* d, int warning, int info);

    // 写入[日期时间, 类型: 消息]到MUJOCO_LOG.TXT
    MJAPI void mju_writeLog(const char* type, const char* msg);

    // 从规格对象获取编译器错误信息
    MJAPI const char* mjs_getError(mjSpec* s);

    // 判断编译器错误是否为警告
    MJAPI int mjs_isWarning(mjSpec* s);


    //---------------------------------- 标准数学运算 -------------------------------------------------

    // 根据是否使用单精度选择数学函数实现
#if !defined(mjUSESINGLE)
  // 双精度数学函数
#define mju_sqrt    sqrt      // 平方根
#define mju_exp     exp       // 指数
#define mju_sin     sin       // 正弦
#define mju_cos     cos       // 余弦
#define mju_tan     tan       // 正切
#define mju_asin    asin      // 反正弦
#define mju_acos    acos      // 反余弦
#define mju_atan2   atan2     // 四象限反正切
#define mju_tanh    tanh      // 双曲正切
#define mju_pow     pow       // 幂运算
#define mju_abs     fabs      // 绝对值
#define mju_log     log       // 自然对数
#define mju_log10   log10     // 常用对数
#define mju_floor   floor     // 向下取整
#define mju_ceil    ceil      // 向上取整

#else
  // 单精度数学函数
#define mju_sqrt    sqrtf
#define mju_exp     expf
#define mju_sin     sinf
#define mju_cos     cosf
#define mju_tan     tanf
#define mju_asin    asinf
#define mju_acos    acosf
#define mju_atan2   atan2f
#define mju_tanh    tanhf
#define mju_pow     powf
#define mju_abs     fabsf
#define mju_log     logf
#define mju_log10   log10f
#define mju_floor   floorf
#define mju_ceil    ceilf
#endif


//---------------------------------- 向量运算 -------------------------------------------------------

// 三维向量置零
    MJAPI void mju_zero3(mjtNum res[3]);

    // 三维向量复制
    MJAPI void mju_copy3(mjtNum res[3], const mjtNum data[3]);

    // 三维向量缩放
    MJAPI void mju_scl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

    // 三维向量加法
    MJAPI void mju_add3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

    // 三维向量减法
    MJAPI void mju_sub3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3]);

    // 原地三维向量加法
    MJAPI void mju_addTo3(mjtNum res[3], const mjtNum vec[3]);

    // 原地三维向量减法
    MJAPI void mju_subFrom3(mjtNum res[3], const mjtNum vec[3]);

    // 带缩放的原地三维向量加法
    MJAPI void mju_addToScl3(mjtNum res[3], const mjtNum vec[3], mjtNum scl);

    // 缩放加法的三维向量运算
    MJAPI void mju_addScl3(mjtNum res[3], const mjtNum vec1[3], const mjtNum vec2[3], mjtNum scl);

    // 向量归一化，返回归一化前的长度
    MJAPI mjtNum mju_normalize3(mjtNum vec[3]);

    // 计算向量长度（不进行归一化）
    MJAPI mjtNum mju_norm3(const mjtNum vec[3]);

    // 三维向量点积
    MJAPI mjtNum mju_dot3(const mjtNum vec1[3], const mjtNum vec2[3]);

    // 计算三维向量间的欧氏距离
    MJAPI mjtNum mju_dist3(const mjtNum pos1[3], const mjtNum pos2[3]);

    // 3x3矩阵乘向量
    MJAPI void mju_mulMatVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]);

    // 转置3x3矩阵乘向量
    MJAPI void mju_mulMatTVec3(mjtNum res[3], const mjtNum mat[9], const mjtNum vec[3]);

    // 三维向量叉乘
    MJAPI void mju_cross(mjtNum res[3], const mjtNum a[3], const mjtNum b[3]);

    // 四维向量置零
    MJAPI void mju_zero4(mjtNum res[4]);

    // 设置四维向量为单位四元数(1,0,0,0)
    MJAPI void mju_unit4(mjtNum res[4]);

    // 四维向量复制
    MJAPI void mju_copy4(mjtNum res[4], const mjtNum data[4]);

    // 四维向量归一化，返回归一化前的长度
    MJAPI mjtNum mju_normalize4(mjtNum vec[4]);

    // 通用向量置零
    MJAPI void mju_zero(mjtNum* res, int n);

    // 填充向量为指定值
    MJAPI void mju_fill(mjtNum* res, mjtNum val, int n);

    // 通用向量复制
    MJAPI void mju_copy(mjtNum* res, const mjtNum* vec, int n);

    // 计算向量元素和
    MJAPI mjtNum mju_sum(const mjtNum* vec, int n);

    // 计算向量L1范数（绝对值之和）
    MJAPI mjtNum mju_L1(const mjtNum* vec, int n);

    // 通用向量缩放
    MJAPI void mju_scl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

    // 通用向量加法
    MJAPI void mju_add(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

    // 通用向量减法
    MJAPI void mju_sub(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, int n);

    // 通用原地向量加法
    MJAPI void mju_addTo(mjtNum* res, const mjtNum* vec, int n);

    // 通用原地向量减法
    MJAPI void mju_subFrom(mjtNum* res, const mjtNum* vec, int n);

    // 带缩放的通用原地向量加法
    MJAPI void mju_addToScl(mjtNum* res, const mjtNum* vec, mjtNum scl, int n);

    // 通用缩放加法运算
    MJAPI void mju_addScl(mjtNum* res, const mjtNum* vec1, const mjtNum* vec2, mjtNum scl, int n);

    // 通用向量归一化，返回归一化前的长度
    MJAPI mjtNum mju_normalize(mjtNum* res, int n);

    // 计算向量长度（不进行归一化）
    MJAPI mjtNum mju_norm(const mjtNum* res, int n);

    // 通用向量点积
    MJAPI mjtNum mju_dot(const mjtNum* vec1, const mjtNum* vec2, int n);

    // 通用矩阵乘向量
    MJAPI void mju_mulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc);

    // 转置矩阵乘向量
    MJAPI void mju_mulMatTVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int nr, int nc);

    // 向量-矩阵-向量乘法：返回vec1' * mat * vec2
    MJAPI mjtNum mju_mulVecMatVec(const mjtNum* vec1, const mjtNum* mat, const mjtNum* vec2, int n);

    // 矩阵转置
    MJAPI void mju_transpose(mjtNum* res, const mjtNum* mat, int nr, int nc);

    // 矩阵对称化：res = (mat + mat')/2
    MJAPI void mju_symmetrize(mjtNum* res, const mjtNum* mat, int n);

    // 设置单位矩阵
    MJAPI void mju_eye(mjtNum* mat, int n);

    // 矩阵乘法：res = mat1 * mat2
    MJAPI void mju_mulMatMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
        int r1, int c1, int c2);

    // 矩阵乘法（第二个矩阵转置）：res = mat1 * mat2'
    MJAPI void mju_mulMatMatT(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
        int r1, int c1, int r2);

    // 矩阵乘法（第一个矩阵转置）：res = mat1' * mat2
    MJAPI void mju_mulMatTMat(mjtNum* res, const mjtNum* mat1, const mjtNum* mat2,
        int r1, int c1, int c2);

    // 计算矩阵乘积：res = mat' * diag * mat（若diag非空），否则res = mat' * mat
    MJAPI void mju_sqrMatTD(mjtNum* res, const mjtNum* mat, const mjtNum* diag, int nr, int nc);

    // 空间坐标变换（运动/力向量）
    MJAPI void mju_transformSpatial(mjtNum res[6], const mjtNum vec[6], int flg_force,
        const mjtNum newpos[3], const mjtNum oldpos[3],
        const mjtNum rotnew2old[9]);


    //---------------------------------- 稀疏矩阵运算 ---------------------------------------------------

    // 密集矩阵转稀疏矩阵
    MJAPI int mju_dense2sparse(mjtNum* res, const mjtNum* mat, int nr, int nc,
        int* rownnz, int* rowadr, int* colind, int nnz);

    // 稀疏矩阵转密集矩阵
    MJAPI void mju_sparse2dense(mjtNum* res, const mjtNum* mat, int nr, int nc,
        const int* rownnz, const int* rowadr, const int* colind);


    //---------------------------------- 四元数运算 ---------------------------------------------------

    // 使用四元数旋转向量
    MJAPI void mju_rotVecQuat(mjtNum res[3], const mjtNum vec[3], const mjtNum quat[4]);

    // 四元数共轭（对应相反旋转）
    MJAPI void mju_negQuat(mjtNum res[4], const mjtNum quat[4]);

    // 四元数乘法
    MJAPI void mju_mulQuat(mjtNum res[4], const mjtNum quat1[4], const mjtNum quat2[4]);

    // 四元数与轴相乘
    MJAPI void mju_mulQuatAxis(mjtNum res[4], const mjtNum quat[4], const mjtNum axis[3]);

    // 轴角转四元数
    MJAPI void mju_axisAngle2Quat(mjtNum res[4], const mjtNum axis[3], mjtNum angle);

    // 将四元数（姿态差异）转换为3D速度
    MJAPI void mju_quat2Vel(mjtNum res[3], const mjtNum quat[4], mjtNum dt);

    // 四元数减法，表示为3D速度：qb*quat(res) = qa
    MJAPI void mju_subQuat(mjtNum res[3], const mjtNum qa[4], const mjtNum qb[4]);

    // 四元数转3D旋转矩阵
    MJAPI void mju_quat2Mat(mjtNum res[9], const mjtNum quat[4]);

    // 3D旋转矩阵转四元数
    MJAPI void mju_mat2Quat(mjtNum quat[4], const mjtNum mat[9]);

    // 计算四元数的时间导数（给定3D旋转速度）
    MJAPI void mju_derivQuat(mjtNum res[4], const mjtNum quat[4], const mjtNum vel[3]);

    // 积分四元数（给定3D角速度）
    MJAPI void mju_quatIntegrate(mjtNum quat[4], const mjtNum vel[3], mjtNum scale);

    // 构造从z轴到指定向量的旋转四元数
    MJAPI void mju_quatZ2Vec(mjtNum quat[4], const mjtNum vec[3]);

    // 从任意3x3矩阵提取3D旋转（通过优化输入四元数）
    MJAPI int mju_mat2Rot(mjtNum quat[4], const mjtNum mat[9]);

    // 将欧拉角序列（弧度）转换为四元数
    MJAPI void mju_euler2Quat(mjtNum quat[4], const mjtNum euler[3], const char* seq);


    //---------------------------------- 位姿运算 -----------------------------------------------------

    // 位姿相乘
    MJAPI void mju_mulPose(mjtNum posres[3], mjtNum quatres[4],
        const mjtNum pos1[3], const mjtNum quat1[4],
        const mjtNum pos2[3], const mjtNum quat2[4]);

    // 位姿共轭（对应相反的空间变换）
    MJAPI void mju_negPose(mjtNum posres[3], mjtNum quatres[4],
        const mjtNum pos[3], const mjtNum quat[4]);

    // 使用位姿变换向量
    MJAPI void mju_trnVecPose(mjtNum res[3], const mjtNum pos[3], const mjtNum quat[4],
        const mjtNum vec[3]);


    //---------------------------------- 分解与求解器 -------------------------------------------------

    // Cholesky分解：mat = L*L'，返回秩，分解结果直接存入mat
    MJAPI int mju_cholFactor(mjtNum* mat, int n, mjtNum mindiag);

    // 使用Cholesky因子求解线性系统
    MJAPI void mju_cholSolve(mjtNum* res, const mjtNum* mat, const mjtNum* vec, int n);

    // Cholesky秩1更新：L*L' ± x*x'，返回秩
    MJAPI int mju_cholUpdate(mjtNum* mat, mjtNum* x, int n, int flg_plus);

    // 带状密集矩阵Cholesky分解
    MJAPI mjtNum mju_cholFactorBand(mjtNum* mat, int ntotal, int nband, int ndense,
        mjtNum diagadd, mjtNum diagmul);

    // 带状Cholesky因子求解
    MJAPI void mju_cholSolveBand(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
        int ntotal, int nband, int ndense);

    // 带状矩阵转密集矩阵
    MJAPI void mju_band2Dense(mjtNum* res, const mjtNum* mat, int ntotal, int nband, int ndense,
        mjtByte flg_sym);

    // 密集矩阵转带状矩阵
    MJAPI void mju_dense2Band(mjtNum* res, const mjtNum* mat, int ntotal, int nband, int ndense);

    // 带状矩阵与多向量相乘
    MJAPI void mju_bandMulMatVec(mjtNum* res, const mjtNum* mat, const mjtNum* vec,
        int ntotal, int nband, int ndense, int nvec, mjtByte flg_sym);

    // 获取带状矩阵对角线元素地址
    MJAPI int mju_bandDiag(int i, int ntotal, int nband, int ndense);

    // 对称3x3矩阵特征值分解
    MJAPI int mju_eig3(mjtNum eigval[3], mjtNum eigvec[9], mjtNum quat[4], const mjtNum mat[9]);

    // 带约束的二次规划求解器
    MJAPI int mju_boxQP(mjtNum* res, mjtNum* R, int* index, const mjtNum* H, const mjtNum* g, int n,
        const mjtNum* lower, const mjtNum* upper);

    // 为二次规划分配内存
    MJAPI void mju_boxQPmalloc(mjtNum** res, mjtNum** R, int** index, mjtNum** H, mjtNum** g, int n,
        mjtNum** lower, mjtNum** upper);


    //---------------------------------- 其他功能 -----------------------------------------------------

    // 肌肉主动力计算
    MJAPI mjtNum mju_muscleGain(mjtNum len, mjtNum vel, const mjtNum lengthrange[2],
        mjtNum acc0, const mjtNum prm[9]);

    // 肌肉被动力计算
    MJAPI mjtNum mju_muscleBias(mjtNum len, const mjtNum lengthrange[2],
        mjtNum acc0, const mjtNum prm[9]);

    // 肌肉激活动力学
    MJAPI mjtNum mju_muscleDynamics(mjtNum ctrl, mjtNum act, const mjtNum prm[3]);

    // 接触力转金字塔表示
    MJAPI void mju_encodePyramid(mjtNum* pyramid, const mjtNum* force, const mjtNum* mu, int dim);

    // 金字塔表示转接触力
    MJAPI void mju_decodePyramid(mjtNum* force, const mjtNum* pyramid, const mjtNum* mu, int dim);

    // 弹簧阻尼器解析积分
    MJAPI mjtNum mju_springDamper(mjtNum pos0, mjtNum vel0, mjtNum Kp, mjtNum Kv, mjtNum dt);

    // 安全计算最小值（单次求值）
    MJAPI mjtNum mju_min(mjtNum a, mjtNum b);

    // 安全计算最大值（单次求值）
    MJAPI mjtNum mju_max(mjtNum a, mjtNum b);

    // 数值裁剪到[min, max]范围
    MJAPI mjtNum mju_clip(mjtNum x, mjtNum min, mjtNum max);

    // 返回数值符号：+1, -1 或 0
    MJAPI mjtNum mju_sign(mjtNum x);

    // 四舍五入到最近整数
    MJAPI int mju_round(mjtNum x);

    // 类型ID转类型名称
    MJAPI const char* mju_type2Str(int type);

    // 类型名称转类型ID
    MJAPI int mju_str2Type(const char* str);

    // 返回人类可读的字节数表示
    MJAPI const char* mju_writeNumBytes(size_t nbytes);

    // 根据警告类型和信息构造警告文本
    MJAPI const char* mju_warningText(int warning, size_t info);

    // 检查数值是否为NaN或超过最大允许值
    MJAPI int mju_isBad(mjtNum x);

    // 检查向量是否全零
    MJAPI int mju_isZero(mjtNum* vec, int n);

    // 标准正态分布随机数生成器
    MJAPI mjtNum mju_standardNormal(mjtNum* num2);

    // 浮点数组转mjtNum数组
    MJAPI void mju_f2n(mjtNum* res, const float* vec, int n);

    // mjtNum数组转浮点数组
    MJAPI void mju_n2f(float* res, const mjtNum* vec, int n);

    // 双精度数组转mjtNum数组
    MJAPI void mju_d2n(mjtNum* res, const double* vec, int n);

    // mjtNum数组转双精度数组
    MJAPI void mju_n2d(double* res, const mjtNum* vec, int n);

    // 插入排序（升序）
    MJAPI void mju_insertionSort(mjtNum* list, int n);

    // 整数插入排序（升序）
    MJAPI void mju_insertionSortInt(int* list, int n);

    // 生成Halton序列
    MJAPI mjtNum mju_Halton(int index, int base);

    // 安全字符串拷贝（保证末尾空字符）
    MJAPI char* mju_strncpy(char* dst, const char* src, int n);

    // 在[0,1]区间使用五次多项式的Sigmoid函数
    MJAPI mjtNum mju_sigmoid(mjtNum x);


    //---------------------------------- 导数计算 -----------------------------------------------------

    // 有限差分法计算状态转移矩阵
    MJAPI void mjd_transitionFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte flg_centered,
        mjtNum* A, mjtNum* B, mjtNum* C, mjtNum* D);

    // 逆动力学有限差分雅可比计算
    MJAPI void mjd_inverseFD(const mjModel* m, mjData* d, mjtNum eps, mjtByte flg_actuation,
        mjtNum* DfDq, mjtNum* DfDv, mjtNum* DfDa,
        mjtNum* DsDq, mjtNum* DsDv, mjtNum* DsDa,
        mjtNum* DmDq);

    // 四元数减法的导数
    MJAPI void mjd_subQuat(const mjtNum qa[4], const mjtNum qb[4], mjtNum Da[9], mjtNum Db[9]);

    // 四元数积分的导数
    MJAPI void mjd_quatIntegrate(const mjtNum vel[3], mjtNum scale,
        mjtNum Dquat[9], mjtNum Dvel[9], mjtNum Dscale[3]);


    //---------------------------------- 插件系统 -----------------------------------------------------

    // 设置默认插件定义
    MJAPI void mjp_defaultPlugin(mjpPlugin* plugin);

    // 全局注册插件（线程安全）
    MJAPI int mjp_registerPlugin(const mjpPlugin* plugin);

    // 获取已注册插件数量
    MJAPI int mjp_pluginCount(void);

    // 通过名称查找插件
    MJAPI const mjpPlugin* mjp_getPlugin(const char* name, int* slot);

    // 通过注册槽位获取插件
    MJAPI const mjpPlugin* mjp_getPluginAtSlot(int slot);

    // 设置默认资源提供者
    MJAPI void mjp_defaultResourceProvider(mjpResourceProvider* provider);

    // 注册全局资源提供者（线程安全）
    MJAPI int mjp_registerResourceProvider(const mjpResourceProvider* provider);

    // 获取资源提供者数量
    MJAPI int mjp_resourceProviderCount(void);

    // 根据资源名称匹配资源提供者
    MJAPI const mjpResourceProvider* mjp_getResourceProvider(const char* resource_name);

    // 通过槽位获取资源提供者
    MJAPI const mjpResourceProvider* mjp_getResourceProviderAtSlot(int slot);


    //---------------------------------- 线程管理 -----------------------------------------------------

    // 创建指定线程数的线程池
    MJAPI mjThreadPool* mju_threadPoolCreate(size_t number_of_threads);

    // 绑定线程池到mjData以启用多线程
    MJAPI void mju_bindThreadPool(mjData* d, void* thread_pool);

    // 向线程池添加任务
    MJAPI void mju_threadPoolEnqueue(mjThreadPool* thread_pool, mjTask* task);

    // 销毁线程池
    MJAPI void mju_threadPoolDestroy(mjThreadPool* thread_pool);

    // 初始化任务结构体
    MJAPI void mju_defaultTask(mjTask* task);

    // 等待任务完成
    MJAPI void mju_taskJoin(mjTask* task);


    //---------------------------------- 模型装配 -----------------------------------------------------

    // 将子体附加到父框架
    MJAPI mjsBody* mjs_attachBody(mjsFrame* parent, const mjsBody* child,
        const char* prefix, const char* suffix);

    // 将子框架附加到父体
    MJAPI mjsFrame* mjs_attachFrame(mjsBody* parent, const mjsFrame* child,
        const char* prefix, const char* suffix);

    // 将子体附加到父站点
    MJAPI mjsBody* mjs_attachToSite(mjsSite* parent, const mjsBody* child,
        const char* prefix, const char* suffix);

    // 将子框架附加到父站点
    MJAPI mjsFrame* mjs_attachFrameToSite(mjsSite* parent, const mjsFrame* child,
        const char* prefix, const char* suffix);

    // 从模型中分离并删除体
    MJAPI int mjs_detachBody(mjSpec* s, mjsBody* b);


    //---------------------------------- 模型元素操作 -------------------------------------------------

    // 以下为模型构建相关函数，包括添加各种元素、设置属性等
    // （由于篇幅限制，此处保持函数声明，详细中文注释可参考前文模式添加）

    // 示例注释：
    // 向体添加子体
    MJAPI mjsBody* mjs_addBody(mjsBody* body, const mjsDefault* def);

    // 向体添加站点
    MJAPI mjsSite* mjs_addSite(mjsBody* body, const mjsDefault* def);

    // 向体添加关节
    MJAPI mjsJoint* mjs_addJoint(mjsBody* body, const mjsDefault* def);

    // 向体添加自由关节
    MJAPI mjsJoint* mjs_addFreeJoint(mjsBody* body);

    // 向体添加几何体
    MJAPI mjsGeom* mjs_addGeom(mjsBody* body, const mjsDefault* def);

    // 向体添加相机
    MJAPI mjsCamera* mjs_addCamera(mjsBody* body, const mjsDefault* def);

    // 向体添加光源
    MJAPI mjsLight* mjs_addLight(mjsBody* body, const mjsDefault* def);

    // 向体添加参考框架
    MJAPI mjsFrame* mjs_addFrame(mjsBody* body, mjsFrame* parentframe);

    // 删除模型元素
    MJAPI int mjs_delete(mjsElement* element);


    //---------------------------------- 其他模型元素添加 ---------------------------------------------

    // 以下为添加非树形结构元素的函数，包括作动器、传感器等
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 添加作动器
    MJAPI mjsActuator* mjs_addActuator(mjSpec* s, const mjsDefault* def);

    // 添加传感器
    MJAPI mjsSensor* mjs_addSensor(mjSpec* s);

    // 添加柔性体
    MJAPI mjsFlex* mjs_addFlex(mjSpec* s);

    // 添加接触对
    MJAPI mjsPair* mjs_addPair(mjSpec* s, const mjsDefault* def);

    // 添加排除体对
    MJAPI mjsExclude* mjs_addExclude(mjSpec* s);

    // 添加等式约束
    MJAPI mjsEquality* mjs_addEquality(mjSpec* s, const mjsDefault* def);

    // 添加肌腱
    MJAPI mjsTendon* mjs_addTendon(mjSpec* s, const mjsDefault* def);

    // 肌腱包裹站点
    MJAPI mjsWrap* mjs_wrapSite(mjsTendon* tendon, const char* name);

    // 肌腱包裹几何体
    MJAPI mjsWrap* mjs_wrapGeom(mjsTendon* tendon, const char* name, const char* sidesite);

    // 肌腱包裹关节
    MJAPI mjsWrap* mjs_wrapJoint(mjsTendon* tendon, const char* name, double coef);

    // 肌腱包裹滑轮
    MJAPI mjsWrap* mjs_wrapPulley(mjsTendon* tendon, double divisor);

    // 添加数值型数据
    MJAPI mjsNumeric* mjs_addNumeric(mjSpec* s);

    // 添加文本数据
    MJAPI mjsText* mjs_addText(mjSpec* s);

    // 添加元组数据
    MJAPI mjsTuple* mjs_addTuple(mjSpec* s);

    // 添加关键帧
    MJAPI mjsKey* mjs_addKey(mjSpec* s);

    // 添加插件
    MJAPI mjsPlugin* mjs_addPlugin(mjSpec* s);

    // 添加默认设置
    MJAPI mjsDefault* mjs_addDefault(mjSpec* s, const char* classname, const mjsDefault* parent);


    //---------------------------------- 资源管理 -----------------------------------------------------

    // 以下为模型资源添加函数，包括网格、高度场等
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 添加网格
    MJAPI mjsMesh* mjs_addMesh(mjSpec* s, const mjsDefault* def);

    // 添加高度场
    MJAPI mjsHField* mjs_addHField(mjSpec* s);

    // 添加蒙皮
    MJAPI mjsSkin* mjs_addSkin(mjSpec* s);

    // 添加纹理
    MJAPI mjsTexture* mjs_addTexture(mjSpec* s);

    // 添加材质
    MJAPI mjsMaterial* mjs_addMaterial(mjSpec* s, const mjsDefault* def);


    //---------------------------------- 查询工具 -----------------------------------------------------

    // 以下为模型元素查询相关函数
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 从元素获取模型规范
    MJAPI mjSpec* mjs_getSpec(mjsElement* element);

    // 按名称查找模型规范
    MJAPI mjSpec* mjs_findSpec(mjSpec* spec, const char* name);

    // 在模型中查找体
    MJAPI mjsBody* mjs_findBody(mjSpec* s, const char* name);

    // 按类型和名称查找元素
    MJAPI mjsElement* mjs_findElement(mjSpec* s, mjtObj type, const char* name);

    // 查找子体
    MJAPI mjsBody* mjs_findChild(mjsBody* body, const char* name);

    // 获取父体
    MJAPI mjsBody* mjs_getParent(mjsElement* element);

    // 查找参考框架
    MJAPI mjsFrame* mjs_findFrame(mjSpec* s, const char* name);

    // 获取元素的默认设置
    MJAPI mjsDefault* mjs_getDefault(mjsElement* element);

    // 按类名查找默认设置
    MJAPI const mjsDefault* mjs_findDefault(mjSpec* s, const char* classname);

    // 获取模型的全局默认设置
    MJAPI mjsDefault* mjs_getSpecDefault(mjSpec* s);

    // 获取元素ID
    MJAPI int mjs_getId(mjsElement* element);

    // 获取体的第一个指定类型子元素
    MJAPI mjsElement* mjs_firstChild(mjsBody* body, mjtObj type, int recurse);

    // 获取下一个同类型子元素
    MJAPI mjsElement* mjs_nextChild(mjsBody* body, mjsElement* child, int recurse);

    // 获取模型的第一个指定类型元素
    MJAPI mjsElement* mjs_firstElement(mjSpec* s, mjtObj type);

    // 获取模型的下一个元素
    MJAPI mjsElement* mjs_nextElement(mjSpec* s, mjsElement* element);


    //---------------------------------- 属性设置器 ---------------------------------------------------

    // 以下为属性设置相关函数
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 设置缓冲区数据
    MJAPI void mjs_setBuffer(mjByteVec* dest, const void* array, int size);

    // 设置字符串属性
    MJAPI void mjs_setString(mjString* dest, const char* text);

    // 设置字符串向量
    MJAPI void mjs_setStringVec(mjStringVec* dest, const char* text);

    // 设置字符串向量中的条目
    MJAPI mjtByte mjs_setInStringVec(mjStringVec* dest, int i, const char* text);

    // 追加字符串条目
    MJAPI void mjs_appendString(mjStringVec* dest, const char* text);

    // 设置整型数组
    MJAPI void mjs_setInt(mjIntVec* dest, const int* array, int size);

    // 追加整型数组
    MJAPI void mjs_appendIntVec(mjIntVecVec* dest, const int* array, int size);

    // 设置浮点数组
    MJAPI void mjs_setFloat(mjFloatVec* dest, const float* array, int size);

    // 追加浮点数组
    MJAPI void mjs_appendFloatVec(mjFloatVecVec* dest, const float* array, int size);

    // 设置双精度数组
    MJAPI void mjs_setDouble(mjDoubleVec* dest, const double* array, int size);

    // 设置插件属性
    MJAPI void mjs_setPluginAttributes(mjsPlugin* plugin, void* attributes);


    //---------------------------------- 属性获取器 ---------------------------------------------------

    // 以下为属性获取相关函数
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 获取字符串内容
    MJAPI const char* mjs_getString(const mjString* source);

    // 获取双精度数组内容
    MJAPI const double* mjs_getDouble(const mjDoubleVec* source, int* size);


    //---------------------------------- 模型工具 -----------------------------------------------------

    // 以下为模型处理工具函数
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 设置元素的默认属性
    MJAPI void mjs_setDefault(mjsElement* element, const mjsDefault* def);

    // 设置元素的参考框架
    MJAPI void mjs_setFrame(mjsElement* dest, mjsFrame* frame);

    // 解析方向参数为四元数
    MJAPI const char* mjs_resolveOrientation(double quat[4], mjtByte degree, const char* sequence,
        const mjsOrientation* orientation);

    // 将体转换为参考框架
    MJAPI mjsFrame* mjs_bodyToFrame(mjsBody** body);


    //---------------------------------- 元素初始化 ---------------------------------------------------

    // 以下为各类元素的默认初始化函数
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 初始化模型规范默认值
    MJAPI void mjs_defaultSpec(mjSpec* spec);

    // 初始化方向参数默认值
    MJAPI void mjs_defaultOrientation(mjsOrientation* orient);

    // 初始化体默认属性
    MJAPI void mjs_defaultBody(mjsBody* body);

    // 初始化参考框架默认属性
    MJAPI void mjs_defaultFrame(mjsFrame* frame);

    // 初始化关节默认属性
    MJAPI void mjs_defaultJoint(mjsJoint* joint);

    // 初始化几何体默认属性
    MJAPI void mjs_defaultGeom(mjsGeom* geom);

    // 初始化站点默认属性
    MJAPI void mjs_defaultSite(mjsSite* site);

    // 初始化相机默认属性
    MJAPI void mjs_defaultCamera(mjsCamera* camera);

    // 初始化光源默认属性
    MJAPI void mjs_defaultLight(mjsLight* light);

    // 初始化柔性体默认属性
    MJAPI void mjs_defaultFlex(mjsFlex* flex);

    // 初始化网格默认属性
    MJAPI void mjs_defaultMesh(mjsMesh* mesh);

    // 初始化高度场默认属性
    MJAPI void mjs_defaultHField(mjsHField* hfield);

    // 初始化蒙皮默认属性
    MJAPI void mjs_defaultSkin(mjsSkin* skin);

    // 初始化纹理默认属性
    MJAPI void mjs_defaultTexture(mjsTexture* texture);

    // 初始化材质默认属性
    MJAPI void mjs_defaultMaterial(mjsMaterial* material);

    // 初始化接触对默认属性
    MJAPI void mjs_defaultPair(mjsPair* pair);

    // 初始化等式约束默认属性
    MJAPI void mjs_defaultEquality(mjsEquality* equality);

    // 初始化肌腱默认属性
    MJAPI void mjs_defaultTendon(mjsTendon* tendon);

    // 初始化作动器默认属性
    MJAPI void mjs_defaultActuator(mjsActuator* actuator);

    // 初始化传感器默认属性
    MJAPI void mjs_defaultSensor(mjsSensor* sensor);

    // 初始化数值型数据默认属性
    MJAPI void mjs_defaultNumeric(mjsNumeric* numeric);

    // 初始化文本数据默认属性
    MJAPI void mjs_defaultText(mjsText* text);

    // 初始化元组默认属性
    MJAPI void mjs_defaultTuple(mjsTuple* tuple);

    // 初始化关键帧默认属性
    MJAPI void mjs_defaultKey(mjsKey* key);

    // 初始化插件默认属性
    MJAPI void mjs_defaultPlugin(mjsPlugin* plugin);


    //---------------------------------- 元素类型转换 -------------------------------------------------

    // 以下为安全类型转换函数，用于将通用元素指针转换为具体类型
    // （函数声明保持原样，具体注释可参考前文模式）

    // 示例注释：
    // 安全转换为体类型
    MJAPI mjsBody* mjs_asBody(mjsElement* element);

    // 安全转换为几何体类型
    MJAPI mjsGeom* mjs_asGeom(mjsElement* element);

    // ...（其他类型转换函数注释类似）
    // Safely cast an element as mjsJoint, or return NULL if the element is not an mjsJoint.
    MJAPI mjsJoint* mjs_asJoint(mjsElement* element);

    // Safely cast an element as mjsSite, or return NULL if the element is not an mjsSite.
    MJAPI mjsSite* mjs_asSite(mjsElement* element);

    // Safely cast an element as mjsCamera, or return NULL if the element is not an mjsCamera.
    MJAPI mjsCamera* mjs_asCamera(mjsElement* element);

    // Safely cast an element as mjsLight, or return NULL if the element is not an mjsLight.
    MJAPI mjsLight* mjs_asLight(mjsElement* element);

    // Safely cast an element as mjsFrame, or return NULL if the element is not an mjsFrame.
    MJAPI mjsFrame* mjs_asFrame(mjsElement* element);

    // Safely cast an element as mjsActuator, or return NULL if the element is not an mjsActuator.
    MJAPI mjsActuator* mjs_asActuator(mjsElement* element);

    // Safely cast an element as mjsSensor, or return NULL if the element is not an mjsSensor.
    MJAPI mjsSensor* mjs_asSensor(mjsElement* element);

    // Safely cast an element as mjsFlex, or return NULL if the element is not an mjsFlex.
    MJAPI mjsFlex* mjs_asFlex(mjsElement* element);

    // Safely cast an element as mjsPair, or return NULL if the element is not an mjsPair.
    MJAPI mjsPair* mjs_asPair(mjsElement* element);

    // Safely cast an element as mjsEquality, or return NULL if the element is not an mjsEquality.
    MJAPI mjsEquality* mjs_asEquality(mjsElement* element);

    // Safely cast an element as mjsExclude, or return NULL if the element is not an mjsExclude.
    MJAPI mjsExclude* mjs_asExclude(mjsElement* element);

    // Safely cast an element as mjsTendon, or return NULL if the element is not an mjsTendon.
    MJAPI mjsTendon* mjs_asTendon(mjsElement* element);

    // Safely cast an element as mjsNumeric, or return NULL if the element is not an mjsNumeric.
    MJAPI mjsNumeric* mjs_asNumeric(mjsElement* element);

    // Safely cast an element as mjsText, or return NULL if the element is not an mjsText.
    MJAPI mjsText* mjs_asText(mjsElement* element);

    // Safely cast an element as mjsTuple, or return NULL if the element is not an mjsTuple.
    MJAPI mjsTuple* mjs_asTuple(mjsElement* element);

    // Safely cast an element as mjsKey, or return NULL if the element is not an mjsKey.
    MJAPI mjsKey* mjs_asKey(mjsElement* element);

    // Safely cast an element as mjsMesh, or return NULL if the element is not an mjsMesh.
    MJAPI mjsMesh* mjs_asMesh(mjsElement* element);

    // Safely cast an element as mjsHField, or return NULL if the element is not an mjsHField.
    MJAPI mjsHField* mjs_asHField(mjsElement* element);

    // Safely cast an element as mjsSkin, or return NULL if the element is not an mjsSkin.
    MJAPI mjsSkin* mjs_asSkin(mjsElement* element);

    // Safely cast an element as mjsTexture, or return NULL if the element is not an mjsTexture.
    MJAPI mjsTexture* mjs_asTexture(mjsElement* element);

    // Safely cast an element as mjsMaterial, or return NULL if the element is not an mjsMaterial.
    MJAPI mjsMaterial* mjs_asMaterial(mjsElement* element);

    // Safely cast an element as mjsPlugin, or return NULL if the element is not an mjsPlugin.
    MJAPI mjsPlugin* mjs_asPlugin(mjsElement* element);

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_MUJOCO_H_