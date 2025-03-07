// 版权声明和许可证信息，保留不变
// Copyright 2024 DeepMind Technologies Limited
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

#ifndef MUJOCO_INCLUDE_MJSPEC_H_
#define MUJOCO_INCLUDE_MJSPEC_H_

#include <stddef.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>

// C-API 兼容性处理
#ifdef __cplusplus
#include <cstddef>
#include <string>
#include <vector>

extern "C" {
#endif

	//-------------------------------- 字符串和数组的句柄定义 -----------------------------------

#ifdef __cplusplus
  // C++ 使用标准库类型定义
	using mjString = std::string;        // 字符串类型
	using mjStringVec = std::vector<std::string>;  // 字符串向量
	using mjIntVec = std::vector<int>;          // 整型向量
	using mjIntVecVec = std::vector<std::vector<int>>;  // 二维整型向量
	using mjFloatVec = std::vector<float>;        // 浮点型向量
	using mjFloatVecVec = std::vector<std::vector<float>>; // 二维浮点型向量
	using mjDoubleVec = std::vector<double>;       // 双精度浮点型向量
	using mjByteVec = std::vector<std::byte>;    // 字节型向量
#else
  // C 语言使用不透明类型
	typedef void mjString;
	typedef void mjStringVec;
	typedef void mjIntVec;
	typedef void mjIntVecVec;
	typedef void mjFloatVec;
	typedef void mjFloatVecVec;
	typedef void mjDoubleVec;
	typedef void mjByteVec;
#endif


	//-------------------------------- 枚举类型定义 (mjt) ------------------------------------------------

	typedef enum mjtGeomInertia_ {     // 惯性推断类型
		mjINERTIA_VOLUME = 0,            // 质量按体积分布
		mjINERTIA_SHELL,                 // 质量按表面分布
	} mjtGeomInertia;


	typedef enum mjtMeshInertia_ {     // 网格惯性类型
		mjMESH_INERTIA_CONVEX = 0,       // 凸网格惯性
		mjMESH_INERTIA_EXACT,            // 精确网格惯性
		mjMESH_INERTIA_LEGACY,           // 旧版网格惯性
		mjMESH_INERTIA_SHELL             // 表面网格惯性
	} mjtMeshInertia;


	typedef enum mjtBuiltin_ {         // 内置程序化纹理类型
		mjBUILTIN_NONE = 0,              // 无内置纹理
		mjBUILTIN_GRADIENT,              // 渐变纹理: rgb1->rgb2
		mjBUILTIN_CHECKER,               // 棋盘格纹理: rgb1, rgb2
		mjBUILTIN_FLAT                   // 2D: rgb1; 立方体: rgb1-上, rgb2-侧, rgb3-下
	} mjtBuiltin;


	typedef enum mjtMark_ {            // 程序化纹理标记类型
		mjMARK_NONE = 0,                 // 无标记
		mjMARK_EDGE,                     // 边缘线
		mjMARK_CROSS,                    // 十字线
		mjMARK_RANDOM                    // 随机点
	} mjtMark;


	typedef enum mjtLimited_ {         // 限制类型
		mjLIMITED_FALSE = 0,             // 无限定
		mjLIMITED_TRUE,                  // 有限定
		mjLIMITED_AUTO,                  // 根据范围自动推断
	} mjtLimited;

	typedef enum mjtAlignFree_ {       // 自由关节对齐方式
		mjALIGNFREE_FALSE = 0,           // 不对齐
		mjALIGNFREE_TRUE,                // 对齐
		mjALIGNFREE_AUTO,                // 遵循全局编译器标志
	} mjtAlignFree;


	typedef enum mjtInertiaFromGeom_ { // 惯性推断来源
		mjINERTIAFROMGEOM_FALSE = 0,     // 不使用几何体惯性；需显式指定
		mjINERTIAFROMGEOM_TRUE,          // 总是使用几何体惯性；覆盖显式定义
		mjINERTIAFROMGEOM_AUTO           // 仅当惯性元素缺失时使用几何体惯性
	} mjtInertiaFromGeom;


	typedef enum mjtOrientation_ {     // 方向指定类型
		mjORIENTATION_QUAT = 0,          // 四元数
		mjORIENTATION_AXISANGLE,         // 轴角表示
		mjORIENTATION_XYAXES,            // X和Y轴
		mjORIENTATION_ZAXIS,             // Z轴（最小旋转）
		mjORIENTATION_EULER,             // 欧拉角
	} mjtOrientation;


	//-------------------------------- 属性结构体定义 (mjs) -----------------------------------------

	typedef struct mjsElement_ {       // 元素类型基类（不可修改）
		mjtObj elemtype;                 // 元素类型标识
	} mjsElement;


	typedef struct mjsCompiler_ {      // 编译器选项
		mjtByte autolimits;              // 根据范围自动推断"limited"属性
		double boundmass;                // 强制最小体质量
		double boundinertia;             // 强制最小体惯性对角线值
		double settotalmass;             // 重新缩放质量和惯性；<=0时忽略
		mjtByte balanceinertia;          // 自动应用A + B >= C惯性规则
		mjtByte fitaabb;                 // 将网格适配到AABB而非惯性盒
		mjtByte degree;                  // 角度单位（弧度或度）
		char eulerseq[3];                // 欧拉角旋转序列
		mjtByte discardvisual;           // 解析时丢弃视觉几何体
		mjtByte usethread;               // 使用多线程加速编译
		mjtByte fusestatic;              // 将静态体与父体合并
		int inertiafromgeom;             // 使用几何体惯性（见mjtInertiaFromGeom）
		int inertiagrouprange[2];        // 用于计算惯性的几何体组范围
		int alignfree;                   // 对齐自由关节到惯性坐标系
		mjLROpt LRopt;                   // 长度范围计算选项
	} mjsCompiler;


	typedef struct mjSpec_ {           // 模型规格定义
		mjsElement* element;             // 元素类型
		mjString* modelname;             // 模型名称

		// 编译器相关数据
		mjsCompiler compiler;            // 编译器选项
		mjtByte strippath;               // 自动去除网格文件路径
		mjString* meshdir;               // 网格和高度场目录
		mjString* texturedir;            // 纹理目录

		// 引擎相关数据
		mjOption option;                 // 物理选项
		mjVisual visual;                 // 视觉选项
		mjStatistic stat;                // 统计信息覆盖（如果定义）

		// 尺寸参数
		size_t memory;                   // 内存池总字节数（arena+stack）
		int nemax;                       // 最大等式约束数
		int nuserdata;                   // userdata中的mjtNum数量
		int nuser_body;                  // body_user中的mjtNum数量
		int nuser_jnt;                   // jnt_user中的mjtNum数量
		int nuser_geom;                  // geom_user中的mjtNum数量
		int nuser_site;                  // site_user中的mjtNum数量
		int nuser_cam;                   // cam_user中的mjtNum数量
		int nuser_tendon;                // tendon_user中的mjtNum数量
		int nuser_actuator;              // actuator_user中的mjtNum数量
		int nuser_sensor;                // sensor_user中的mjtNum数量
		int nkey;                        // 关键帧数量
		int njmax;                       // (已弃用) 最大约束数
		int nconmax;                     // (已弃用) 最大接触检测数
		size_t nstack;                   // (已弃用) mjData栈中的mjtNum数量

		// 全局数据
		mjString* comment;               // XML顶部注释
		mjString* modelfiledir;          // 模型文件路径

		// 其他
		mjtByte hasImplicitPluginElem;   // 是否已存在隐式插件元素
	} mjSpec;


	typedef struct mjsOrientation_ {   // 替代方向指定结构
		mjtOrientation type;             // 当前使用的方向类型
		double axisangle[4];             // 轴角表示（轴xyz + 角度）
		double xyaxes[6];                // X和Y轴向量（各3个元素）
		double zaxis[3];                 // Z轴向量（最小旋转）
		double euler[3];                 // 欧拉角
	} mjsOrientation;


	typedef struct mjsPlugin_ {        // 插件规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 实例名称
		mjString* plugin_name;           // 插件名称
		mjtByte active;                  // 是否激活
		mjString* info;                  // 附加到编译器错误的信息
	} mjsPlugin;


	typedef struct mjsBody_ {          // 体规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* childclass;            // 子类名称

		// 体坐标系
		double pos[3];                   // 位置坐标
		double quat[4];                  // 四元数方向
		mjsOrientation alt;              // 替代方向表示

		// 惯性坐标系
		double mass;                     // 质量
		double ipos[3];                  // 惯性系位置偏移
		double iquat[4];                 // 惯性系方向偏移
		double inertia[3];               // 惯性矩阵对角线（惯性系下）
		mjsOrientation ialt;             // 惯性系替代方向表示
		double fullinertia[6];           // 完整惯性矩阵（非对角线元素）

		// 其他属性
		mjtByte mocap;                   // 是否为运动捕捉体
		double gravcomp;                 // 重力补偿系数
		mjDoubleVec* userdata;           // 用户数据
		mjtByte explicitinertial;        // 是否显式保存惯性参数
		mjsPlugin plugin;                // 被动力插件
		mjString* info;                  // 附加到编译器错误的信息
	} mjsBody;


	typedef struct mjsFrame_ {         // 框架规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* childclass;            // 子类名称
		double pos[3];                   // 位置坐标
		double quat[4];                  // 四元数方向
		mjsOrientation alt;              // 替代方向表示
		mjString* info;                  // 附加到编译器错误的信息
	} mjsFrame;


	typedef struct mjsJoint_ {         // 关节规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjtJoint type;                   // 关节类型

		// 运动学参数
		double pos[3];                   // 锚点位置
		double axis[3];                  // 关节轴向量
		double ref;                      // 参考配置下的关节位置qpos0
		int align;                       // 自由关节对齐方式（见mjtAlignFree）

		// 刚度参数
		double stiffness;                // 刚度系数
		double springref;                // 弹簧参考值qpos_spring
		double springdamper[2];          // 时间常数和阻尼比

		// 限制参数
		int limited;                     // 是否有限制（见mjtLimited）
		double range[2];                 // 关节运动范围
		double margin;                   // 关节限制检测边距
		mjtNum solref_limit[mjNREF];     // 限制求解器参考参数
		mjtNum solimp_limit[mjNIMP];     // 限制求解器阻抗参数
		int actfrclimited;               // 执行器力是否受限（见mjtLimited）
		double actfrcrange[2];           // 执行器力限制范围

		// 自由度属性
		double armature;                 // 附加惯量（滑动关节为质量）
		double damping;                  // 阻尼系数
		double frictionloss;             // 摩擦损耗
		mjtNum solref_friction[mjNREF];  // 摩擦求解器参考参数
		mjtNum solimp_friction[mjNIMP];  // 摩擦求解器阻抗参数

		// 其他属性
		int group;                       // 分组标识
		mjtByte actgravcomp;             // 是否通过执行器应用重力补偿
		mjDoubleVec* userdata;           // 用户数据
		mjString* info;                  // 附加到编译器错误的信息
	} mjsJoint;


	typedef struct mjsGeom_ {          // 几何体规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjtGeom type;                    // 几何体类型

		// 坐标系与尺寸
		double pos[3];                   // 位置坐标
		double quat[4];                  // 四元数方向
		mjsOrientation alt;              // 替代方向表示
		double fromto[6];                // 端点坐标（用于胶囊体、圆柱体等）
		double size[3];                  // 类型相关尺寸参数

		// 接触属性
		int contype;                     // 接触类型掩码
		int conaffinity;                 // 接触影响掩码
		int condim;                      // 接触维度（1-6）
		int priority;                    // 接触优先级
		double friction[3];              // 滑动、滚动、自旋摩擦系数
		double solmix;                   // 接触对求解器混合参数
		mjtNum solref[mjNREF];           // 求解器参考参数
		mjtNum solimp[mjNIMP];           // 求解器阻抗参数
		double margin;                   // 接触检测边距
		double gap;                      // 参与求解的距离阈值（margin - gap）

		// 惯性推断参数
		double mass;                     // 用于计算密度的质量
		double density;                  // 根据体积/表面积计算质量的密度
		mjtGeomInertia typeinertia;      // 惯性计算类型（体积/表面）

		// 流体力学参数
		mjtNum fluid_ellipsoid;          // 是否启用椭球流体模型
		mjtNum fluid_coefs[5];           // 椭球流体相互作用系数

		// 视觉属性
		mjString* material;              // 材质名称
		float rgba[4];                   // 未指定材质时的RGBA颜色
		int group;                       // 分组标识

		// 其他属性
		mjString* hfieldname;            // 关联的高度场名称
		mjString* meshname;              // 关联的网格名称
		double fitscale;                 // 网格统一缩放比例
		mjDoubleVec* userdata;           // 用户数据
		mjsPlugin plugin;                // SDF插件
		mjString* info;                  // 附加到编译器错误的信息
	} mjsGeom;


	typedef struct mjsSite_ {          // 站点规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 坐标系与尺寸
		double pos[3];                   // 位置坐标
		double quat[4];                  // 四元数方向
		mjsOrientation alt;              // 替代方向表示
		double fromto[6];                // 端点坐标（用于胶囊体等）
		double size[3];                  // 几何尺寸

		// 视觉属性
		mjtGeom type;                    // 几何体类型
		mjString* material;              // 材质名称
		int group;                       // 分组标识
		float rgba[4];                   // 未指定材质时的RGBA颜色

		// 其他属性
		mjDoubleVec* userdata;           // 用户数据
		mjString* info;                  // 附加到编译器错误的信息
	} mjsSite;


	typedef struct mjsCamera_ {        // 相机规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 外参参数
		double pos[3];                   // 位置坐标
		double quat[4];                  // 四元数方向
		mjsOrientation alt;              // 替代方向表示
		mjtCamLight mode;                // 跟踪模式
		mjString* targetbody;            // 跟踪目标体名称

		// 内参参数
		int orthographic;                // 是否正交投影
		double fovy;                     // 垂直视场角（度）
		double ipd;                      // 瞳孔间距
		float intrinsic[4];              // 相机内参（物理单位）
		float sensor_size[2];            // 传感器尺寸（物理单位）
		float resolution[2];             // 分辨率（像素）
		float focal_length[2];           // 焦距（物理单位）
		float focal_pixel[2];            // 焦距（像素）
		float principal_length[2];       // 主点坐标（物理单位）
		float principal_pixel[2];        // 主点坐标（像素）

		// 其他属性
		mjDoubleVec* userdata;           // 用户数据
		mjString* info;                  // 附加到编译器错误的信息
	} mjsCamera;


	typedef struct mjsLight_ {         // 光源规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 坐标系参数
		double pos[3];                   // 位置坐标
		double dir[3];                   // 方向向量
		mjtCamLight mode;                // 跟踪模式
		mjString* targetbody;            // 跟踪目标体名称

		// 光源属性
		mjtByte active;                  // 是否激活
		mjtByte directional;             // 是否为定向光/聚光灯
		mjtByte castshadow;              // 是否投射阴影
		double bulbradius;               // 光源半径（用于软阴影）
		float attenuation[3];            // 衰减系数（二次模型）
		float cutoff;                    // 截止角度
		float exponent;                  // 聚光指数
		float ambient[3];                // 环境光颜色
		float diffuse[3];                // 漫反射颜色
		float specular[3];               // 镜面反射颜色

		// 其他属性
		mjString* info;                  // 附加到编译器错误的信息
	} mjsLight;


	typedef struct mjsFlex_ {          // 柔性体规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 接触属性
		int contype;                     // 接触类型掩码
		int conaffinity;                 // 接触影响掩码
		int condim;                      // 接触维度（1-6）
		int priority;                    // 接触优先级
		double friction[3];              // 滑动、滚动、自旋摩擦系数
		double solmix;                   // 接触对求解器混合参数
		mjtNum solref[mjNREF];           // 求解器参考参数
		mjtNum solimp[mjNIMP];           // 求解器阻抗参数
		double margin;                   // 接触检测边距
		double gap;                      // 参与求解的距离阈值（margin - gap）

		// 其他物理属性
		int dim;                         // 元素维度（2D/3D）
		double radius;                   // 原始元素周围半径
		mjtByte internal;                // 启用内部碰撞
		mjtByte flatskin;                // 使用平面着色渲染表面
		int selfcollide;                 // 自碰撞模式
		int activelayers;                // 3D中激活的层数
		int group;                       // 可视化分组
		double edgestiffness;            // 边刚度
		double edgedamping;              // 边阻尼
		float rgba[4];                   // 未指定材质时的RGBA颜色
		mjString* material;              // 材质名称
		double young;                    // 杨氏模量
		double poisson;                  // 泊松比
		double damping;                  // 瑞利阻尼系数
		double thickness;                // 厚度（仅2D）

		// 网格属性
		mjStringVec* nodebody;           // 节点关联体名称
		mjStringVec* vertbody;           // 顶点关联体名称
		mjDoubleVec* node;               // 节点坐标
		mjDoubleVec* vert;               // 顶点坐标
		mjIntVec* elem;                  // 元素顶点索引
		mjFloatVec* texcoord;            // 纹理坐标
		mjIntVec* facetexcoord;          // 面纹理坐标索引

		// 其他属性
		mjString* info;                  // 附加到编译器错误的信息
	} mjsFlex;


	typedef struct mjsMesh_ {          // 网格规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* content_type;          // 文件内容类型
		mjString* file;                  // 网格文件路径
		double refpos[3];                // 参考位置偏移
		double refquat[4];               // 参考方向偏移
		double scale[3];                 // 网格缩放比例
		mjtMeshInertia inertia;          // 惯性计算类型
		mjtByte smoothnormal;            // 是否平滑法线（排除大角度面）
		int maxhullvert;                 // 凸包最大顶点数
		mjFloatVec* uservert;            // 用户提供的顶点数据
		mjFloatVec* usernormal;          // 用户提供的法线数据
		mjFloatVec* usertexcoord;        // 用户提供的纹理坐标
		mjIntVec* userface;              // 用户提供的面索引
		mjIntVec* userfacetexcoord;      // 用户提供的面纹理坐标索引
		mjsPlugin plugin;                // SDF插件
		mjString* info;                  // 附加到编译器错误的信息
	} mjsMesh;


	typedef struct mjsHField_ {        // 高度场规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* content_type;          // 文件内容类型
		mjString* file;                  // 高度场文件路径
		double size[4];                  // 高度场尺寸（覆盖几何体尺寸）
		int nrow;                        // 行数
		int ncol;                        // 列数
		mjFloatVec* userdata;            // 用户提供的高度数据
		mjString* info;                  // 附加到编译器错误的信息
	} mjsHField;


	typedef struct mjsSkin_ {          // 皮肤规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* file;                  // 皮肤文件路径
		mjString* material;              // 材质名称
		float rgba[4];                   // 未指定材质时的RGBA颜色
		float inflate;                   // 法线方向膨胀量
		int group;                       // 可视化分组

		// 网格数据
		mjFloatVec* vert;                // 顶点坐标
		mjFloatVec* texcoord;            // 纹理坐标
		mjIntVec* face;                  // 面索引

		// 绑定数据
		mjStringVec* bodyname;           // 绑定体名称列表
		mjFloatVec* bindpos;             // 绑定位置
		mjFloatVec* bindquat;            // 绑定方向
		mjIntVecVec* vertid;             // 顶点ID列表（每个顶点关联的体）
		mjFloatVecVec* vertweight;       // 顶点权重列表

		// 其他属性
		mjString* info;                  // 附加到编译器错误的信息
	} mjsSkin;


	typedef struct mjsTexture_ {       // 纹理规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjtTexture type;                 // 纹理类型

		// 方法1：内置纹理
		int builtin;                     // 内置类型（见mjtBuiltin）
		int mark;                        // 标记类型（见mjtMark）
		double rgb1[3];                  // 主颜色
		double rgb2[3];                  // 次颜色
		double markrgb[3];               // 标记颜色
		double random;                   // 随机点生成概率
		int height;                      // 像素高度（立方体/天空盒为正方形）
		int width;                       // 像素宽度
		int nchannel;                    // 通道数

		// 方法2：单文件加载
		mjString* content_type;          // 文件内容类型
		mjString* file;                  // PNG文件路径（立方体各面相同）
		int gridsize[2];                 // 复合纹理网格尺寸（1,1表示重复）
		char gridlayout[13];             // 面布局（行优先：L,R,F,B,U,D，.表示未使用）

		// 方法3：多文件加载（立方体贴图）
		mjStringVec* cubefiles;          // 立方体各面独立文件

		// 方法4：用户数据
		mjByteVec* data;                 // 纹理数据缓冲区

		// 翻转选项
		mjtByte hflip;                   // 水平翻转
		mjtByte vflip;                   // 垂直翻转

		// 其他属性
		mjString* info;                  // 附加到编译器错误的信息
	} mjsTexture;


	typedef struct mjsMaterial_ {      // 材质规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjStringVec* textures;           // 关联的纹理名称列表
		mjtByte texuniform;              // 是否均匀分布立方体贴图
		float texrepeat[2];              // 2D纹理重复次数
		float emission;                  // 自发光强度
		float specular;                  // 镜面反射强度
		float shininess;                 // 高光指数
		float reflectance;               // 反射率
		float metallic;                  // 金属度
		float roughness;                 // 粗糙度
		float rgba[4];                   // 基础颜色
		mjString* info;                  // 附加到编译器错误的信息
	} mjsMaterial;


	typedef struct mjsPair_ {          // 接触对规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* geomname1;             // 几何体1名称
		mjString* geomname2;             // 几何体2名称

		// 可选参数（未设置时从几何体推断）
		int condim;                      // 接触维度
		mjtNum solref[mjNREF];           // 法线方向求解器参考
		mjtNum solreffriction[mjNREF];   // 摩擦方向求解器参考
		mjtNum solimp[mjNIMP];           // 求解器阻抗参数
		double margin;                   // 接触检测边距
		double gap;                      // 参与求解的距离阈值（margin - gap）
		double friction[5];              // 完整摩擦参数
		mjString* info;                  // 附加到错误的信息
	} mjsPair;


	typedef struct mjsExclude_ {       // 排除规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* bodyname1;             // 体1名称
		mjString* bodyname2;             // 体2名称
		mjString* info;                  // 附加到错误的信息
	} mjsExclude;


	typedef struct mjsEquality_ {      // 等式约束规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjtEq type;                      // 约束类型
		double data[mjNEQDATA];          // 类型相关数据
		mjtByte active;                  // 是否初始激活
		mjString* name1;                 // 对象1名称
		mjString* name2;                 // 对象2名称
		mjtObj objtype;                  // 对象类型
		mjtNum solref[mjNREF];           // 求解器参考参数
		mjtNum solimp[mjNIMP];           // 求解器阻抗参数
		mjString* info;                  // 附加到错误的信息
	} mjsEquality;


	typedef struct mjsTendon_ {        // 肌腱规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 刚度、阻尼、摩擦
		double stiffness;                // 刚度系数
		double springlength[2];          // 弹簧自然长度（-1表示使用qpos_spring）
		double damping;                  // 阻尼系数
		double frictionloss;             // 摩擦损耗
		mjtNum solref_friction[mjNREF];  // 摩擦求解器参考参数
		mjtNum solimp_friction[mjNIMP];  // 摩擦求解器阻抗参数

		// 长度限制
		int limited;                     // 是否有限制（见mjtLimited）
		double range[2];                 // 长度范围
		double margin;                   // 限制检测边距
		mjtNum solref_limit[mjNREF];     // 限制求解器参考参数
		mjtNum solimp_limit[mjNIMP];     // 限制求解器阻抗参数

		// 视觉属性
		mjString* material;              // 材质名称
		double width;                    // 渲染宽度
		float rgba[4];                   // 未指定材质时的RGBA颜色
		int group;                       // 分组标识

		// 其他属性
		mjDoubleVec* userdata;           // 用户数据
		mjString* info;                  // 附加到错误的信息
	} mjsTendon;


	typedef struct mjsWrap_ {          // 肌腱包裹对象规格
		mjsElement* element;             // 元素类型
		mjString* info;                  // 附加到错误的信息
	} mjsWrap;


	typedef struct mjsActuator_ {      // 执行器规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 增益与偏置
		mjtGain gaintype;                // 增益类型
		double gainprm[mjNGAIN];         // 增益参数
		mjtBias biastype;                // 偏置类型
		double biasprm[mjNGAIN];         // 偏置参数

		// 激活状态
		mjtDyn dyntype;                  // 动态类型
		double dynprm[mjNDYN];           // 动态参数
		int actdim;                      // 激活变量维度
		mjtByte actearly;                // 是否提前应用激活到qfrc

		// 传动装置
		mjtTrn trntype;                  // 传动类型
		double gear[6];                  // 传动比（长度和力缩放）
		mjString* target;                // 传动目标名称
		mjString* refsite;               // 参考站点（站点传动用）
		mjString* slidersite;            // 滑块站点（曲柄滑块用）
		double cranklength;              // 曲柄长度（曲柄滑块用）
		double lengthrange[2];           // 传动长度范围
		double inheritrange;             // 自动设置位置/速度范围

		// 输入/输出限制
		int ctrllimited;                 // 控制是否受限（见mjtLimited）
		double ctrlrange[2];             // 控制范围
		int forcelimited;                // 力是否受限（见mjtLimited）
		double forcerange[2];            // 力范围
		int actlimited;                  // 激活是否受限（见mjtLimited）
		double actrange[2];              // 激活范围

		// 其他属性
		int group;                       // 分组标识
		mjDoubleVec* userdata;           // 用户数据
		mjsPlugin plugin;                // 执行器插件
		mjString* info;                  // 附加到编译器错误的信息
	} mjsActuator;


	typedef struct mjsSensor_ {        // 传感器规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称

		// 传感器定义
		mjtSensor type;                  // 传感器类型
		mjtObj objtype;                  // 传感对象类型
		mjString* objname;               // 传感对象名称
		mjtObj reftype;                  // 参考对象类型
		mjString* refname;               // 参考对象名称

		// 用户定义传感器
		mjtDataType datatype;            // 数据类型
		mjtStage needstage;              // 所需计算阶段
		int dim;                         // 输出维度

		// 输出后处理
		double cutoff;                   // 截断值（实数和正数类型）
		double noise;                    // 噪声标准差

		// 其他属性
		mjDoubleVec* userdata;           // 用户数据
		mjsPlugin plugin;                // 传感器插件
		mjString* info;                  // 附加到编译器错误的信息
	} mjsSensor;


	typedef struct mjsNumeric_ {       // 自定义数值字段规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjDoubleVec* data;               // 初始化数据
		int size;                        // 数组尺寸（可大于数据尺寸）
		mjString* info;                  // 附加到编译器错误的信息
	} mjsNumeric;


	typedef struct mjsText_ {          // 自定义文本规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjString* data;                  // 文本字符串
		mjString* info;                  // 附加到编译器错误的信息
	} mjsText;


	typedef struct mjsTuple_ {         // 元组规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		mjIntVec* objtype;               // 对象类型列表
		mjStringVec* objname;            // 对象名称列表
		mjDoubleVec* objprm;             // 对象参数列表
		mjString* info;                  // 附加到编译器错误的信息
	} mjsTuple;


	typedef struct mjsKey_ {           // 关键帧规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 名称
		double time;                     // 时间点
		mjDoubleVec* qpos;               // 关节位置
		mjDoubleVec* qvel;               // 关节速度
		mjDoubleVec* act;                // 激活状态
		mjDoubleVec* mpos;               // 运动捕捉位置
		mjDoubleVec* mquat;              // 运动捕捉方向
		mjDoubleVec* ctrl;               // 控制输入
		mjString* info;                  // 附加到编译器错误的信息
	} mjsKey;


	typedef struct mjsDefault_ {       // 默认规格
		mjsElement* element;             // 元素类型
		mjString* name;                  // 类名称
		mjsJoint* joint;                 // 关节默认参数
		mjsGeom* geom;                   // 几何体默认参数
		mjsSite* site;                   // 站点默认参数
		mjsCamera* camera;               // 相机默认参数
		mjsLight* light;                 // 光源默认参数
		mjsFlex* flex;                   // 柔性体默认参数
		mjsMesh* mesh;                   // 网格默认参数
		mjsMaterial* material;           // 材质默认参数
		mjsPair* pair;                   // 接触对默认参数
		mjsEquality* equality;           // 等式约束默认参数
		mjsTendon* tendon;               // 肌腱默认参数
		mjsActuator* actuator;           // 执行器默认参数
	} mjsDefault;

#ifdef __cplusplus
}
#endif

#endif  // MUJOCO_INCLUDE_MJSPEC_H_
