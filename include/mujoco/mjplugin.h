// 版权声明和许可证信息，保留不变
// Copyright 2022 DeepMind Technologies Limited
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

#ifndef MUJOCO_INCLUDE_MJPLUGIN_H_
#define MUJOCO_INCLUDE_MJPLUGIN_H_

#include <mujoco/mjdata.h>
#include <mujoco/mjmodel.h>
#include <mujoco/mjtnum.h>
#include <mujoco/mjvisualize.h>


//---------------------------------- 资源提供器接口 ---------------------------------------------

struct mjResource_ {
    char* name;                      // 资源名称（文件名等）
    void* data;                      // 不透明数据指针
    char timestamp[512];             // 资源时间戳
    const struct mjpResourceProvider* provider;  // 关联的资源提供器
};
typedef struct mjResource_ mjResource;

// 打开资源的回调函数，成功返回非零值
typedef int (*mjfOpenResource)(mjResource* resource);

// 读取资源的回调函数
// 返回缓冲区存储的字节数，错误时返回-1
typedef int (*mjfReadResource)(mjResource* resource, const void** buffer);

// 关闭资源的回调函数（负责释放分配的内存）
typedef void (*mjfCloseResource)(mjResource* resource);

// 获取资源目录的回调函数
// 设置目录字符串和长度
typedef void (*mjfGetResourceDir)(mjResource* resource, const char** dir, int* ndir);

// 检查资源是否修改的回调函数
// 返回0表示时间戳匹配，>0表示资源较新，<0表示资源较旧
typedef int (*mjfResourceModified)(const mjResource* resource, const char* timestamp);

// 资源提供器描述结构体
struct mjpResourceProvider {
    const char* prefix;              // 资源名前缀匹配模式
    mjfOpenResource open;            // 打开回调
    mjfReadResource read;            // 读取回调
    mjfCloseResource close;          // 关闭回调
    mjfGetResourceDir getdir;        // 获取目录回调（可选）
    mjfResourceModified modified;    // 修改检查回调（可选）
    void* data;                      // 不透明数据指针（资源不变性）
};
typedef struct mjpResourceProvider mjpResourceProvider;


//---------------------------------- 插件系统 -------------------------------------------------------

// 插件能力位掩码枚举
typedef enum mjtPluginCapabilityBit_ {
    mjPLUGIN_ACTUATOR = 1 << 0,       // 执行器力计算能力
    mjPLUGIN_SENSOR = 1 << 1,       // 传感器测量能力
    mjPLUGIN_PASSIVE = 1 << 2,       // 被动力计算能力
    mjPLUGIN_SDF = 1 << 3,       // 符号距离场计算能力
} mjtPluginCapabilityBit;

// 插件描述结构体
struct mjpPlugin_ {
    const char* name;               // 全局唯一插件名称标识

    int nattribute;                 // 配置属性数量
    const char* const* attributes;  // 配置属性名称数组

    int capabilityflags;            // 能力标志位（mjtPluginCapabilityBit组合）
    int needstage;                  // 传感器计算阶段需求（mjtStage枚举）

    // 获取插件实例状态所需mjtNum数量（必须实现）
    int (*nstate)(const mjModel* m, int instance);

    // 获取指定传感器的输出维度（仅传感器插件必须实现）
    int (*nsensordata)(const mjModel* m, int instance, int sensor_id);

    // 初始化插件实例（必须实现），成功返回0
    int (*init)(const mjModel* m, mjData* d, int instance);

    // 销毁插件实例（可选实现）
    void (*destroy)(mjData* d, int instance);

    // 复制插件实例数据（可选实现）
    void (*copy)(mjData* dest, const mjModel* m, const mjData* src, int instance);

    // 重置插件状态（必须实现）
    void (*reset)(const mjModel* m, mjtNum* plugin_state, void* plugin_data, int instance);

    // 插件主计算函数（必须实现）
    void (*compute)(const mjModel* m, mjData* d, int instance, int capability_bit);

    // 时间步进函数（可选实现）
    void (*advance)(const mjModel* m, mjData* d, int instance);

    // 可视化处理函数（可选实现）
    void (*visualize)(const mjModel* m, mjData* d, const mjvOption* opt, mjvScene* scn, int instance);

    // ------------------------- 执行器相关方法（可选） -------------------------

    // 更新执行器act_dot值（在原生计算之后，主计算之前调用）
    void (*actuator_act_dot)(const mjModel* m, mjData* d, int instance);

    // ------------------------- SDF相关方法（可选） ---------------------------

    // 计算点到表面的符号距离
    mjtNum(*sdf_distance)(const mjtNum point[3], const mjData* d, int instance);

    // 计算距离场的局部坐标梯度
    void (*sdf_gradient)(mjtNum gradient[3], const mjtNum point[3], const mjData* d, int instance);

    // 静态距离计算（用于网格生成）
    mjtNum(*sdf_staticdistance)(const mjtNum point[3], const mjtNum* attributes);

    // 属性转换与默认值设置
    void (*sdf_attribute)(mjtNum attribute[], const char* name[], const char* value[]);

    // 获取隐式表面的轴对齐包围盒
    void (*sdf_aabb)(mjtNum aabb[6], const mjtNum* attributes);
};
typedef struct mjpPlugin_ mjpPlugin;

// ------------------------- 跨平台插件初始化宏 ---------------------------

#if defined(__has_attribute)

#if __has_attribute(constructor)
  // GCC/Clang的构造函数属性，库加载时自动执行初始化
#define mjPLUGIN_LIB_INIT __attribute__((constructor)) static void _mjplugin_init(void)
#endif  // __has_attribute(constructor)

#elif defined(_MSC_VER)

  // Windows DLL入口点定义
#ifndef mjDLLMAIN
#define mjDLLMAIN DllMain
#endif

// 跨语言链接规范
#if !defined(mjEXTERNC)
#if defined(__cplusplus)
#define mjEXTERNC extern "C"
#else
#define mjEXTERNC
#endif  // defined(__cplusplus)
#endif  // !defined(mjEXTERNC)

// DLL主函数实现，处理加载事件
// NOLINTBEGIN(runtime/int)  // 忽略类型长度警告
#define mjPLUGIN_LIB_INIT                                                                 \
    static void _mjplugin_dllmain(void);                                                    \
    mjEXTERNC int __stdcall mjDLLMAIN(void* hinst, unsigned long reason, void* reserved) {  \
      if (reason == 1) {                                                                    \
        _mjplugin_dllmain();                                                                \
      }                                                                                     \
      return 1;                                                                             \
    }                                                                                       \
    static void _mjplugin_dllmain(void)
  // NOLINTEND(runtime/int)

#endif  // defined(_MSC_VER)

// 插件库加载回调函数类型定义
typedef void (*mjfPluginLibraryLoadCallback)(const char* filename, int first, int count);

#endif  // MUJOCO_INCLUDE_MJPLUGIN_H_
