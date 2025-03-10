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

#ifndef MUJOCO_MJEXPORT_H_
#define MUJOCO_MJEXPORT_H_

// Windows平台下的DLL导入/导出宏定义
#if defined _WIN32 || defined __CYGWIN__
#define MUJOCO_HELPER_DLL_IMPORT __declspec(dllimport)
#define MUJOCO_HELPER_DLL_EXPORT __declspec(dllexport)
#define MUJOCO_HELPER_DLL_LOCAL
#else
  // 非Windows平台（通常是类Unix系统）的可见性控制
#if __GNUC__ >= 4
#define MUJOCO_HELPER_DLL_IMPORT __attribute__ ((visibility ("default")))
#define MUJOCO_HELPER_DLL_EXPORT __attribute__ ((visibility ("default")))
#define MUJOCO_HELPER_DLL_LOCAL  __attribute__ ((visibility ("hidden")))
#else
  // 旧版本GCC不支持可见性属性
#define MUJOCO_HELPER_DLL_IMPORT
#define MUJOCO_HELPER_DLL_EXPORT
#define MUJOCO_HELPER_DLL_LOCAL
#endif
#endif

// 静态库配置
#ifdef MJ_STATIC
  // 静态链接时不需要导入/导出属性
#define MJAPI
#define MJLOCAL
#else
  // 动态库配置
#ifdef MUJOCO_DLL_EXPORTS
  // 当编译库本身时使用导出属性
#define MJAPI MUJOCO_HELPER_DLL_EXPORT
#else
  // 当使用库时使用导入属性
#define MJAPI MUJOCO_HELPER_DLL_IMPORT
#endif
// 内部使用的本地符号
#define MJLOCAL MUJOCO_HELPER_DLL_LOCAL
#endif

#endif  // MUJOCO_MJEXPORT_H_
