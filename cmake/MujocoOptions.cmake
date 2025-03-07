# 版权声明和许可证信息
# Copyright 2021 DeepMind Technologies Limited
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     https://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# ======================== 全局编译设置 ========================
set(CMAKE_C_STANDARD 11)               # 强制使用C11标准
set(CMAKE_C_STANDARD_REQUIRED ON)      # 必须满足指定C标准
set(CMAKE_CXX_STANDARD 17)             # 强制使用C++17标准
set(CMAKE_CXX_STANDARD_REQUIRED ON)    # 必须满足指定C++标准
set(CMAKE_CXX_EXTENSIONS OFF)          # 禁用编译器扩展（如GNU的-std=gnu++17）
set(CMAKE_C_EXTENSIONS OFF)            # 禁用C语言扩展
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)  # 生成compile_commands.json（用于LLVM工具链）

# ======================== 构建类型配置 ========================
if(NOT CMAKE_CONFIGURATION_TYPES)
  # 如果没有指定构建类型，默认设为Release
  # 现在修改为Debug
  if(NOT CMAKE_BUILD_TYPE)
    message(STATUS "Setting build type to 'Release' as none was specified.")
    set(CMAKE_BUILD_TYPE
        "Debug"
        CACHE STRING "Choose the type of build, recommanded options are: Debug or Release" FORCE
    )
  endif()
  # 定义允许的构建类型
  set(BUILD_TYPES "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS ${BUILD_TYPES})
endif()

# ======================== 安装路径配置 ========================
include(GNUInstallDirs)  # 引入GNU标准安装目录定义

# 修改默认输出目录（便于Windows下DLL与EXE同目录）
set(CMAKE_RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_BINDIR}")
set(CMAKE_LIBRARY_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")
set(CMAKE_ARCHIVE_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/${CMAKE_INSTALL_LIBDIR}")

# ======================== 编译器特性配置 ========================
set(OpenGL_GL_PREFERENCE GLVND)        # 优先使用GLVND OpenGL分发
set(CMAKE_POSITION_INDEPENDENT_CODE ON) # 启用位置无关代码（PIC）
# set(CMAKE_C_VISIBILITY_PRESET hidden)   # 隐藏C符号可见性
# set(CMAKE_CXX_VISIBILITY_PRESET hidden) # 隐藏C++符号可见性
# set(CMAKE_VISIBILITY_INLINES_HIDDEN ON) # 隐藏内联函数符号

# ======================== 平台特定编译选项 ========================
if(MSVC)
  # MSVC编译器选项
  add_compile_options(/Gy       # 启用函数级链接
                      /Gw       # 全局数据优化
                      /Od       # 强制禁用优化，避免函数内联导致无法单步调试
                      /Zi       # 生成调试信息
                      /DEBUG    # 生成可调试的二进制文件
                      /FS       # 解决PDB写入冲突
  )
  # 设置PDB输出路径
  set(CMAKE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
  set(CMAKE_COMPILE_PDB_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/bin)
elseif(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  # GCC/Clang编译器选项
  add_compile_options(-fdata-sections  # 将数据放入独立段
                      -ffunction-sections  # 将函数放入独立段
  )
endif()

# ======================== 动态库默认配置 ========================
set(BUILD_SHARED_LIBS
    ON
    CACHE BOOL "Build Mujoco as shared library."  # 默认构建共享库
)

# ======================== 项目选项配置 ========================
option(MUJOCO_ENABLE_AVX "Build binaries that require AVX instructions, if possible." ON)  # 启用AVX指令集支持
option(MUJOCO_ENABLE_AVX_INTRINSICS "Make use of hand-written AVX intrinsics, if possible." ON)  # 启用手写AVX内联
option(MUJOCO_ENABLE_RPATH "Enable RPath support when installing Mujoco." ON)  # 启用安装时的RPATH支持
mark_as_advanced(MUJOCO_ENABLE_RPATH)  # 标记为高级选项（默认不显示）

# ======================== AVX支持检测 ========================
if(MUJOCO_ENABLE_AVX)
  include(CheckAvxSupport)            # 包含AVX支持检测模块
  get_avx_compile_options(AVX_COMPILE_OPTIONS)  # 获取AVX编译选项
else()
  set(AVX_COMPILE_OPTIONS)  # 清空AVX选项
endif()

# ======================== macOS框架选项 ========================
option(MUJOCO_BUILD_MACOS_FRAMEWORKS "Build libraries as macOS Frameworks" OFF)  # macOS框架构建开关

# ======================== 链接选项配置 ========================
include(MujocoLinkOptions)  # 包含链接选项模块
get_mujoco_extra_link_options(EXTRA_LINK_OPTIONS)  # 获取额外链接选项

# ======================== 严格编译检查选项 ========================
if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU" OR (CMAKE_CXX_COMPILER_ID MATCHES "Clang" AND NOT MSVC))
  set(EXTRA_COMPILE_OPTIONS
      # -Werror             # 将警告视为错误（严重影响调试）
      -Wall               # 启用所有警告
      -Wpedantic          # 严格遵循标准警告
      -Wimplicit-fallthrough  # 检查switch语句fallthrough
      -Wunused            # 未使用变量/函数警告
      -Wvla               # 变长数组警告
      -Wno-int-in-bool-context  # 禁用整型在布尔上下文中的警告
      -Wno-sign-compare    # 禁用有符号/无符号比较警告
      -Wno-unknown-pragmas # 禁用未知pragma警告
  )
  # 在Debug模式下，不将警告视为错误
  if(NOT CMAKE_BUILD_TYPE STREQUAL "Debug")
    list(APPEND EXTRA_COMPILE_OPTIONS -Werror)
  endif()
  if(CMAKE_CXX_COMPILER_ID STREQUAL "GNU")
    set(EXTRA_COMPILE_OPTIONS ${EXTRA_COMPILE_OPTIONS}
        -Wimplicit-fallthrough=5  # 严格要求fallthrough注释
        -Wno-maybe-uninitialized  # 禁用可能未初始化警告
    )
  endif()
endif()

# ======================== 过程间优化配置 ========================
if(NOT CMAKE_INTERPROCEDURAL_OPTIMIZATION AND (CMAKE_BUILD_TYPE AND NOT CMAKE_BUILD_TYPE STREQUAL "Debug"))
  set(CMAKE_INTERPROCEDURAL_OPTIMIZATION ON)  # 非Debug构建启用LTO（影响调试）
endif()

# ======================== 安全加固选项 ========================
include(MujocoHarden)  # 包含安全加固模块
set(EXTRA_COMPILE_OPTIONS ${EXTRA_COMPILE_OPTIONS} ${MUJOCO_HARDEN_COMPILE_OPTIONS})
set(EXTRA_LINK_OPTIONS ${EXTRA_LINK_OPTIONS} ${MUJOCO_HARDEN_LINK_OPTIONS})

# ======================== Windows特定定义 ========================
if(WIN32)
  add_definitions(-D_CRT_SECURE_NO_WARNINGS  # 禁用CRT安全警告
                  -D_CRT_SECURE_NO_DEPRECATE  # 禁用CRT弃用警告
  )
endif()
# ======================== Debug模式定义补正 ========================
if(CMAKE_BUILD_TYPE STREQUAL "Debug")
  add_compile_options(-g3 -ggdb)
endif()

