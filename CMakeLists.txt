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

# 设置CMake最低版本要求
cmake_minimum_required(VERSION 3.16)

# 强制设置CMake策略以控制特定行为（不显示警告）
# CMP0063: 正确处理C语言符号可见性属性
set(CMAKE_POLICY_DEFAULT_CMP0063 NEW)
# CMP0069: 启用时强制进行过程间优化
set(CMAKE_POLICY_DEFAULT_CMP0069 NEW)
# CMP0072: 优先使用GLVND（OpenGL Vendor-Neutral Dispatch）库
set(CMAKE_POLICY_DEFAULT_CMP0072 NEW)
# CMP0077: 防止ccd库覆盖BUILD_SHARED_LIBS选项
set(CMAKE_POLICY_DEFAULT_CMP0077 NEW)

# 在项目定义前设置MSVC增量链接默认开启（影响Visual Studio编译器）
set(MSVC_INCREMENTAL_DEFAULT ON)

# 定义项目基本信息
project(
  mujoco
  VERSION 3.3.1           # 项目版本号
  DESCRIPTION "MuJoCo Physics Simulator"  # 项目描述
  HOMEPAGE_URL "https://mujoco.org"       # 项目主页
)

# 启用C和C++语言支持
enable_language(C)
enable_language(CXX)

# 将项目cmake目录添加到模块搜索路径（用于查找自定义CMake脚本）
list(APPEND CMAKE_MODULE_PATH "${PROJECT_SOURCE_DIR}/cmake")

# 定义项目构建选项
option(MUJOCO_BUILD_EXAMPLES "Build samples for MuJoCo" ON)        # 是否构建示例程序
option(MUJOCO_BUILD_SIMULATE "Build simulate library for MuJoCo" ON)  # 是否构建模拟库
option(MUJOCO_BUILD_TESTS "Build tests for MuJoCo" ON)             # 是否构建测试用例
option(MUJOCO_TEST_PYTHON_UTIL "Build and test utility libraries for Python bindings" ON)  # 是否测试Python工具

# 如果是苹果平台且需要构建示例或模拟库，则启用Objective-C支持
if(APPLE AND (MUJOCO_BUILD_EXAMPLES OR MUJOCO_BUILD_SIMULATE))
  enable_language(OBJC)
  enable_language(OBJCXX)
endif()

# 包含自定义CMake模块
include(MujocoOptions)      # 项目特定编译选项
include(MujocoMacOS)        # macOS平台特定配置
include(MujocoDependencies) # 依赖管理配置

# 定义公共头文件列表
set(MUJOCO_HEADERS
    include/mujoco/mjdata.h
    include/mujoco/mjexport.h
    include/mujoco/mjmacro.h
    include/mujoco/mjmodel.h
    include/mujoco/mjplugin.h
    include/mujoco/mjrender.h
    include/mujoco/mjsan.h
    include/mujoco/mjspec.h
    include/mujoco/mjthread.h
    include/mujoco/mjtnum.h
    include/mujoco/mjui.h
    include/mujoco/mjvisualize.h
    include/mujoco/mjxmacro.h
    include/mujoco/mujoco.h
)

# Windows平台资源文件配置（用于版本信息等）
if(WIN32)
  set(MUJOCO_RESOURCE_FILES ${CMAKE_CURRENT_SOURCE_DIR}/dist/mujoco.rc)
else()
  set(MUJOCO_RESOURCE_FILES "")
endif()

# 创建共享库目标
add_library(mujoco SHARED ${MUJOCO_RESOURCE_FILES})
# 设置头文件包含路径
target_include_directories(
  mujoco
  PUBLIC $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>  # 构建时包含路径
         $<INSTALL_INTERFACE:${CMAKE_INSTALL_INCLUDEDIR}>         # 安装后包含路径
  PRIVATE src  # 私有头文件路径
)

# 添加子目录（按功能模块划分）
add_subdirectory(plugin/elasticity)  # 弹性插件
add_subdirectory(plugin/actuator)    # 执行器插件
add_subdirectory(plugin/sensor)      # 传感器插件
add_subdirectory(plugin/sdf)         # 符号距离场插件
add_subdirectory(src/engine)         # 物理引擎核心
add_subdirectory(src/user)           # 用户接口
add_subdirectory(src/xml)            # XML解析相关
add_subdirectory(src/render)         # 渲染相关
add_subdirectory(src/thread)         # 多线程支持
add_subdirectory(src/ui)             # 用户界面
add_subdirectory(src/inverse)        # 逆动力学相关

# 定义编译器预处理宏
target_compile_definitions(mujoco PRIVATE 
    _GNU_SOURCE             # 启用GNU扩展功能
    CCD_STATIC_DEFINE       # 定义CCD库为静态链接
    MUJOCO_DLL_EXPORTS      # 标记动态库导出符号
    -DMC_IMPLEM_ENABLE      # 启用特定实现细节
)
# 如果启用AVX指令集优化
if(MUJOCO_ENABLE_AVX_INTRINSICS)
  target_compile_definitions(mujoco PUBLIC mjUSEPLATFORMSIMD)
endif()

# 设置编译器选项
target_compile_options(
  mujoco
  PRIVATE ${AVX_COMPILE_OPTIONS}         # AVX指令集编译选项
          ${MUJOCO_MACOS_COMPILE_OPTIONS} # macOS特有编译选项
          ${EXTRA_COMPILE_OPTIONS}        # 额外编译选项
          ${MUJOCO_CXX_FLAGS}             # C++特有编译标志
)

# 设置链接器选项
target_link_options(
  mujoco
  PRIVATE
  ${MUJOCO_MACOS_LINK_OPTIONS}  # macOS特有链接选项
  ${EXTRA_LINK_OPTIONS}          # 额外链接选项
)

# 链接第三方依赖库
target_link_libraries(
  mujoco
  PRIVATE ccd             # 碰撞检测库
          lodepng         # PNG图像处理库
          qhullstatic_r   # 凸包算法库（静态链接）
          tinyobjloader   # 轻量级OBJ加载库
          tinyxml2        # XML解析库
)

# 设置目标属性（版本号和公开头文件）
set_target_properties(
  mujoco PROPERTIES 
  VERSION "${mujoco_VERSION}"     # 动态库版本号
  PUBLIC_HEADER "${MUJOCO_HEADERS}" # 公开头文件列表
)

# macOS Framework特殊处理（模拟框架结构）
if(APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS)
  # 使用Xcode工具链的tapi处理符号
  set(TAPI
      "/Applications/Xcode.app/Contents/Developer/Toolchains/XcodeDefault.xctoolchain/usr/bin/tapi"
  )
  # 生成Info.plist文件
  configure_file(
    ${CMAKE_CURRENT_SOURCE_DIR}/dist/Info.plist.framework.in
    ${CMAKE_CURRENT_SOURCE_DIR}/dist/Info.framework.plist
  )
  # 设置框架输出路径和安装名称
  set_target_properties(
    mujoco
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY
               "${CMAKE_LIBRARY_OUTPUT_DIRECTORY}/mujoco.framework/Versions/A"
               BUILD_WITH_INSTALL_NAME_DIR TRUE
               INSTALL_NAME_DIR "@rpath/mujoco.framework/Versions/A"
  )
  # 自定义构建步骤创建框架结构
  add_custom_command(
    TARGET mujoco
    POST_BUILD
    COMMAND mkdir -p $<TARGET_FILE_DIR:mujoco>/Headers                       # 创建头文件目录
    COMMAND cd ${CMAKE_CURRENT_SOURCE_DIR} && cp ${MUJOCO_HEADERS} $<TARGET_FILE_DIR:mujoco>/Headers  # 复制头文件
    COMMAND mkdir -p $<TARGET_FILE_DIR:mujoco>/Modules                      # 创建模块目录
    COMMAND cp ${CMAKE_CURRENT_SOURCE_DIR}/dist/module.modulemap $<TARGET_FILE_DIR:mujoco>/Modules  # 复制模块映射文件
    COMMAND mkdir -p $<TARGET_FILE_DIR:mujoco>/Resources                     # 创建资源目录
    COMMAND mv ${CMAKE_CURRENT_SOURCE_DIR}/dist/Info.framework.plist         # 移动Info.plist
            $<TARGET_FILE_DIR:mujoco>/Resources/Info.plist
    COMMAND ${TAPI} stubify $<TARGET_FILE:mujoco> -o $<TARGET_FILE_DIR:mujoco>/mujoco.tbd  # 生成符号表
    COMMAND ln -fhs A $<TARGET_FILE_DIR:mujoco>/../Current                   # 创建版本符号链接
    COMMAND ln -fhs Versions/Current/mujoco.tbd $<TARGET_FILE_DIR:mujoco>/../../mujoco.tbd
    COMMAND ln -fhs Versions/Current/Headers $<TARGET_FILE_DIR:mujoco>/../../Headers
    COMMAND ln -fhs Versions/Current/Modules $<TARGET_FILE_DIR:mujoco>/../../Modules
    COMMAND ln -fhs Versions/Current/Resources $<TARGET_FILE_DIR:mujoco>/../../Resources
    COMMAND_EXPAND_LISTS  # 允许命令中的列表扩展
  )
endif()

# 创建命名空间别名（用于示例程序）
add_library(mujoco::mujoco ALIAS mujoco)

# 添加模型子目录
add_subdirectory(model)

# 构建顺序处理：simulate必须在sample之前（因embed_in_bundle宏依赖）
if(MUJOCO_BUILD_SIMULATE)
  add_subdirectory(simulate)  # 模拟库
endif()

if(MUJOCO_BUILD_EXAMPLES)
  add_subdirectory(sample)  # 示例程序
endif()

# 测试相关配置
if(BUILD_TESTING AND MUJOCO_BUILD_TESTS)
  enable_testing()  # 启用测试功能
  add_subdirectory(test)  # 单元测试
  if(MUJOCO_TEST_PYTHON_UTIL)
    add_subdirectory(python/mujoco/util)  # Python工具测试
  endif()
endif()

# 非macOS框架模式下的安装配置
if(NOT (APPLE AND MUJOCO_BUILD_MACOS_FRAMEWORKS))
  # 安装目标文件
  install(
    TARGETS mujoco
    EXPORT ${PROJECT_NAME}  # 导出配置名称
    RUNTIME DESTINATION "${CMAKE_INSTALL_BINDIR}" COMPONENT runtime  # 可执行文件安装路径
    LIBRARY DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT runtime   # 动态库安装路径
    ARCHIVE DESTINATION "${CMAKE_INSTALL_LIBDIR}" COMPONENT dev       # 静态库安装路径
    PUBLIC_HEADER DESTINATION "${CMAKE_INSTALL_INCLUDEDIR}/mujoco" COMPONENT dev  # 头文件安装路径
  )

  # 定义CMake包配置路径
  set(CONFIG_PACKAGE_LOCATION "${CMAKE_INSTALL_LIBDIR}/cmake/${PROJECT_NAME}")

  # 生成目标导出文件
  install(
    EXPORT ${PROJECT_NAME}
    DESTINATION ${CONFIG_PACKAGE_LOCATION}
    NAMESPACE mujoco::  # 命名空间前缀
    FILE "${PROJECT_NAME}Targets.cmake"  # 导出文件名
  )

  # 生成包版本文件
  include(CMakePackageConfigHelpers)
  write_basic_package_version_file(
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
    VERSION ${mujoco_VERSION}
    COMPATIBILITY AnyNewerVersion  # 版本兼容性策略
  )

  # 配置包配置文件
  configure_package_config_file(
    cmake/${PROJECT_NAME}Config.cmake.in 
    "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
    INSTALL_DESTINATION ${CONFIG_PACKAGE_LOCATION}
  )

  # 安装配置文件
  install(FILES "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}Config.cmake"
                "${CMAKE_CURRENT_BINARY_DIR}/${PROJECT_NAME}ConfigVersion.cmake"
          DESTINATION ${CONFIG_PACKAGE_LOCATION}
  )

  # 安装模型文件
  install(
    DIRECTORY model
    DESTINATION "${CMAKE_INSTALL_DATADIR}/mujoco"  # 安装到共享数据目录
    PATTERN "CMakeLists.txt" EXCLUDE  # 排除CMake文件
  )

endif()
