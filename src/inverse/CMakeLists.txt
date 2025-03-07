  
# 定义测试可执行文件
add_executable(inverse_test inverse_test.cpp)

# 链接主项目库（假设主项目库名为 mujoco）
target_link_libraries(inverse_test PUBLIC mujoco)

# 包含必要的头文件路径（根据项目实际路径调整）
target_include_directories(inverse_test PUBLIC
    ${CMAKE_SOURCE_DIR}/include    # 其他可能的头文件路径
    ${CMAKE_SOURCE_DIR}/src
)

# 可选：设置可执行文件输出目录（与项目其他输出一致）
set_target_properties(inverse_test PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY ${CMAKE_SOURCE_DIR}/out/build/x64-Debug/bin
)