# CMakeLists.txt for CNC Feature Recognition Example
cmake_minimum_required(VERSION 3.12)
project(CNC_Feature_Recognition_Example)

# 设置C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 查找CGAL
find_package(CGAL REQUIRED COMPONENTS Core)

# 查找Boost
find_package(Boost REQUIRED COMPONENTS graph)

# 如果需要Qt5可视化（可选）
find_package(Qt5 QUIET COMPONENTS Core Widgets OpenGL)

# 添加可执行文件
add_executable(cnc_feature_recognition cnc_feature_recognition_example.cpp)

# 链接库
target_link_libraries(cnc_feature_recognition
    CGAL::CGAL
    CGAL::CGAL_Core
    Boost::graph
)

# 如果找到Qt5，添加可视化示例
if(Qt5_FOUND)
    add_executable(cnc_feature_viewer cnc_feature_viewer.cpp)
    target_link_libraries(cnc_feature_viewer
        CGAL::CGAL
        CGAL::CGAL_Core
        Boost::graph
        Qt5::Core
        Qt5::Widgets
        Qt5::OpenGL
    )
endif()

# 添加编译选项
if(MSVC)
    target_compile_options(cnc_feature_recognition PRIVATE /W3)
else()
    target_compile_options(cnc_feature_recognition PRIVATE -Wall -Wextra -pedantic)
endif()

# 设置输出目录
set_target_properties(cnc_feature_recognition
    PROPERTIES
    RUNTIME_OUTPUT_DIRECTORY "${CMAKE_BINARY_DIR}/bin"
)