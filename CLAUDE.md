# CLAUDE.md v2.0

这个文件为在此仓库中工作的Claude Code (claude.ai/code)提供指导。

## 项目概述

CGAL (Computational Geometry Algorithms Library) 是一个C++库，提供计算几何中高效可靠的算法。自版本5.0起，CGAL是一个仅头文件的库，无需预先构建库文件。CGAL库包含154个专门的包，每个包针对特定的计算几何问题提供解决方案。

## 代码库架构

### 包结构概览

CGAL采用模块化的包(package)结构，每个包对应一个数据结构或算法。每个包的典型结构：

```
PackageName/
├── include/           # 头文件
├── examples/          # 示例代码
├── test/             # 单元测试
├── benchmark/        # 性能测试
├── demo/             # 演示程序
└── doc/              # 文档
```

### 包分类详解（154个包）

#### 1. 几何内核包（Geometry Kernels）

几何内核是CGAL的基础，定义了基本的几何对象和谓词。

- **Kernel_23** - 2D和3D几何内核的统一接口，是CGAL最核心的包
- **Cartesian_kernel** - 基于笛卡尔坐标系的几何内核，适用于大多数应用
- **Homogeneous_kernel** - 基于齐次坐标的几何内核，提供精确的有理数运算
- **Filtered_kernel** - 带浮点数过滤的内核，结合速度和精确性
- **Kernel_d** - 高维几何内核，支持任意维度的几何计算
- **NewKernel_d** - 新一代高维内核，性能更优
- **Circular_kernel_2** - 2D圆弧几何内核，处理圆和圆弧
- **Circular_kernel_3** - 3D球面几何内核
- **Algebraic_kernel_for_circles** - 圆的代数内核
- **Algebraic_kernel_for_spheres** - 球的代数内核

#### 2. 三角剖分包（Triangulation）

三角剖分是计算几何的核心数据结构。

##### 2D三角剖分
- **Triangulation_2** - 2D Delaunay三角剖分，最基础的三角剖分包
- **TDS_2** - 2D三角剖分数据结构
- **Periodic_2_triangulation_2** - 2D周期性三角剖分，处理周期性边界条件
- **Hyperbolic_triangulation_2** - 双曲平面上的三角剖分
- **Periodic_4_hyperbolic_triangulation_2** - 周期性双曲三角剖分
- **Triangulation_on_sphere_2** - 球面上的三角剖分
- **Triangulation_on_hyperbolic_surface_2** - 双曲曲面上的三角剖分

##### 3D三角剖分
- **Triangulation_3** - 3D Delaunay三角剖分
- **TDS_3** - 3D三角剖分数据结构
- **Constrained_triangulation_3** - 带约束的3D三角剖分
- **Periodic_3_triangulation_3** - 3D周期性三角剖分
- **SMDS_3** - 3D单纯形网格数据结构

##### 通用三角剖分
- **Triangulation** - 三角剖分的通用接口和工具

#### 3. 网格生成与处理（Mesh Generation and Processing）

##### 网格生成
- **Mesh_2** - 2D网格生成，支持约束和质量准则
- **Mesh_3** - 3D体网格生成，从隐式函数或表面生成四面体网格
- **Periodic_3_mesh_3** - 周期性3D网格生成
- **Surface_mesher** - 表面网格生成器
- **Mesher_level** - 网格生成的层次化框架

##### 网格处理
- **Polygon_mesh_processing** - 多边形网格处理算法集合（修复、简化、细分等）
- **Surface_mesh** - 半边数据结构的表面网格表示
- **Surface_mesh_simplification** - 表面网格简化算法
- **Surface_mesh_approximation** - 表面网格逼近
- **Surface_mesh_parameterization** - 表面网格参数化（UV展开）
- **Surface_mesh_segmentation** - 表面网格分割
- **Surface_mesh_deformation** - 表面网格变形
- **Surface_mesh_shortest_path** - 表面网格最短路径计算
- **Surface_mesh_skeletonization** - 表面网格骨架提取
- **Surface_mesh_topology** - 表面网格拓扑操作
- **Tetrahedral_remeshing** - 四面体网格重新划分
- **Subdivision_method_3** - 细分曲面方法

#### 4. 凸包算法（Convex Hull）

- **Convex_hull_2** - 2D凸包算法（Graham扫描、Jarvis步进等）
- **Convex_hull_3** - 3D凸包算法（快速凸包算法）
- **Convex_hull_d** - 高维凸包算法

#### 5. Alpha形状（Alpha Shapes）

- **Alpha_shapes_2** - 2D Alpha形状，用于点云形状重建
- **Alpha_shapes_3** - 3D Alpha形状
- **Alpha_wrap_3** - 3D Alpha包裹，生成水密网格

#### 6. Voronoi图与Delaunay图（Voronoi and Delaunay Diagrams）

- **Voronoi_diagram_2** - 2D Voronoi图的通用框架
- **Segment_Delaunay_graph_2** - 线段的Delaunay图
- **Segment_Delaunay_graph_Linf_2** - L∞度量下的线段Delaunay图
- **Apollonius_graph_2** - 加权点的Voronoi图（Apollonius图）

#### 7. 布尔运算（Boolean Operations）

- **Boolean_set_operations_2** - 2D多边形的布尔运算（并、交、差）
- **Nef_2** - 2D Nef多面体，支持精确的布尔运算
- **Nef_3** - 3D Nef多面体
- **Nef_S2** - 球面上的Nef多面体

#### 8. Minkowski和（Minkowski Sum）

- **Minkowski_sum_2** - 2D Minkowski和计算
- **Minkowski_sum_3** - 3D Minkowski和计算

#### 9. 多边形与多面体（Polygons and Polyhedra）

- **Polygon** - 2D多边形的表示和操作
- **Polygon_repair** - 多边形修复算法
- **Polyhedron** - 3D多面体的半边数据结构
- **HalfedgeDS** - 半边数据结构框架

#### 10. 排列与包络线（Arrangements and Envelopes）

- **Arrangement_on_surface_2** - 2D曲线排列，处理曲线的交点和拓扑
- **Envelope_2** - 2D包络线计算
- **Envelope_3** - 3D包络面计算
- **Surface_sweep_2** - 2D扫描线算法框架

#### 11. 形状检测与重建（Shape Detection and Reconstruction）

##### 形状检测
- **Shape_detection** - 基本形状检测（平面、圆柱、球等）
- **Shape_regularization** - 形状正则化

##### 表面重建
- **Advancing_front_surface_reconstruction** - 前沿推进表面重建
- **Poisson_surface_reconstruction_3** - 泊松表面重建
- **Scale_space_reconstruction_3** - 尺度空间表面重建
- **Polygonal_surface_reconstruction** - 多边形表面重建
- **Kinetic_surface_reconstruction** - 动态表面重建
- **Kinetic_space_partition** - 动态空间分割

##### 点云处理
- **Point_set_3** - 3D点集数据结构和算法
- **Point_set_2** - 2D点集处理
- **Point_set_processing_3** - 3D点云处理算法（滤波、法向估计、特征提取等）
- **Jet_fitting_3** - 喷射拟合，用于曲率估计

#### 12. 空间搜索与数据结构（Spatial Searching and Data Structures）

- **Spatial_searching** - 空间搜索算法（kd-tree、范围搜索等）
- **SearchStructures** - 搜索结构的通用框架
- **AABB_tree** - 轴对齐包围盒树，用于快速碰撞检测
- **Orthtree** - 正交树（octree/quadtree）
- **Box_intersection_d** - 高维盒子相交检测
- **Interval_skip_list** - 区间跳表数据结构

#### 13. 优化算法（Optimization）

- **QP_solver** - 二次规划求解器
- **Optimisation_basic** - 基础优化算法
- **Optimal_bounding_box** - 最优包围盒计算
- **Optimal_transportation_reconstruction_2** - 最优传输重建
- **Solver_interface** - 求解器接口（LAPACK、Eigen等）

#### 14. 距离计算（Distance Computation）

- **Distance_2** - 2D距离计算
- **Distance_3** - 3D距离计算
- **Frechet_distance** - Fréchet距离计算
- **Polytope_distance_d** - 高维多面体距离

#### 15. 分解与分割（Decomposition and Partitioning）

- **Convex_decomposition_3** - 3D凸分解
- **Partition_2** - 2D多边形分割
- **Visibility_2** - 2D可见性计算

#### 16. 插值与拟合（Interpolation and Fitting）

- **Interpolation** - 散点数据插值
- **Barycentric_coordinates_2** - 2D重心坐标
- **Ridges_3** - 3D脊线和谷线提取
- **Principal_component_analysis** - 主成分分析
- **Principal_component_analysis_LGPL** - LGPL版本的PCA

#### 17. 代数基础（Algebraic Foundations）

- **Algebraic_foundations** - 代数基础框架
- **Algebraic_kernel_d** - 高维代数内核
- **Arithmetic_kernel** - 算术内核
- **Number_types** - 数值类型（有理数、区间算术等）
- **Polynomial** - 多项式操作
- **Modular_arithmetic** - 模运算

#### 18. 组合结构（Combinatorial Structures）

- **Combinatorial_map** - 组合映射
- **Generalized_map** - 广义映射
- **Linear_cell_complex** - 线性单元复形

#### 19. 流处理（Stream Processing）

- **Stream_lines_2** - 2D流线生成
- **Stream_support** - 流支持工具
- **Heat_method_3** - 3D热方法（测地距离计算）

#### 20. 几何处理工具（Geometry Processing Utilities）

- **Generator** - 几何对象生成器（随机点、网格等）
- **Circulator** - 循环器，用于遍历循环结构
- **Modifier** - 修改器模式实现
- **Property_map** - 属性映射框架
- **Union_find** - 并查集数据结构
- **Weights** - 权重计算（重心坐标、均值坐标等）

#### 21. 包围体（Bounding Volumes）

- **Bounding_volumes** - 包围体计算（最小包围球、包围盒等）
- **Inscribed_areas** - 内接区域计算

#### 22. 交点计算（Intersections）

- **Intersections_2** - 2D几何对象相交
- **Intersections_3** - 3D几何对象相交

#### 23. 简化算法（Simplification）

- **Polyline_simplification_2** - 2D折线简化
- **Snap_rounding_2** - 2D捕捉舍入

#### 24. 特殊算法（Specialized Algorithms）

- **Isosurfacing_3** - 3D等值面提取（Marching Cubes等）
- **Skin_surface_3** - 分子皮肤表面
- **Straight_skeleton_2** - 2D直骨架
- **Straight_skeleton_extrusion_2** - 直骨架挤出
- **Matrix_search** - 矩阵搜索算法
- **Cone_spanners_2** - 2D锥形扳手图
- **Set_movable_separability_2** - 2D可移动可分离性

#### 25. 机器学习与分类（Machine Learning and Classification）

- **Classification** - 点云分类框架（支持随机森林等）

#### 26. 图形界面与可视化（Graphics and Visualization）

- **Basic_viewer** - 基础3D查看器
- **GraphicsView** - Qt图形视图框架集成
- **Three** - 3D演示框架
- **CGAL_ipelets** - IPE绘图软件插件

#### 27. 支持库（Support Libraries）

- **BGL** - Boost图库接口
- **STL_Extension** - STL扩展
- **Hash_map** - 哈希映射实现
- **Interval_support** - 区间算术支持
- **Random_numbers** - 随机数生成器
- **Profiling_tools** - 性能分析工具
- **CGAL_Core** - 核心数值库
- **CGAL_ImageIO** - 图像输入输出
- **LEDA** - LEDA库接口
- **Miscellany** - 杂项工具
- **Spatial_sorting** - 空间排序算法

#### 28. 基础设施包（Infrastructure）

- **Installation** - 安装和配置
- **Documentation** - 文档系统
- **Scripts** - 构建和开发脚本
- **Testsuite** - 测试套件框架
- **Maintenance** - 维护工具
- **Lab** - 实验性功能
- **Data** - 测试数据
- **LICENSES** - 许可证文件

## 常用命令

### 构建示例或演示

由于CGAL是仅头文件库，构建过程相对简单：

```bash
# 构建特定包的示例（例如：Triangulation_2）
cd Triangulation_2/examples/Triangulation_2
mkdir -p build/debug
cd build/debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCGAL_DIR=/path/to/cgal.git ../..
make
```

### 创建新程序的CMakeLists.txt

使用CGAL提供的脚本创建基础构建配置：

```bash
# 在你的程序目录中运行
/path/to/cgal.git/Scripts/scripts/cgal_create_cmake_script
mkdir -p build/debug
cd build/debug
cmake -DCMAKE_BUILD_TYPE=Debug -DCGAL_DIR=/path/to/cgal.git ../..
make your_program
```

### 测试

启用测试需要设置`CGAL_ENABLE_TESTING`：

```bash
cmake -DCGAL_ENABLE_TESTING=ON -DCMAKE_BUILD_TYPE=Debug .
make
ctest
```

## 开发指南

### 依赖关系

虽然CGAL是仅头文件库，但某些部分需要链接到外部库：

#### 必需依赖
- **GMP** - GNU多精度算术库，提供任意精度整数运算
- **MPFR** - 多精度浮点数库，提供任意精度浮点运算

#### 可选依赖
- **Boost** - 用于高级数据结构和算法
- **Qt** - 用于图形界面和可视化
- **Eigen** - 线性代数库
- **LAPACK/BLAS** - 数值线性代数
- **Intel TBB** - 并行计算支持
- **METIS/SCOTCH** - 图分割库（用于网格分割）
- **CGAL_ImageIO** - 支持各种3D图像格式

### 包开发最佳实践

创建新包或修改现有包时：

1. **结构规范**
   - 遵循标准包目录结构
   - 在`include/CGAL/`下组织头文件
   - 提供清晰的命名空间划分

2. **代码质量**
   - 使用CGAL的断言宏（`CGAL_assertion`等）
   - 实现完整的错误处理
   - 提供详细的代码注释

3. **测试覆盖**
   - 在`test/`目录添加单元测试
   - 在`examples/`提供使用示例
   - 在`benchmark/`添加性能测试

4. **文档要求**
   - 使用Doxygen格式编写API文档
   - 提供用户手册和理论背景
   - 包含代码示例和最佳实践

### 性能优化技巧

1. **使用合适的内核**
   - 浮点计算使用`Exact_predicates_inexact_constructions_kernel`
   - 精确计算使用`Exact_predicates_exact_constructions_kernel`
   - 高性能应用考虑`Filtered_kernel`

2. **数据结构选择**
   - 大规模点云使用`Point_set_3`
   - 动态更新使用`Delaunay_triangulation`的增量算法
   - 空间查询使用`AABB_tree`或`Spatial_searching`

3. **并行化**
   - 使用TBB进行并行计算
   - 利用CGAL的并行算法版本
   - 注意线程安全性

### 调试技巧

1. **使用CGAL断言**
   ```cpp
   #define CGAL_DEBUG  // 启用调试模式
   CGAL_assertion(condition);
   CGAL_precondition(condition);
   CGAL_postcondition(condition);
   ```

2. **可视化调试**
   - 使用`Basic_viewer`快速可视化
   - 导出VTK/PLY格式在ParaView中查看
   - 使用Qt演示程序进行交互式调试

3. **性能分析**
   - 使用`Profiling_tools`包
   - 集成外部性能分析器（Valgrind、Intel VTune等）

### 文档构建

文档使用Doxygen生成：

```bash
cd Documentation/doc
make doc        # 生成HTML文档
make doc_pdf    # 生成PDF文档
```

## 关键脚本

- `Scripts/scripts/cgal_create_cmake_script` - 为新程序创建CMakeLists.txt
- `Scripts/scripts/cgal_create_CMakeLists` - 创建更复杂的构建配置
- `Scripts/developer_scripts/cgal_build` - 开发者构建脚本
- `Scripts/developer_scripts/create_new_package` - 创建新包的模板
- `Scripts/developer_scripts/test_package` - 测试单个包

## 项目特点

- **仅头文件库**: 从5.0版本开始无需预编译，简化部署
- **模板化设计**: 大量使用C++模板实现泛型算法，支持多种数值类型
- **精确数值计算**: 集成GMP/MPFR支持任意精度算术，保证几何算法的鲁棒性
- **严格测试**: 每个包都有完整的测试套件，确保代码质量
- **跨平台**: 支持Windows、Linux、macOS，以及各种编译器
- **模块化架构**: 154个独立包，可按需使用，减少依赖
- **活跃社区**: 持续更新，广泛应用于学术界和工业界
- **丰富文档**: 完整的API文档、用户手册和理论背景

## 版本信息

- 当前文档版本：v2.0
- 适用CGAL版本：5.0及以上
- 包数量：154个
- 最后更新：2025年

## 相关资源

- [CGAL官方网站](https://www.cgal.org/)
- [最新文档](https://doc.cgal.org/)
- [GitHub仓库](https://github.com/CGAL/cgal)
- [开发指南](https://github.com/CGAL/cgal/wiki/Guidelines)
- [论坛和邮件列表](https://www.cgal.org/mailing_list.html)
- [Stack Overflow标签](https://stackoverflow.com/questions/tagged/cgal)