# CGAL Algebraic_kernel_for_circles 技术文档

**版本：v1.0**  
**更新日期：2025年1月**

## 目录

1. [包概述](#1-包概述)
2. [架构设计](#2-架构设计)
3. [核心组件](#3-核心组件)
4. [API参考](#4-api参考)
5. [使用指南](#5-使用指南)
6. [应用场景](#6-应用场景)
7. [依赖关系](#7-依赖关系)
8. [性能考虑](#8-性能考虑)
9. [最佳实践](#9-最佳实践)
10. [版本历史](#10-版本历史)

---

## 1. 包概述

### 1.1 简介

`Algebraic_kernel_for_circles` 是CGAL（Computational Geometry Algorithms Library）中专门用于处理圆和圆弧代数运算的核心包。该包提供了一套完整的代数内核，用于精确计算圆与圆、圆与直线之间的交点，以及处理相关的代数根。

### 1.2 设计目标

- **精确计算**：提供圆相关几何运算的精确代数解
- **效率优化**：针对圆的特殊性质优化计算性能
- **通用接口**：与CGAL其他几何内核保持一致的接口设计
- **可扩展性**：支持不同精度的数值类型

### 1.3 主要功能

1. **圆的代数表示**：使用多项式形式 `(X-a)² + (Y-b)² - R²` 表示圆
2. **交点计算**：精确计算圆与圆、圆与直线的交点
3. **根的处理**：管理和比较代数根
4. **临界点分析**：计算圆的X/Y方向临界点
5. **符号判定**：在给定点评估多项式的符号

### 1.4 应用领域

- 圆弧排列（Circular Arrangements）
- 曲线和表面的计算几何
- CAD/CAM系统中的圆弧处理
- 机器人路径规划中的圆弧轨迹
- 计算机图形学中的精确圆渲染

---

## 2. 架构设计

### 2.1 整体架构

```
Algebraic_kernel_for_circles_2_2
    ├── 多项式表示层
    │   ├── Polynomial_for_circles_2_2 (圆多项式)
    │   └── Polynomial_1_2 (直线多项式)
    ├── 代数根层
    │   ├── Root_for_circles_2_2 (圆的根)
    │   └── Root_of_2 (二次根)
    └── 函数对象层
        ├── Solve (求解器)
        ├── Sign_at (符号判定)
        ├── Compare_x/y/xy (比较器)
        └── X/Y_critical_points (临界点)
```

### 2.2 设计模式

#### 2.2.1 模板元编程
包采用C++模板技术实现类型参数化，支持不同的数值类型：

```cpp
template< class RT_ >
struct Algebraic_kernel_for_circles_2_2 {
    typedef RT_ RT;  // Ring Type (环类型)
    typedef typename Root_of_traits< RT >::RootOf_1 FT;  // Field Type
    // ...
};
```

#### 2.2.2 函数对象模式
所有算法操作都封装为函数对象，提供统一的调用接口：

```cpp
typedef AlgebraicFunctors::Solve<Self> Solve;
typedef AlgebraicFunctors::Sign_at<Self> Sign_at;
```

#### 2.2.3 特征类（Traits）模式
通过特征类抽象数值类型的具体实现：

```cpp
typedef typename Root_of_traits< RT >::RootOf_1 FT;
typedef typename Root_of_traits< RT >::RootOf_2 Root_of_2;
```

### 2.3 数据流

1. **输入阶段**：接收圆心坐标和半径平方
2. **构造阶段**：创建多项式表示
3. **求解阶段**：计算交点或临界点
4. **输出阶段**：返回代数根表示的结果

---

## 3. 核心组件

### 3.1 Polynomial_for_circles_2_2

#### 3.1.1 功能描述
表示二维平面中圆的代数多项式，形式为 `(X-a)² + (Y-b)² - R²`。

#### 3.1.2 类定义

```cpp
template < typename FT_ >
class Polynomial_for_circles_2_2 {
    FT_ rep[3]; // 存储 a, b, R²
public:
    typedef FT_ FT;
    
    // 构造函数
    Polynomial_for_circles_2_2(const FT & a, const FT & b, const FT & rsq);
    
    // 访问器
    const FT & a() const;     // 圆心x坐标
    const FT & b() const;     // 圆心y坐标
    const FT & r_sq() const;   // 半径平方
};
```

#### 3.1.3 设计理由
- 使用半径平方避免开方运算，保持精确性
- 紧凑的数组存储，提高缓存局部性
- 简单的接口设计，便于使用

### 3.2 Polynomial_1_2

#### 3.2.1 功能描述
表示二维平面中直线的代数多项式，形式为 `ax + by + c = 0`。

#### 3.2.2 类定义

```cpp
template < typename RT_ >
class Polynomial_1_2 {
    RT_ rep[3]; // 存储 a, b, c
public:
    typedef RT_ RT;
    
    // 构造函数
    Polynomial_1_2(const RT & a, const RT & b, const RT & c);
    
    // 访问器
    const RT & a() const;  // x系数
    const RT & b() const;  // y系数
    const RT & c() const;  // 常数项
};
```

### 3.3 Root_for_circles_2_2

#### 3.3.1 功能描述
表示圆相关计算中产生的二维代数根点。

#### 3.3.2 类定义

```cpp
template < typename RT_ >
class Root_for_circles_2_2 {
    typedef typename Root_of_traits< RT_ >::RootOf_2 Root_of_2;
    
    Handle_for<Root_of_2> x_;  // x坐标的代数根
    Handle_for<Root_of_2> y_;  // y坐标的代数根
    
public:
    // 构造函数
    Root_for_circles_2_2(const Root_of_2& r1, const Root_of_2& r2);
    
    // 访问器
    const Root_of_2& x() const;
    const Root_of_2& y() const;
    
    // 边界框计算
    CGAL::Bbox_2 bbox() const;
};
```

#### 3.3.3 设计特点
- 使用`Handle_for`实现引用计数和内存管理
- 支持精确的代数根表示
- 提供区间算术的边界框计算

### 3.4 Solve 函数对象

#### 3.4.1 功能描述
求解两个几何对象（圆或直线）的交点。

#### 3.4.2 接口定义

```cpp
template < class AK >
class Solve {
public:
    // 圆-圆求交
    template < class OutputIterator >
    OutputIterator operator()(
        const Polynomial_for_circles_2_2 & c1,
        const Polynomial_for_circles_2_2 & c2,
        OutputIterator res) const;
    
    // 直线-圆求交
    template < class OutputIterator >
    OutputIterator operator()(
        const Polynomial_1_2 & line,
        const Polynomial_for_circles_2_2 & circle,
        OutputIterator res) const;
    
    // 直线-直线求交
    template < class OutputIterator >
    OutputIterator operator()(
        const Polynomial_1_2 & l1,
        const Polynomial_1_2 & l2,
        OutputIterator res) const;
};
```

#### 3.4.3 算法实现

##### 圆-圆求交算法
1. 计算两圆心距离的平方
2. 计算判别式以确定交点数量
3. 根据判别式符号：
   - 负数：无交点
   - 零：一个重根（相切）
   - 正数：两个不同交点
4. 使用代数方法精确计算交点坐标

##### 复杂度分析
- 时间复杂度：O(1)
- 空间复杂度：O(1)

### 3.5 Sign_at 函数对象

#### 3.5.1 功能描述
计算多项式在给定点的符号。

#### 3.5.2 接口定义

```cpp
template < class AK >
class Sign_at {
public:
    CGAL::Sign operator()(
        const Polynomial_for_circles_2_2 & equation,
        const Root_for_circles_2_2 & point) const;
    
    CGAL::Sign operator()(
        const Polynomial_1_2 & equation,
        const Root_for_circles_2_2 & point) const;
};
```

### 3.6 临界点计算器

#### 3.6.1 X_critical_points

```cpp
template < class AK >
class X_critical_points {
public:
    // 获取指定的临界点（最左或最右）
    Root_for_circles_2_2 operator()(
        const Polynomial_for_circles_2_2 & c,
        bool i) const;  // i=false: 最左点, i=true: 最右点
    
    // 获取所有X方向临界点
    template <class OutputIterator>
    OutputIterator operator()(
        const Polynomial_for_circles_2_2 & c,
        OutputIterator res) const;
};
```

#### 3.6.2 Y_critical_points

```cpp
template < class AK >
class Y_critical_points {
public:
    // 获取指定的临界点（最下或最上）
    Root_for_circles_2_2 operator()(
        const Polynomial_for_circles_2_2 & c,
        bool i) const;  // i=false: 最下点, i=true: 最上点
    
    // 获取所有Y方向临界点
    template <class OutputIterator>
    OutputIterator operator()(
        const Polynomial_for_circles_2_2 & c,
        OutputIterator res) const;
};
```

### 3.7 比较器

#### 3.7.1 Compare_x

```cpp
template < class AK >
class Compare_x {
public:
    CGAL::Comparison_result operator()(
        const Root_for_circles_2_2 & r1,
        const Root_for_circles_2_2 & r2) const;
};
```

#### 3.7.2 Compare_y

```cpp
template < class AK >
class Compare_y {
public:
    CGAL::Comparison_result operator()(
        const Root_for_circles_2_2 & r1,
        const Root_for_circles_2_2 & r2) const;
};
```

#### 3.7.3 Compare_xy

```cpp
template < class AK >
class Compare_xy {
public:
    CGAL::Comparison_result operator()(
        const Root_for_circles_2_2 & r1,
        const Root_for_circles_2_2 & r2) const;
};
```

---

## 4. API参考

### 4.1 类型定义

```cpp
// 基本类型
typedef RT_                                     RT;  // 环类型
typedef typename Root_of_traits<RT>::RootOf_1  FT;  // 域类型

// 多项式类型
typedef CGAL::Polynomial_1_2<RT>                    Polynomial_1_2;
typedef CGAL::Polynomial_for_circles_2_2<RT>        Polynomial_for_circles_2_2;

// 根类型
typedef typename Root_of_traits<RT>::RootOf_2       Root_of_2;
typedef CGAL::Root_for_circles_2_2<RT>              Root_for_circles_2_2;
```

### 4.2 构造器

```cpp
// 构造圆多项式
Construct_polynomial_for_circles_2_2 construct_polynomial_for_circles_2_2_object() const;

// 构造直线多项式
Construct_polynomial_1_2 construct_polynomial_1_2_object() const;
```

### 4.3 算法函数

```cpp
// 求解器
Solve solve_object() const;

// 符号判定
Sign_at sign_at_object() const;

// 临界点计算
X_critical_points x_critical_points_object() const;
Y_critical_points y_critical_points_object() const;

// 比较器
Compare_x compare_x_object() const;
Compare_y compare_y_object() const;
Compare_xy compare_xy_object() const;
```

---

## 5. 使用指南

### 5.1 基本使用示例

#### 5.1.1 创建代数内核

```cpp
#include <CGAL/Algebraic_kernel_for_circles_2_2.h>
#include <CGAL/Quotient.h>
#include <CGAL/MP_Float.h>

// 定义数值类型
typedef CGAL::Quotient<CGAL::MP_Float> NT;

// 创建代数内核
typedef CGAL::Algebraic_kernel_for_circles_2_2<NT> Algebraic_kernel;
Algebraic_kernel ak;
```

#### 5.1.2 创建圆和直线

```cpp
// 获取构造器
auto circle_constructor = ak.construct_polynomial_for_circles_2_2_object();
auto line_constructor = ak.construct_polynomial_1_2_object();

// 创建圆: 中心(2, 3), 半径平方 = 25
auto circle1 = circle_constructor(NT(2), NT(3), NT(25));

// 创建圆: 中心(5, 3), 半径平方 = 16
auto circle2 = circle_constructor(NT(5), NT(3), NT(16));

// 创建直线: x + y - 5 = 0
auto line = line_constructor(NT(1), NT(1), NT(-5));
```

#### 5.1.3 计算交点

```cpp
// 获取求解器
auto solver = ak.solve_object();

// 存储结果的容器
std::vector<std::pair<Algebraic_kernel::Root_for_circles_2_2, unsigned>> results;

// 计算两圆交点
solver(circle1, circle2, std::back_inserter(results));

// 输出结果
for(const auto& result : results) {
    std::cout << "交点: (" << result.first.x() << ", " 
              << result.first.y() << ")" 
              << ", 重数: " << result.second << std::endl;
}
```

#### 5.1.4 计算临界点

```cpp
// 获取临界点计算器
auto x_critical = ak.x_critical_points_object();
auto y_critical = ak.y_critical_points_object();

// 计算X方向临界点（最左和最右点）
auto leftmost = x_critical(circle1, false);   // 最左点
auto rightmost = x_critical(circle1, true);    // 最右点

// 计算Y方向临界点（最下和最上点）
auto bottommost = y_critical(circle1, false);  // 最下点
auto topmost = y_critical(circle1, true);       // 最上点
```

#### 5.1.5 符号判定

```cpp
// 获取符号判定器
auto sign_at = ak.sign_at_object();

// 创建一个点
Algebraic_kernel::Root_for_circles_2_2 point(
    Algebraic_kernel::Root_of_2(3),
    Algebraic_kernel::Root_of_2(4)
);

// 判定点相对于圆的位置
CGAL::Sign sign = sign_at(circle1, point);

if(sign == CGAL::NEGATIVE) {
    std::cout << "点在圆内" << std::endl;
} else if(sign == CGAL::ZERO) {
    std::cout << "点在圆上" << std::endl;
} else {
    std::cout << "点在圆外" << std::endl;
}
```

### 5.2 高级使用示例

#### 5.2.1 与Circular_kernel集成

```cpp
#include <CGAL/Cartesian.h>
#include <CGAL/Circular_kernel_2.h>
#include <CGAL/Arr_circular_line_arc_traits_2.h>

// 定义内核链
using NT = CGAL::Quotient<CGAL::MP_Float>;
using Linear_k = CGAL::Cartesian<NT>;
using Algebraic_k = CGAL::Algebraic_kernel_for_circles_2_2<NT>;
using Circular_k = CGAL::Circular_kernel_2<Linear_k, Algebraic_k>;

// 使用圆弧traits
using Traits = CGAL::Arr_circular_line_arc_traits_2<Circular_k>;
```

#### 5.2.2 批量处理交点

```cpp
template<typename AK>
class CircleIntersectionProcessor {
    typedef typename AK::Polynomial_for_circles_2_2 Circle;
    typedef typename AK::Root_for_circles_2_2 Point;
    
    AK ak;
    
public:
    // 计算多个圆的所有交点
    void process_all_intersections(const std::vector<Circle>& circles) {
        auto solver = ak.solve_object();
        
        for(size_t i = 0; i < circles.size(); ++i) {
            for(size_t j = i+1; j < circles.size(); ++j) {
                std::vector<std::pair<Point, unsigned>> intersections;
                solver(circles[i], circles[j], 
                      std::back_inserter(intersections));
                
                if(!intersections.empty()) {
                    std::cout << "圆" << i << "与圆" << j 
                             << "有" << intersections.size() 
                             << "个交点" << std::endl;
                }
            }
        }
    }
};
```

#### 5.2.3 精确的圆弧长度计算

```cpp
template<typename AK>
class CircularArcProcessor {
    typedef typename AK::Polynomial_for_circles_2_2 Circle;
    typedef typename AK::Root_for_circles_2_2 Point;
    
    AK ak;
    
public:
    // 计算圆弧端点
    std::pair<Point, Point> compute_arc_endpoints(
        const Circle& circle,
        double start_angle,  // 弧度
        double end_angle)    // 弧度
    {
        auto x_critical = ak.x_critical_points_object();
        auto y_critical = ak.y_critical_points_object();
        
        // 获取圆心和半径
        typename AK::FT cx = circle.a();
        typename AK::FT cy = circle.b();
        typename AK::FT r_sq = circle.r_sq();
        
        // 这里需要更复杂的三角函数计算
        // 简化示例...
        
        return std::make_pair(Point(), Point());
    }
};
```

---

## 6. 应用场景

### 6.1 圆弧排列（Circular Arrangements）

圆弧排列是计算几何中的重要数据结构，用于表示由圆弧和线段组成的平面细分。

```cpp
// 创建圆弧排列
#include <CGAL/Arrangement_2.h>

typedef CGAL::Arrangement_2<Traits> Arrangement;

Arrangement arr;
// 插入圆弧和线段...
```

### 6.2 Voronoi图的圆弧边界

在某些应用中，Voronoi图的边界可能包含圆弧。

```cpp
// 处理带权重的Voronoi图
class WeightedVoronoi {
    Algebraic_kernel ak;
    
    // 计算两个加权点之间的等势线（圆弧）
    auto compute_bisector(
        const WeightedPoint& p1,
        const WeightedPoint& p2);
};
```

### 6.3 机器人路径规划

在机器人导航中，圆弧常用于平滑路径。

```cpp
class CircularPathPlanner {
    Algebraic_kernel ak;
    
    // 计算避障圆弧路径
    std::vector<CircularArc> plan_circular_path(
        const Point& start,
        const Point& goal,
        const std::vector<Circle>& obstacles);
};
```

### 6.4 CAD/CAM应用

```cpp
class CADCircleProcessor {
    Algebraic_kernel ak;
    
    // 计算圆的偏置
    Circle offset_circle(const Circle& original, double offset);
    
    // 圆的修剪
    std::vector<CircularArc> trim_circle(
        const Circle& circle,
        const std::vector<Line>& trim_lines);
};
```

---

## 7. 依赖关系

### 7.1 直接依赖

1. **Algebraic_foundations** - 代数基础设施
2. **Arithmetic_kernel** - 算术运算内核
3. **Number_types** - 数值类型支持
4. **CGAL_Core** - CGAL核心库
5. **Root_of_traits** - 代数根特征类

### 7.2 间接依赖

```
Algebraic_kernel_for_circles
    ├── Algebraic_foundations
    │   └── Number_types
    ├── Arithmetic_kernel
    │   └── CGAL_Core
    └── Circular_kernel_2 (反向依赖)
        └── Arrangement_on_surface_2
```

### 7.3 编译要求

- C++14或更高版本
- 支持模板的编译器
- 可选：GMP/MPFR库（用于精确算术）

---

## 8. 性能考虑

### 8.1 时间复杂度

| 操作 | 复杂度 | 说明 |
|------|--------|------|
| 圆-圆求交 | O(1) | 固定的代数运算 |
| 圆-直线求交 | O(1) | 二次方程求解 |
| 临界点计算 | O(1) | 直接计算 |
| 符号判定 | O(1) | 点代入计算 |
| 根比较 | O(1) | 代数比较 |

### 8.2 空间复杂度

- 圆多项式：3个数值（圆心坐标和半径平方）
- 直线多项式：3个系数
- 代数根：使用引用计数的句柄管理

### 8.3 优化策略

#### 8.3.1 数值类型选择

```cpp
// 快速但可能不精确
typedef double FastNT;
typedef CGAL::Algebraic_kernel_for_circles_2_2<FastNT> FastKernel;

// 精确但较慢
typedef CGAL::Quotient<CGAL::MP_Float> ExactNT;
typedef CGAL::Algebraic_kernel_for_circles_2_2<ExactNT> ExactKernel;

// 自适应精度
typedef CGAL::Lazy_exact_nt<CGAL::Quotient<CGAL::MP_Float>> AdaptiveNT;
typedef CGAL::Algebraic_kernel_for_circles_2_2<AdaptiveNT> AdaptiveKernel;
```

#### 8.3.2 缓存策略

```cpp
class CachedCircleIntersector {
    std::map<std::pair<Circle, Circle>, IntersectionResult> cache;
    
public:
    IntersectionResult intersect(const Circle& c1, const Circle& c2) {
        auto key = std::make_pair(c1, c2);
        if(cache.find(key) != cache.end()) {
            return cache[key];
        }
        // 计算并缓存结果
        auto result = compute_intersection(c1, c2);
        cache[key] = result;
        return result;
    }
};
```

### 8.4 内存管理

- 使用`Handle_for`实现引用计数
- 避免不必要的根对象复制
- 及时释放临时对象

---

## 9. 最佳实践

### 9.1 数值类型选择指南

1. **原型开发**：使用`double`快速验证算法
2. **生产环境**：使用`CGAL::Quotient<CGAL::MP_Float>`确保精确性
3. **性能关键**：使用`CGAL::Lazy_exact_nt`平衡精度和速度

### 9.2 错误处理

```cpp
template<typename AK>
class SafeCircleProcessor {
    AK ak;
    
public:
    bool safe_intersect(
        const typename AK::Polynomial_for_circles_2_2& c1,
        const typename AK::Polynomial_for_circles_2_2& c2,
        std::vector<typename AK::Root_for_circles_2_2>& results)
    {
        try {
            // 检查退化情况
            if(c1 == c2) {
                std::cerr << "警告：两个圆相同" << std::endl;
                return false;
            }
            
            // 检查半径
            if(c1.r_sq() <= 0 || c2.r_sq() <= 0) {
                std::cerr << "错误：非正半径" << std::endl;
                return false;
            }
            
            // 执行求交
            auto solver = ak.solve_object();
            solver(c1, c2, std::back_inserter(results));
            
            return true;
            
        } catch(const std::exception& e) {
            std::cerr << "求交失败: " << e.what() << std::endl;
            return false;
        }
    }
};
```

### 9.3 调试技巧

```cpp
// 启用CGAL断言
#define CGAL_KERNEL_CHECK

// 输出调试信息
template<typename AK>
void debug_circle(const typename AK::Polynomial_for_circles_2_2& circle) {
    std::cout << "圆: 中心(" << circle.a() << ", " << circle.b() 
              << "), 半径²=" << circle.r_sq() << std::endl;
}

// 验证结果
template<typename AK>
bool verify_intersection(
    const typename AK::Polynomial_for_circles_2_2& circle,
    const typename AK::Root_for_circles_2_2& point)
{
    auto sign_at = AK().sign_at_object();
    return sign_at(circle, point) == CGAL::ZERO;
}
```

### 9.4 性能分析

```cpp
#include <chrono>

template<typename AK>
class PerformanceAnalyzer {
public:
    void benchmark_intersections(int num_circles) {
        AK ak;
        auto solver = ak.solve_object();
        
        // 生成测试数据
        std::vector<typename AK::Polynomial_for_circles_2_2> circles;
        // ... 生成圆
        
        auto start = std::chrono::high_resolution_clock::now();
        
        // 执行求交
        for(int i = 0; i < num_circles; ++i) {
            for(int j = i+1; j < num_circles; ++j) {
                std::vector<std::pair<typename AK::Root_for_circles_2_2, unsigned>> results;
                solver(circles[i], circles[j], std::back_inserter(results));
            }
        }
        
        auto end = std::chrono::high_resolution_clock::now();
        auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end - start);
        
        std::cout << "处理" << num_circles << "个圆的所有交点耗时: " 
                  << duration.count() << "ms" << std::endl;
    }
};
```

### 9.5 代码组织建议

```cpp
// circle_processor.h
template<typename AlgebraicKernel>
class CircleProcessor {
public:
    typedef AlgebraicKernel AK;
    typedef typename AK::Polynomial_for_circles_2_2 Circle;
    typedef typename AK::Polynomial_1_2 Line;
    typedef typename AK::Root_for_circles_2_2 Point;
    
    // 公共接口
    std::vector<Point> intersect(const Circle& c1, const Circle& c2);
    bool is_inside(const Circle& c, const Point& p);
    
private:
    AK ak_;
    
    // 私有实现细节
    void validate_circle(const Circle& c);
};
```

---

## 10. 版本历史

### v1.0 (2025年1月)
- 初始版本发布
- 完整的技术文档
- 涵盖所有核心功能
- 包含使用示例和最佳实践

### 未来计划
- 支持三维圆和球体
- 优化特殊情况的性能
- 添加更多几何谓词
- 扩展对其他曲线的支持

---

## 附录A：常见问题

### Q1：为什么使用半径的平方而不是半径？
**答**：使用半径平方可以避免开方运算，保持计算的精确性。在代数运算中，平方根可能引入无理数，使用平方形式可以保持在有理数域内进行运算。

### Q2：如何处理数值精度问题？
**答**：建议使用CGAL提供的精确数值类型，如`CGAL::Quotient<CGAL::MP_Float>`或`CGAL::Lazy_exact_nt`。这些类型提供任意精度的算术运算。

### Q3：性能不满足要求怎么办？
**答**：
1. 首先分析瓶颈所在
2. 考虑使用过滤技术（如区间算术）
3. 对频繁计算的结果进行缓存
4. 在允许的情况下使用近似算法

### Q4：如何集成到现有项目？
**答**：Algebraic_kernel_for_circles是头文件库，只需包含相应头文件即可。确保项目支持C++14或更高版本。

---

## 附录B：术语表

| 术语 | 英文 | 说明 |
|------|------|------|
| 代数内核 | Algebraic Kernel | 提供代数运算的核心组件 |
| 多项式 | Polynomial | 数学多项式表示 |
| 代数根 | Algebraic Root | 多项式方程的解 |
| 环类型 | Ring Type | 支持加减乘运算的数值类型 |
| 域类型 | Field Type | 支持加减乘除运算的数值类型 |
| 临界点 | Critical Point | 函数的极值点 |
| 判别式 | Discriminant | 判断方程根的性质的表达式 |
| 重数 | Multiplicity | 根的重复次数 |

---

## 附录C：参考资料

1. CGAL官方文档：https://doc.cgal.org/
2. 《Computational Geometry: Algorithms and Applications》
3. 《Effective Computational Geometry for Curves and Surfaces》
4. CGAL源代码：https://github.com/CGAL/cgal

---

**文档编写者**：CGAL开发团队  
**最后更新**：2025年1月  
**版权声明**：本文档遵循CGAL许可协议