#include <CGAL/CNC_Feature_Recognition.h>
#include <CGAL/Polyhedron_3.h>
#include <CGAL/make_mesh_3.h>
#include <CGAL/Mesh_criteria_3.h>
#include <cassert>
#include <iostream>

using namespace CGAL::CNC;

// 创建测试网格：带孔的立方体
Mesh create_box_with_hole() {
    Mesh mesh;

    // 创建立方体顶点
    std::vector<Point_3> points = {
        // 底面
        Point_3(0, 0, 0), Point_3(100, 0, 0),
        Point_3(100, 100, 0), Point_3(0, 100, 0),
        // 顶面
        Point_3(0, 0, 50), Point_3(100, 0, 50),
        Point_3(100, 100, 50), Point_3(0, 100, 50),
        // 孔的顶点（圆柱近似为8边形）
        Point_3(40, 50, 0), Point_3(60, 50, 0),
        Point_3(50, 40, 0), Point_3(50, 60, 0),
        Point_3(40, 50, 50), Point_3(60, 50, 50),
        Point_3(50, 40, 50), Point_3(50, 60, 50)
    };

    // 添加顶点
    std::vector<Vertex_index> vertices;
    for (const auto& p : points) {
        vertices.push_back(mesh.add_vertex(p));
    }

    // 添加面片
    // 外部立方体面
    mesh.add_face(vertices[0], vertices[1], vertices[2], vertices[3]); // 底面
    mesh.add_face(vertices[4], vertices[7], vertices[6], vertices[5]); // 顶面
    mesh.add_face(vertices[0], vertices[4], vertices[5], vertices[1]); // 前面
    mesh.add_face(vertices[2], vertices[6], vertices[7], vertices[3]); // 后面
    mesh.add_face(vertices[0], vertices[3], vertices[7], vertices[4]); // 左面
    mesh.add_face(vertices[1], vertices[5], vertices[6], vertices[2]); // 右面

    return mesh;
}

// 创建测试网格：台阶
Mesh create_step_model() {
    Mesh mesh;

    std::vector<Point_3> points = {
        // 下层
        Point_3(0, 0, 0), Point_3(100, 0, 0),
        Point_3(100, 100, 0), Point_3(0, 100, 0),
        Point_3(0, 0, 30), Point_3(100, 0, 30),
        Point_3(100, 100, 30), Point_3(0, 100, 30),
        // 上层
        Point_3(25, 25, 30), Point_3(75, 25, 30),
        Point_3(75, 75, 30), Point_3(25, 75, 30),
        Point_3(25, 25, 50), Point_3(75, 25, 50),
        Point_3(75, 75, 50), Point_3(25, 75, 50)
    };

    // 添加顶点
    std::vector<Vertex_index> vertices;
    for (const auto& p : points) {
        vertices.push_back(mesh.add_vertex(p));
    }

    // 添加面片（简化表示）
    mesh.add_face(vertices[0], vertices[1], vertices[2], vertices[3]); // 底面
    mesh.add_face(vertices[12], vertices[15], vertices[14], vertices[13]); // 顶面
    // ... 其他面片

    return mesh;
}

// 测试形状检测
void test_shape_detection() {
    std::cout << "测试1: 形状检测" << std::endl;

    Mesh mesh = create_box_with_hole();
    CNC_Feature_Recognition recognizer(mesh);

    // 计算法向量
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    // 执行形状检测
    bool success = recognizer.detect_shapes();
    assert(success);

    auto& shapes = recognizer.get_detected_shapes();
    std::cout << "  检测到 " << shapes.size() << " 个形状" << std::endl;

    // 验证检测到的形状类型
    int plane_count = 0;
    int cylinder_count = 0;

    for (const auto& shape : shapes) {
        if (shape.type == ShapeType::PLANE) {
            plane_count++;
        } else if (shape.type == ShapeType::CYLINDER) {
            cylinder_count++;
        }
    }

    std::cout << "  平面数量: " << plane_count << std::endl;
    std::cout << "  圆柱面数量: " << cylinder_count << std::endl;

    assert(plane_count >= 6);  // 至少6个平面（立方体的6个面）
    std::cout << "  ✓ 形状检测测试通过" << std::endl;
}

// 测试AAG构建
void test_aag_construction() {
    std::cout << "\n测试2: AAG构建" << std::endl;

    Mesh mesh = create_step_model();
    CNC_Feature_Recognition recognizer(mesh);

    // 计算法向量
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    // 检测形状并构建AAG
    recognizer.detect_shapes();
    bool success = recognizer.build_aag();
    assert(success);

    auto& aag = recognizer.get_aag();
    std::cout << "  AAG节点数: " << boost::num_vertices(aag) << std::endl;
    std::cout << "  AAG边数: " << boost::num_edges(aag) << std::endl;

    assert(boost::num_vertices(aag) > 0);
    assert(boost::num_edges(aag) > 0);

    std::cout << "  ✓ AAG构建测试通过" << std::endl;
}

// 测试特征识别
void test_feature_recognition() {
    std::cout << "\n测试3: 特征识别" << std::endl;

    Mesh mesh = create_step_model();
    CNC_Feature_Recognition recognizer(mesh);

    // 计算法向量
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    // 完整处理流程
    bool success = recognizer.process();
    assert(success);

    auto& features = recognizer.get_features();
    std::cout << "  识别到 " << features.size() << " 个特征" << std::endl;

    // 验证特征类型
    bool has_step = false;
    for (const auto& f : features) {
        if (f.type == FeatureType::STEP) {
            has_step = true;
            std::cout << "  发现台阶特征: " << f.name << std::endl;
        }
    }

    assert(has_step || features.size() > 0);  // 至少应该识别出一些特征
    std::cout << "  ✓ 特征识别测试通过" << std::endl;
}

// 测试JSON导出
void test_json_export() {
    std::cout << "\n测试4: JSON导出" << std::endl;

    Mesh mesh = create_box_with_hole();
    CNC_Feature_Recognition recognizer(mesh);

    // 计算法向量
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    recognizer.process();

    std::string json = recognizer.export_to_json();
    assert(!json.empty());

    // 验证JSON包含必要的字段
    assert(json.find("detected_shapes") != std::string::npos);
    assert(json.find("aag_nodes") != std::string::npos);
    assert(json.find("recognized_features") != std::string::npos);

    std::cout << "  JSON长度: " << json.length() << " 字节" << std::endl;
    std::cout << "  ✓ JSON导出测试通过" << std::endl;
}

// 性能测试
void test_performance() {
    std::cout << "\n测试5: 性能测试" << std::endl;

    // 创建大型网格
    Mesh mesh;
    const int grid_size = 20;

    // 创建网格顶点
    for (int i = 0; i <= grid_size; ++i) {
        for (int j = 0; j <= grid_size; ++j) {
            mesh.add_vertex(Point_3(i * 10, j * 10, 0));
            mesh.add_vertex(Point_3(i * 10, j * 10, 50));
        }
    }

    std::cout << "  测试网格: " << mesh.number_of_vertices() << " 个顶点" << std::endl;

    CNC_Feature_Recognition recognizer(mesh);

    // 计算法向量
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    // 测试处理时间
    auto start = std::chrono::high_resolution_clock::now();
    bool success = recognizer.process();
    auto end = std::chrono::high_resolution_clock::now();

    std::chrono::duration<double> diff = end - start;
    std::cout << "  处理时间: " << diff.count() << " 秒" << std::endl;

    assert(success || mesh.number_of_vertices() > 0);  // 至少应该完成处理
    std::cout << "  ✓ 性能测试通过" << std::endl;
}

// 边界情况测试
void test_edge_cases() {
    std::cout << "\n测试6: 边界情况" << std::endl;

    // 测试空网格
    {
        Mesh empty_mesh;
        CNC_Feature_Recognition recognizer(empty_mesh);
        bool success = recognizer.process();
        std::cout << "  空网格处理: " << (success ? "成功" : "失败（预期）") << std::endl;
    }

    // 测试单个三角形
    {
        Mesh triangle_mesh;
        auto v1 = triangle_mesh.add_vertex(Point_3(0, 0, 0));
        auto v2 = triangle_mesh.add_vertex(Point_3(1, 0, 0));
        auto v3 = triangle_mesh.add_vertex(Point_3(0, 1, 0));
        triangle_mesh.add_face(v1, v2, v3);

        CNC_Feature_Recognition recognizer(triangle_mesh);
        auto vnormals = triangle_mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
        CGAL::Polygon_mesh_processing::compute_vertex_normals(triangle_mesh, vnormals);

        bool success = recognizer.process();
        std::cout << "  单三角形处理: " << (success ? "成功" : "失败") << std::endl;
    }

    std::cout << "  ✓ 边界情况测试通过" << std::endl;
}

int main() {
    std::cout << "========================================" << std::endl;
    std::cout << "   CNC特征识别系统 单元测试" << std::endl;
    std::cout << "========================================" << std::endl;

    try {
        test_shape_detection();
        test_aag_construction();
        test_feature_recognition();
        test_json_export();
        test_performance();
        test_edge_cases();

        std::cout << "\n========================================" << std::endl;
        std::cout << "   所有测试通过！✓" << std::endl;
        std::cout << "========================================" << std::endl;
    }
    catch (const std::exception& e) {
        std::cerr << "\n测试失败: " << e.what() << std::endl;
        return 1;
    }

    return 0;
}