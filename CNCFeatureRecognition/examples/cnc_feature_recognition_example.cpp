#include <CGAL/CNC_Feature_Recognition.h>
#include <CGAL/IO/OFF_reader.h>
#include <CGAL/IO/STL_reader.h>
#include <CGAL/Polygon_mesh_processing/compute_normal.h>
#include <CGAL/Timer.h>
#include <iostream>
#include <fstream>
#include <string>

using namespace CGAL::CNC;

// 读取网格文件
bool read_mesh(const std::string& filename, Mesh& mesh) {
    std::ifstream input(filename);
    if (!input) {
        std::cerr << "无法打开文件: " << filename << std::endl;
        return false;
    }

    // 根据文件扩展名选择读取方式
    std::string extension = filename.substr(filename.find_last_of('.') + 1);

    bool success = false;
    if (extension == "off" || extension == "OFF") {
        success = CGAL::read_off(input, mesh);
    } else if (extension == "stl" || extension == "STL") {
        std::vector<Point_3> points;
        std::vector<std::vector<std::size_t>> faces;
        success = CGAL::read_STL(input, points, faces);
        if (success) {
            CGAL::Polygon_mesh_processing::polygon_soup_to_polygon_mesh(points, faces, mesh);
        }
    } else {
        std::cerr << "不支持的文件格式: " << extension << std::endl;
        return false;
    }

    if (!success) {
        std::cerr << "读取文件失败: " << filename << std::endl;
        return false;
    }

    std::cout << "成功读取网格文件:" << std::endl;
    std::cout << "  顶点数: " << mesh.number_of_vertices() << std::endl;
    std::cout << "  面片数: " << mesh.number_of_faces() << std::endl;
    std::cout << "  边数: " << mesh.number_of_edges() << std::endl;

    return true;
}

// 打印特征识别结果
void print_recognition_results(const CNC_Feature_Recognition& recognizer) {
    std::cout << "\n==================== 特征识别结果 ====================" << std::endl;

    // 打印检测到的形状
    auto& shapes = recognizer.get_detected_shapes();
    std::cout << "\n检测到的基本形状: " << shapes.size() << " 个" << std::endl;
    for (std::size_t i = 0; i < shapes.size(); ++i) {
        std::cout << "  形状 " << i << ": ";
        switch (shapes[i].type) {
            case ShapeType::PLANE:
                std::cout << "平面 (法向: "
                         << shapes[i].params.normal << ")";
                break;
            case ShapeType::CYLINDER:
                std::cout << "圆柱面 (半径: "
                         << shapes[i].params.radius << ")";
                break;
            case ShapeType::CONE:
                std::cout << "锥面 (锥角: "
                         << shapes[i].params.apex_angle << "°)";
                break;
            case ShapeType::TORUS:
                std::cout << "圆环面 (主半径: "
                         << shapes[i].params.major_radius
                         << ", 次半径: "
                         << shapes[i].params.minor_radius << ")";
                break;
            case ShapeType::SPHERE:
                std::cout << "球面";
                break;
            default:
                std::cout << "未知";
        }
        std::cout << ", 内点数: " << shapes[i].inliers.size() << std::endl;
    }

    // 打印AAG信息
    auto& aag = recognizer.get_aag();
    std::cout << "\n属性邻接图 (AAG):" << std::endl;
    std::cout << "  节点数: " << boost::num_vertices(aag) << std::endl;
    std::cout << "  边数: " << boost::num_edges(aag) << std::endl;

    // 打印识别的加工特征
    auto& features = recognizer.get_features();
    std::cout << "\n识别的加工特征: " << features.size() << " 个" << std::endl;

    // 按特征类型统计
    std::map<FeatureType, int> feature_count;
    for (auto& f : features) {
        feature_count[f.type]++;
    }

    std::cout << "\n特征类型统计:" << std::endl;
    for (auto& [type, count] : feature_count) {
        std::cout << "  ";
        switch (type) {
            case FeatureType::HOLE: std::cout << "通孔"; break;
            case FeatureType::BLIND_HOLE: std::cout << "盲孔"; break;
            case FeatureType::SLOT: std::cout << "槽"; break;
            case FeatureType::POCKET: std::cout << "型腔"; break;
            case FeatureType::STEP: std::cout << "台阶"; break;
            case FeatureType::BOSS: std::cout << "凸台"; break;
            case FeatureType::RIB: std::cout << "筋板"; break;
            case FeatureType::CHAMFER: std::cout << "倒角"; break;
            case FeatureType::FILLET: std::cout << "圆角"; break;
            default: std::cout << "未知";
        }
        std::cout << ": " << count << " 个" << std::endl;
    }

    // 详细打印每个特征
    std::cout << "\n特征详细信息:" << std::endl;
    for (std::size_t i = 0; i < features.size(); ++i) {
        auto& f = features[i];
        std::cout << "\n[" << i+1 << "] " << f.name << std::endl;
        std::cout << "  位置: ("
                 << f.location.x() << ", "
                 << f.location.y() << ", "
                 << f.location.z() << ")" << std::endl;

        if (f.machining_direction != Vector_3(0, 0, 0)) {
            std::cout << "  加工方向: " << f.machining_direction << std::endl;
        }

        if (!f.parameters.empty()) {
            std::cout << "  参数:" << std::endl;
            for (auto& [key, value] : f.parameters) {
                std::cout << "    " << key << " = " << value << std::endl;
            }
        }

        std::cout << "  涉及形状: " << f.involved_shapes.size() << " 个" << std::endl;
    }
}

// 导出识别结果到文件
void export_results(const CNC_Feature_Recognition& recognizer, const std::string& output_file) {
    std::ofstream out(output_file);
    if (!out) {
        std::cerr << "无法创建输出文件: " << output_file << std::endl;
        return;
    }

    out << recognizer.export_to_json();
    out.close();

    std::cout << "\n结果已导出到: " << output_file << std::endl;
}

// 主程序
int main(int argc, char* argv[]) {
    std::cout << "========================================" << std::endl;
    std::cout << "     CGAL CNC 加工特征识别系统        " << std::endl;
    std::cout << "========================================" << std::endl;

    // 检查命令行参数
    if (argc < 2) {
        std::cout << "\n使用方法: " << argv[0] << " <输入网格文件> [输出JSON文件]" << std::endl;
        std::cout << "支持的格式: OFF, STL" << std::endl;
        std::cout << "\n示例:" << std::endl;
        std::cout << "  " << argv[0] << " model.off" << std::endl;
        std::cout << "  " << argv[0] << " model.stl features.json" << std::endl;
        return 1;
    }

    std::string input_file = argv[1];
    std::string output_file = (argc > 2) ? argv[2] : "features.json";

    // 读取网格
    Mesh mesh;
    if (!read_mesh(input_file, mesh)) {
        return 1;
    }

    // 计算顶点法向量（如果需要）
    std::cout << "\n计算顶点法向量..." << std::endl;
    auto vnormals = mesh.add_property_map<Vertex_index, Vector_3>("v:normal").first;
    CGAL::Polygon_mesh_processing::compute_vertex_normals(mesh, vnormals);

    // 创建特征识别器
    std::cout << "\n开始特征识别..." << std::endl;
    CNC_Feature_Recognition recognizer(mesh);

    // 计时
    CGAL::Timer timer;
    timer.start();

    // 执行识别
    bool success = recognizer.process();

    timer.stop();

    if (!success) {
        std::cerr << "\n特征识别失败！" << std::endl;
        return 1;
    }

    std::cout << "\n特征识别完成！用时: " << timer.time() << " 秒" << std::endl;

    // 打印结果
    print_recognition_results(recognizer);

    // 导出结果
    export_results(recognizer, output_file);

    // 生成加工建议
    std::cout << "\n==================== 加工建议 ====================" << std::endl;

    auto& features = recognizer.get_features();

    // 统计特征类型
    std::map<FeatureType, std::vector<const RecognizedFeature*>> feature_groups;
    for (auto& f : features) {
        feature_groups[f.type].push_back(&f);
    }

    // 建议加工顺序
    std::cout << "\n建议的加工顺序:" << std::endl;
    int order = 1;

    // 1. 先加工面
    if (feature_groups.count(FeatureType::STEP)) {
        std::cout << order++ << ". 粗铣台阶面 ("
                 << feature_groups[FeatureType::STEP].size() << " 个)" << std::endl;
    }

    // 2. 型腔和凸台
    if (feature_groups.count(FeatureType::POCKET)) {
        std::cout << order++ << ". 铣削型腔 ("
                 << feature_groups[FeatureType::POCKET].size() << " 个)" << std::endl;
    }

    if (feature_groups.count(FeatureType::BOSS)) {
        std::cout << order++ << ". 精铣凸台 ("
                 << feature_groups[FeatureType::BOSS].size() << " 个)" << std::endl;
    }

    // 3. 槽
    if (feature_groups.count(FeatureType::SLOT)) {
        std::cout << order++ << ". 铣槽 ("
                 << feature_groups[FeatureType::SLOT].size() << " 个)" << std::endl;
    }

    // 4. 孔
    if (feature_groups.count(FeatureType::HOLE)) {
        std::cout << order++ << ". 钻通孔 ("
                 << feature_groups[FeatureType::HOLE].size() << " 个)" << std::endl;
    }

    if (feature_groups.count(FeatureType::BLIND_HOLE)) {
        std::cout << order++ << ". 钻盲孔 ("
                 << feature_groups[FeatureType::BLIND_HOLE].size() << " 个)" << std::endl;
    }

    // 5. 倒角和圆角
    if (feature_groups.count(FeatureType::CHAMFER)) {
        std::cout << order++ << ". 倒角 ("
                 << feature_groups[FeatureType::CHAMFER].size() << " 个)" << std::endl;
    }

    if (feature_groups.count(FeatureType::FILLET)) {
        std::cout << order++ << ". 圆角 ("
                 << feature_groups[FeatureType::FILLET].size() << " 个)" << std::endl;
    }

    // 刀具建议
    std::cout << "\n建议的刀具:" << std::endl;

    // 根据特征推荐刀具
    std::set<std::string> tools;

    for (auto& f : features) {
        switch (f.type) {
            case FeatureType::HOLE:
            case FeatureType::BLIND_HOLE:
                if (f.parameters.count("diameter")) {
                    double d = f.parameters.at("diameter");
                    tools.insert("钻头 Ø" + std::to_string(static_cast<int>(d)) + "mm");
                }
                break;

            case FeatureType::SLOT:
                if (f.parameters.count("width")) {
                    double w = f.parameters.at("width");
                    tools.insert("立铣刀 Ø" + std::to_string(static_cast<int>(w)) + "mm");
                }
                break;

            case FeatureType::POCKET:
            case FeatureType::BOSS:
                tools.insert("立铣刀 Ø10mm (粗加工)");
                tools.insert("立铣刀 Ø6mm (精加工)");
                break;

            case FeatureType::CHAMFER:
                tools.insert("倒角铣刀 45°");
                break;

            case FeatureType::FILLET:
                tools.insert("球头铣刀");
                break;

            default:
                break;
        }
    }

    int tool_num = 1;
    for (auto& tool : tools) {
        std::cout << "  T" << tool_num++ << ": " << tool << std::endl;
    }

    std::cout << "\n========================================" << std::endl;
    std::cout << "          处理完成！" << std::endl;
    std::cout << "========================================" << std::endl;

    return 0;
}