#ifndef CGAL_CNC_FEATURE_RECOGNITION_H
#define CGAL_CNC_FEATURE_RECOGNITION_H

#include <CGAL/Exact_predicates_inexact_constructions_kernel.h>
#include <CGAL/Surface_mesh.h>
#include <CGAL/Shape_detection/Efficient_RANSAC.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Plane.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cylinder.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Cone.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Torus.h>
#include <CGAL/Shape_detection/Efficient_RANSAC/Sphere.h>
#include <CGAL/Polygon_mesh_processing/border.h>
#include <CGAL/Polygon_mesh_processing/detect_features.h>
#include <CGAL/boost/graph/graph_traits_Surface_mesh.h>
#include <CGAL/property_map.h>
#include <boost/graph/adjacency_list.hpp>
#include <unordered_map>
#include <vector>
#include <set>

namespace CGAL {
namespace CNC {

// 核心类型定义
using Kernel = CGAL::Exact_predicates_inexact_constructions_kernel;
using Point_3 = Kernel::Point_3;
using Vector_3 = Kernel::Vector_3;
using Plane_3 = Kernel::Plane_3;
using Mesh = CGAL::Surface_mesh<Point_3>;
using Vertex_index = Mesh::Vertex_index;
using Face_index = Mesh::Face_index;
using Edge_index = Mesh::Edge_index;
using Halfedge_index = Mesh::Halfedge_index;

// RANSAC形状检测类型
using Point_with_normal = std::pair<Point_3, Vector_3>;
using Pwn_vector = std::vector<Point_with_normal>;
using Point_map = CGAL::First_of_pair_property_map<Point_with_normal>;
using Normal_map = CGAL::Second_of_pair_property_map<Point_with_normal>;
using Traits = CGAL::Shape_detection::Efficient_RANSAC_traits<Kernel, Pwn_vector, Point_map, Normal_map>;
using Efficient_RANSAC = CGAL::Shape_detection::Efficient_RANSAC<Traits>;
using Shape = CGAL::Shape_detection::Shape_base<Traits>;
using Plane = CGAL::Shape_detection::Plane<Traits>;
using Cylinder = CGAL::Shape_detection::Cylinder<Traits>;
using Cone = CGAL::Shape_detection::Cone<Traits>;
using Torus = CGAL::Shape_detection::Torus<Traits>;
using Sphere = CGAL::Shape_detection::Sphere<Traits>;

// 2.5D加工特征类型枚举
enum class FeatureType {
    UNKNOWN,
    HOLE,           // 孔
    BLIND_HOLE,     // 盲孔
    SLOT,           // 槽
    POCKET,         // 型腔
    STEP,           // 台阶
    BOSS,           // 凸台
    RIB,            // 筋板
    CHAMFER,        // 倒角
    FILLET          // 圆角
};

// 形状类型枚举
enum class ShapeType {
    PLANE,
    CYLINDER,
    CONE,
    TORUS,
    SPHERE,
    UNKNOWN
};

// 边界类型枚举
enum class EdgeType {
    SHARP,      // 锐边
    CONVEX,     // 凸边
    CONCAVE,    // 凹边
    SMOOTH      // 平滑边
};

// 检测到的形状信息
struct DetectedShape {
    ShapeType type;
    std::shared_ptr<Shape> shape;
    std::vector<std::size_t> inliers;  // 内点索引
    Plane_3 supporting_plane;          // 支撑平面（对于2.5D特征）
    double area;                        // 形状面积
    Point_3 centroid;                   // 质心

    // 形状特定参数
    struct Parameters {
        // 圆柱参数
        Point_3 axis_point;
        Vector_3 axis_direction;
        double radius;
        double height;

        // 平面参数
        Vector_3 normal;

        // 锥面参数
        double apex_angle;

        // 圆环参数
        double major_radius;
        double minor_radius;
    } params;
};

// AAG节点属性
struct AAGNodeAttribute {
    std::size_t shape_id;
    ShapeType shape_type;
    double area;
    Point_3 centroid;
    Vector_3 normal;
    std::vector<Face_index> faces;
    std::set<std::size_t> boundary_edges;
};

// AAG边属性
struct AAGEdgeAttribute {
    EdgeType edge_type;
    double dihedral_angle;
    double edge_length;
    bool is_boundary;
    std::vector<Edge_index> mesh_edges;
};

// 属性邻接图(AAG)定义
using AAGraph = boost::adjacency_list<
    boost::setS,
    boost::vecS,
    boost::undirectedS,
    AAGNodeAttribute,
    AAGEdgeAttribute
>;
using AAGVertex = boost::graph_traits<AAGraph>::vertex_descriptor;
using AAGEdge = boost::graph_traits<AAGraph>::edge_descriptor;

// 识别的加工特征
struct RecognizedFeature {
    FeatureType type;
    std::string name;
    std::vector<AAGVertex> involved_shapes;  // 涉及的形状节点
    std::unordered_map<std::string, double> parameters;  // 特征参数
    Point_3 location;
    Vector_3 machining_direction;  // 加工方向
    double volume;  // 特征体积
    int priority;   // 加工优先级
};

// CNC特征识别主类
class CNC_Feature_Recognition {
private:
    const Mesh& mesh_;
    std::vector<DetectedShape> detected_shapes_;
    AAGraph aag_graph_;
    std::vector<RecognizedFeature> recognized_features_;

    // 参数设置
    struct Parameters {
        // RANSAC参数
        double probability = 0.05;
        double min_points = 200;
        double epsilon = 0.002;
        double cluster_epsilon = 0.01;
        double normal_threshold = 0.9;

        // 边界检测参数
        double sharp_angle_threshold = 60.0;  // 度

        // 特征识别参数
        double hole_diameter_tolerance = 0.1;
        double pocket_depth_ratio = 0.2;
        double slot_aspect_ratio = 3.0;
    } params_;

public:
    CNC_Feature_Recognition(const Mesh& mesh) : mesh_(mesh) {}

    // 主处理流程
    bool process() {
        // 1. 形状检测
        if (!detect_shapes()) {
            return false;
        }

        // 2. 构建AAG
        if (!build_aag()) {
            return false;
        }

        // 3. 特征识别
        if (!recognize_features()) {
            return false;
        }

        return true;
    }

    // 步骤1: 使用RANSAC检测基本形状
    bool detect_shapes() {
        // 准备点云和法向量
        Pwn_vector points;
        points.reserve(mesh_.number_of_vertices());

        auto vnormals = mesh_.property_map<Vertex_index, Vector_3>("v:normal");
        if (!vnormals.second) {
            // 计算顶点法向量
            CGAL::Polygon_mesh_processing::compute_vertex_normals(
                mesh_,
                mesh_.add_property_map<Vertex_index, Vector_3>("v:normal").first
            );
            vnormals = mesh_.property_map<Vertex_index, Vector_3>("v:normal");
        }

        for (auto v : mesh_.vertices()) {
            points.push_back(std::make_pair(
                mesh_.point(v),
                vnormals.first[v]
            ));
        }

        // 设置RANSAC
        Efficient_RANSAC ransac;
        ransac.set_input(points);

        // 添加形状检测器
        ransac.add_shape_factory<Plane>();
        ransac.add_shape_factory<Cylinder>();
        ransac.add_shape_factory<Cone>();
        ransac.add_shape_factory<Torus>();
        ransac.add_shape_factory<Sphere>();

        // 执行检测
        Efficient_RANSAC::Parameters ransac_params;
        ransac_params.probability = params_.probability;
        ransac_params.min_points = params_.min_points;
        ransac_params.epsilon = params_.epsilon;
        ransac_params.cluster_epsilon = params_.cluster_epsilon;
        ransac_params.normal_threshold = params_.normal_threshold;

        ransac.detect(ransac_params);

        // 提取检测结果
        auto shapes = ransac.shapes();
        for (auto s : shapes) {
            DetectedShape ds;

            // 判断形状类型
            if (dynamic_cast<Plane*>(s.get())) {
                ds.type = ShapeType::PLANE;
                auto plane = dynamic_cast<Plane*>(s.get());
                ds.params.normal = plane->plane_normal();
            }
            else if (dynamic_cast<Cylinder*>(s.get())) {
                ds.type = ShapeType::CYLINDER;
                auto cyl = dynamic_cast<Cylinder*>(s.get());
                ds.params.axis_point = cyl->axis()[0];
                ds.params.axis_direction = cyl->axis()[1];
                ds.params.radius = cyl->radius();
            }
            else if (dynamic_cast<Cone*>(s.get())) {
                ds.type = ShapeType::CONE;
                auto cone = dynamic_cast<Cone*>(s.get());
                ds.params.apex_angle = cone->angle();
            }
            else if (dynamic_cast<Torus*>(s.get())) {
                ds.type = ShapeType::TORUS;
                auto torus = dynamic_cast<Torus*>(s.get());
                ds.params.major_radius = torus->major_radius();
                ds.params.minor_radius = torus->minor_radius();
            }
            else if (dynamic_cast<Sphere*>(s.get())) {
                ds.type = ShapeType::SPHERE;
            }

            ds.shape = s;
            ds.inliers = s->indices_of_assigned_points();

            // 计算形状属性
            compute_shape_attributes(ds);

            detected_shapes_.push_back(ds);
        }

        return !detected_shapes_.empty();
    }

    // 步骤2: 构建属性邻接图
    bool build_aag() {
        // 为每个检测到的形状创建AAG节点
        std::unordered_map<std::size_t, AAGVertex> shape_to_vertex;

        for (std::size_t i = 0; i < detected_shapes_.size(); ++i) {
            auto& shape = detected_shapes_[i];

            AAGNodeAttribute node_attr;
            node_attr.shape_id = i;
            node_attr.shape_type = shape.type;
            node_attr.area = shape.area;
            node_attr.centroid = shape.centroid;

            if (shape.type == ShapeType::PLANE) {
                node_attr.normal = shape.params.normal;
            }

            auto v = boost::add_vertex(node_attr, aag_graph_);
            shape_to_vertex[i] = v;
        }

        // 检测形状之间的邻接关系
        for (std::size_t i = 0; i < detected_shapes_.size(); ++i) {
            for (std::size_t j = i + 1; j < detected_shapes_.size(); ++j) {
                AAGEdgeAttribute edge_attr;

                if (are_adjacent(detected_shapes_[i], detected_shapes_[j], edge_attr)) {
                    boost::add_edge(
                        shape_to_vertex[i],
                        shape_to_vertex[j],
                        edge_attr,
                        aag_graph_
                    );
                }
            }
        }

        return boost::num_vertices(aag_graph_) > 0;
    }

    // 步骤3: 基于AAG进行特征识别
    bool recognize_features() {
        // 识别孔特征
        recognize_holes();

        // 识别槽特征
        recognize_slots();

        // 识别型腔特征
        recognize_pockets();

        // 识别台阶特征
        recognize_steps();

        // 识别凸台特征
        recognize_bosses();

        return !recognized_features_.empty();
    }

private:
    // 计算形状属性
    void compute_shape_attributes(DetectedShape& shape) {
        // 计算质心
        Point_3 centroid(0, 0, 0);
        for (auto idx : shape.inliers) {
            centroid = centroid + Vector_3(mesh_.point(Vertex_index(idx)));
        }
        shape.centroid = Point_3(
            centroid.x() / shape.inliers.size(),
            centroid.y() / shape.inliers.size(),
            centroid.z() / shape.inliers.size()
        );

        // 估算面积（简化计算）
        shape.area = shape.inliers.size() * 0.01;  // 假设每个点代表0.01单位面积
    }

    // 判断两个形状是否邻接
    bool are_adjacent(const DetectedShape& s1, const DetectedShape& s2, AAGEdgeAttribute& edge_attr) {
        // 检查是否有共享边界
        std::set<std::size_t> s1_points(s1.inliers.begin(), s1.inliers.end());
        std::set<std::size_t> s2_points(s2.inliers.begin(), s2.inliers.end());

        // 查找邻近点
        int adjacent_count = 0;
        for (auto p1 : s1_points) {
            for (auto p2 : s2_points) {
                double dist = CGAL::squared_distance(
                    mesh_.point(Vertex_index(p1)),
                    mesh_.point(Vertex_index(p2))
                );
                if (dist < params_.cluster_epsilon * params_.cluster_epsilon) {
                    adjacent_count++;
                }
            }
        }

        if (adjacent_count > 10) {  // 至少10个邻近点
            // 计算二面角
            if (s1.type == ShapeType::PLANE && s2.type == ShapeType::PLANE) {
                double cos_angle = s1.params.normal * s2.params.normal;
                edge_attr.dihedral_angle = std::acos(std::abs(cos_angle)) * 180.0 / M_PI;

                // 分类边类型
                if (edge_attr.dihedral_angle > params_.sharp_angle_threshold) {
                    edge_attr.edge_type = EdgeType::SHARP;
                } else if (cos_angle > 0) {
                    edge_attr.edge_type = EdgeType::CONVEX;
                } else {
                    edge_attr.edge_type = EdgeType::CONCAVE;
                }
            }

            return true;
        }

        return false;
    }

    // 识别孔特征
    void recognize_holes() {
        auto vertices = boost::vertices(aag_graph_);

        for (auto v : boost::make_iterator_range(vertices)) {
            auto& node = aag_graph_[v];

            // 圆柱面可能是孔
            if (node.shape_type == ShapeType::CYLINDER) {
                auto& shape = detected_shapes_[node.shape_id];

                RecognizedFeature feature;
                feature.type = FeatureType::HOLE;
                feature.name = "Hole_" + std::to_string(recognized_features_.size());
                feature.involved_shapes.push_back(v);
                feature.location = shape.centroid;
                feature.machining_direction = shape.params.axis_direction;
                feature.parameters["diameter"] = shape.params.radius * 2.0;
                feature.parameters["depth"] = shape.params.height;

                // 检查是否为盲孔（一端封闭）
                int plane_neighbors = 0;
                auto neighbors = boost::adjacent_vertices(v, aag_graph_);
                for (auto n : boost::make_iterator_range(neighbors)) {
                    if (aag_graph_[n].shape_type == ShapeType::PLANE) {
                        plane_neighbors++;
                    }
                }

                if (plane_neighbors == 1) {
                    feature.type = FeatureType::BLIND_HOLE;
                    feature.name = "BlindHole_" + std::to_string(recognized_features_.size());
                }

                recognized_features_.push_back(feature);
            }
        }
    }

    // 识别槽特征
    void recognize_slots() {
        auto vertices = boost::vertices(aag_graph_);

        for (auto v : boost::make_iterator_range(vertices)) {
            auto& node = aag_graph_[v];

            // 矩形平面底部 + 两个平行侧面 = 槽
            if (node.shape_type == ShapeType::PLANE) {
                auto neighbors = boost::adjacent_vertices(v, aag_graph_);
                std::vector<AAGVertex> plane_neighbors;

                for (auto n : boost::make_iterator_range(neighbors)) {
                    if (aag_graph_[n].shape_type == ShapeType::PLANE) {
                        plane_neighbors.push_back(n);
                    }
                }

                // 检查是否有两个平行的侧面
                if (plane_neighbors.size() >= 2) {
                    for (std::size_t i = 0; i < plane_neighbors.size(); ++i) {
                        for (std::size_t j = i + 1; j < plane_neighbors.size(); ++j) {
                            auto& shape1 = detected_shapes_[aag_graph_[plane_neighbors[i]].shape_id];
                            auto& shape2 = detected_shapes_[aag_graph_[plane_neighbors[j]].shape_id];

                            // 检查平行性
                            double dot = std::abs(shape1.params.normal * shape2.params.normal);
                            if (dot > 0.99) {  // 几乎平行
                                RecognizedFeature feature;
                                feature.type = FeatureType::SLOT;
                                feature.name = "Slot_" + std::to_string(recognized_features_.size());
                                feature.involved_shapes = {v, plane_neighbors[i], plane_neighbors[j]};
                                feature.location = node.centroid;

                                // 计算槽参数
                                double width = CGAL::squared_distance(
                                    shape1.centroid,
                                    shape2.centroid
                                );
                                feature.parameters["width"] = std::sqrt(width);
                                feature.parameters["depth"] = node.area / feature.parameters["width"];

                                recognized_features_.push_back(feature);
                                break;
                            }
                        }
                    }
                }
            }
        }
    }

    // 识别型腔特征
    void recognize_pockets() {
        auto vertices = boost::vertices(aag_graph_);

        for (auto v : boost::make_iterator_range(vertices)) {
            auto& node = aag_graph_[v];

            // 底面 + 多个侧面 = 型腔
            if (node.shape_type == ShapeType::PLANE) {
                auto neighbors = boost::adjacent_vertices(v, aag_graph_);
                std::vector<AAGVertex> wall_planes;

                for (auto n : boost::make_iterator_range(neighbors)) {
                    if (aag_graph_[n].shape_type == ShapeType::PLANE) {
                        // 检查是否为垂直侧壁
                        auto& wall_shape = detected_shapes_[aag_graph_[n].shape_id];
                        auto& bottom_shape = detected_shapes_[node.shape_id];

                        double dot = std::abs(wall_shape.params.normal * bottom_shape.params.normal);
                        if (dot < 0.1) {  // 近似垂直
                            wall_planes.push_back(n);
                        }
                    }
                }

                // 至少4个侧壁形成封闭型腔
                if (wall_planes.size() >= 4) {
                    RecognizedFeature feature;
                    feature.type = FeatureType::POCKET;
                    feature.name = "Pocket_" + std::to_string(recognized_features_.size());
                    feature.involved_shapes.push_back(v);
                    feature.involved_shapes.insert(
                        feature.involved_shapes.end(),
                        wall_planes.begin(),
                        wall_planes.end()
                    );
                    feature.location = node.centroid;
                    feature.parameters["area"] = node.area;
                    feature.parameters["wall_count"] = static_cast<double>(wall_planes.size());

                    recognized_features_.push_back(feature);
                }
            }
        }
    }

    // 识别台阶特征
    void recognize_steps() {
        auto edges = boost::edges(aag_graph_);

        for (auto e : boost::make_iterator_range(edges)) {
            auto& edge_attr = aag_graph_[e];
            auto src = boost::source(e, aag_graph_);
            auto tgt = boost::target(e, aag_graph_);

            // 两个平行平面 + 垂直连接面 = 台阶
            if (aag_graph_[src].shape_type == ShapeType::PLANE &&
                aag_graph_[tgt].shape_type == ShapeType::PLANE) {

                auto& shape1 = detected_shapes_[aag_graph_[src].shape_id];
                auto& shape2 = detected_shapes_[aag_graph_[tgt].shape_id];

                // 检查平行性
                double dot = std::abs(shape1.params.normal * shape2.params.normal);
                if (dot > 0.99) {  // 平行平面
                    RecognizedFeature feature;
                    feature.type = FeatureType::STEP;
                    feature.name = "Step_" + std::to_string(recognized_features_.size());
                    feature.involved_shapes = {src, tgt};

                    // 计算台阶高度
                    double height = std::abs(
                        (shape1.centroid - shape2.centroid) * shape1.params.normal
                    );
                    feature.parameters["height"] = height;
                    feature.location = Point_3(
                        (shape1.centroid.x() + shape2.centroid.x()) / 2,
                        (shape1.centroid.y() + shape2.centroid.y()) / 2,
                        (shape1.centroid.z() + shape2.centroid.z()) / 2
                    );

                    recognized_features_.push_back(feature);
                }
            }
        }
    }

    // 识别凸台特征
    void recognize_bosses() {
        auto vertices = boost::vertices(aag_graph_);

        for (auto v : boost::make_iterator_range(vertices)) {
            auto& node = aag_graph_[v];

            // 顶面 + 多个侧面 + 高于基准面 = 凸台
            if (node.shape_type == ShapeType::PLANE) {
                auto neighbors = boost::adjacent_vertices(v, aag_graph_);
                std::vector<AAGVertex> side_faces;
                bool has_base = false;

                for (auto n : boost::make_iterator_range(neighbors)) {
                    auto& neighbor_node = aag_graph_[n];
                    if (neighbor_node.shape_type == ShapeType::PLANE) {
                        auto& top_shape = detected_shapes_[node.shape_id];
                        auto& side_shape = detected_shapes_[neighbor_node.shape_id];

                        double dot = std::abs(top_shape.params.normal * side_shape.params.normal);

                        if (dot < 0.1) {  // 垂直侧面
                            side_faces.push_back(n);
                        } else if (dot > 0.99) {  // 平行底面
                            // 检查是否高于此面
                            double height = (top_shape.centroid - side_shape.centroid) * top_shape.params.normal;
                            if (height > 0) {
                                has_base = true;
                            }
                        }
                    }
                }

                // 有底面和至少3个侧面
                if (has_base && side_faces.size() >= 3) {
                    RecognizedFeature feature;
                    feature.type = FeatureType::BOSS;
                    feature.name = "Boss_" + std::to_string(recognized_features_.size());
                    feature.involved_shapes.push_back(v);
                    feature.involved_shapes.insert(
                        feature.involved_shapes.end(),
                        side_faces.begin(),
                        side_faces.end()
                    );
                    feature.location = node.centroid;
                    feature.parameters["top_area"] = node.area;
                    feature.parameters["side_count"] = static_cast<double>(side_faces.size());

                    recognized_features_.push_back(feature);
                }
            }
        }
    }

public:
    // 获取检测到的形状
    const std::vector<DetectedShape>& get_detected_shapes() const {
        return detected_shapes_;
    }

    // 获取AAG图
    const AAGraph& get_aag() const {
        return aag_graph_;
    }

    // 获取识别的特征
    const std::vector<RecognizedFeature>& get_features() const {
        return recognized_features_;
    }

    // 导出结果为JSON格式
    std::string export_to_json() const {
        std::stringstream ss;
        ss << "{\n";
        ss << "  \"detected_shapes\": " << detected_shapes_.size() << ",\n";
        ss << "  \"aag_nodes\": " << boost::num_vertices(aag_graph_) << ",\n";
        ss << "  \"aag_edges\": " << boost::num_edges(aag_graph_) << ",\n";
        ss << "  \"recognized_features\": [\n";

        for (std::size_t i = 0; i < recognized_features_.size(); ++i) {
            auto& feature = recognized_features_[i];
            ss << "    {\n";
            ss << "      \"type\": \"" << feature_type_to_string(feature.type) << "\",\n";
            ss << "      \"name\": \"" << feature.name << "\",\n";
            ss << "      \"location\": ["
               << feature.location.x() << ", "
               << feature.location.y() << ", "
               << feature.location.z() << "],\n";
            ss << "      \"parameters\": {\n";

            std::size_t param_count = 0;
            for (auto& [key, value] : feature.parameters) {
                ss << "        \"" << key << "\": " << value;
                if (++param_count < feature.parameters.size()) {
                    ss << ",";
                }
                ss << "\n";
            }

            ss << "      }\n";
            ss << "    }";
            if (i < recognized_features_.size() - 1) {
                ss << ",";
            }
            ss << "\n";
        }

        ss << "  ]\n";
        ss << "}\n";

        return ss.str();
    }

private:
    std::string feature_type_to_string(FeatureType type) const {
        switch (type) {
            case FeatureType::HOLE: return "HOLE";
            case FeatureType::BLIND_HOLE: return "BLIND_HOLE";
            case FeatureType::SLOT: return "SLOT";
            case FeatureType::POCKET: return "POCKET";
            case FeatureType::STEP: return "STEP";
            case FeatureType::BOSS: return "BOSS";
            case FeatureType::RIB: return "RIB";
            case FeatureType::CHAMFER: return "CHAMFER";
            case FeatureType::FILLET: return "FILLET";
            default: return "UNKNOWN";
        }
    }
};

} // namespace CNC
} // namespace CGAL

#endif // CGAL_CNC_FEATURE_RECOGNITION_H