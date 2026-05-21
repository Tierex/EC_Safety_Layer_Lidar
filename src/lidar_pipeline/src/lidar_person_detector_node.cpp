#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"

#include "sensor_msgs/msg/point_cloud2.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

#include <Eigen/Core>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/point_types.h>
#include <pcl/search/kdtree.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl_conversions/pcl_conversions.h>

using std::placeholders::_1;

class LidarPersonDetector : public rclcpp::Node
{
public:
  using PointT = pcl::PointXYZ;
  using CloudT = pcl::PointCloud<PointT>;

  struct ObjectBox
  {
    int id = 0;
    float x = 0.0f;
    float y = 0.0f;
    float z = 0.0f;
    float dx = 0.0f;
    float dy = 0.0f;
    float dz = 0.0f;
    int num_points = 0;
  };

  LidarPersonDetector()
  : Node("lidar_person_detector")
  {
    declareParameters();
    loadParameters();

    rclcpp::QoS qos{rclcpp::KeepLast(static_cast<size_t>(qos_depth_))};

    if (reliable_qos_) {
      qos.reliable();
    } else {
      qos.best_effort();
    }

    qos.durability_volatile();

    cloud_sub_ = this->create_subscription<sensor_msgs::msg::PointCloud2>(
      input_topic_,
      qos,
      std::bind(&LidarPersonDetector::cloudCallback, this, _1));

    object_pub_ =
      this->create_publisher<std_msgs::msg::Float32MultiArray>(output_topic_, 10);

    marker_pub_ =
      this->create_publisher<visualization_msgs::msg::MarkerArray>(marker_topic_, 10);

    RCLCPP_INFO(this->get_logger(), "Lidar detector started");
    RCLCPP_INFO(this->get_logger(), "Input:  %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Fields per detection: %u", fields_per_object_);
  }

private:
  static constexpr uint32_t fields_per_object_ = 11;
  static constexpr double pi_ = 3.14159265358979323846;

  void declareParameters()
  {
    this->declare_parameter<std::string>("input_topic", "/velodyne_points");
    this->declare_parameter<std::string>("output_topic", "/detected_objects");
    this->declare_parameter<std::string>("marker_topic", "/detected_object_markers");

    this->declare_parameter<int>("qos_depth", 10);
    this->declare_parameter<bool>("reliable_qos", false);
    this->declare_parameter<int>("process_every_n_frames", 1);

    this->declare_parameter<double>("roi_x_min", 0.0);
    this->declare_parameter<double>("roi_x_max", 10.0);
    this->declare_parameter<double>("roi_y_min", -10.0);
    this->declare_parameter<double>("roi_y_max", 10.0);
    this->declare_parameter<double>("roi_z_min", -3.0);
    this->declare_parameter<double>("roi_z_max", 3.0);

    this->declare_parameter<double>("voxel_size", 0.05);

    this->declare_parameter<bool>("use_ground_removal", true);
    this->declare_parameter<double>("ground_max_distance", 0.10);
    this->declare_parameter<double>("ground_max_angle_deg", 15.0);
    this->declare_parameter<int>("ground_max_iterations", 50);

    this->declare_parameter<double>("cluster_tolerance", 0.25);
    this->declare_parameter<int>("min_cluster_size", 8);
    this->declare_parameter<int>("max_cluster_size", 2000);

    this->declare_parameter<double>("min_x_size", 0.15);
    this->declare_parameter<double>("min_y_size", 0.15);
    this->declare_parameter<double>("min_z_size", 0.30);

    this->declare_parameter<double>("max_x_size", 1.20);
    this->declare_parameter<double>("max_y_size", 1.20);
    this->declare_parameter<double>("max_z_size", 2.00);

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<int>("debug_every_n_frames", 30);
    this->declare_parameter<bool>("debug_objects", false);

    this->declare_parameter<bool>("publish_markers", false);
    this->declare_parameter<double>("marker_lifetime", 0.25);
  }

  void loadParameters()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();

    qos_depth_ = this->get_parameter("qos_depth").as_int();
    reliable_qos_ = this->get_parameter("reliable_qos").as_bool();
    process_every_n_frames_ = this->get_parameter("process_every_n_frames").as_int();

    roi_x_min_ = this->get_parameter("roi_x_min").as_double();
    roi_x_max_ = this->get_parameter("roi_x_max").as_double();
    roi_y_min_ = this->get_parameter("roi_y_min").as_double();
    roi_y_max_ = this->get_parameter("roi_y_max").as_double();
    roi_z_min_ = this->get_parameter("roi_z_min").as_double();
    roi_z_max_ = this->get_parameter("roi_z_max").as_double();

    voxel_size_ = this->get_parameter("voxel_size").as_double();

    use_ground_removal_ = this->get_parameter("use_ground_removal").as_bool();
    ground_max_distance_ = this->get_parameter("ground_max_distance").as_double();
    ground_max_angle_deg_ = this->get_parameter("ground_max_angle_deg").as_double();
    ground_max_iterations_ = this->get_parameter("ground_max_iterations").as_int();

    cluster_tolerance_ = this->get_parameter("cluster_tolerance").as_double();
    min_cluster_size_ = this->get_parameter("min_cluster_size").as_int();
    max_cluster_size_ = this->get_parameter("max_cluster_size").as_int();

    min_x_size_ = this->get_parameter("min_x_size").as_double();
    min_y_size_ = this->get_parameter("min_y_size").as_double();
    min_z_size_ = this->get_parameter("min_z_size").as_double();

    max_x_size_ = this->get_parameter("max_x_size").as_double();
    max_y_size_ = this->get_parameter("max_y_size").as_double();
    max_z_size_ = this->get_parameter("max_z_size").as_double();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ = this->get_parameter("debug_every_n_frames").as_int();
    debug_objects_ = this->get_parameter("debug_objects").as_bool();

    publish_markers_ = this->get_parameter("publish_markers").as_bool();
    marker_lifetime_ = this->get_parameter("marker_lifetime").as_double();

    if (qos_depth_ < 1) qos_depth_ = 1;
    if (process_every_n_frames_ < 1) process_every_n_frames_ = 1;
    if (debug_every_n_frames_ < 1) debug_every_n_frames_ = 1;
    if (ground_max_iterations_ < 1) ground_max_iterations_ = 1;
    if (min_cluster_size_ < 1) min_cluster_size_ = 1;
    if (max_cluster_size_ < min_cluster_size_) max_cluster_size_ = min_cluster_size_;
  }

  void cloudCallback(const sensor_msgs::msg::PointCloud2::SharedPtr msg)
  {
    ++frame_count_;

    if ((frame_count_ % process_every_n_frames_) != 0) {
      return;
    }

    const auto t0 = std::chrono::steady_clock::now();

    double source_abs_sec = stampToSec(msg->header.stamp);

    if (!std::isfinite(source_abs_sec) || source_abs_sec <= 0.0) {
      source_abs_sec = this->now().seconds();
    }

    if (first_source_stamp_sec_ < 0.0) {
      first_source_stamp_sec_ = source_abs_sec;
    }

    const double source_time_sec = source_abs_sec - first_source_stamp_sec_;

    CloudT::Ptr input_cloud(new CloudT);
    pcl::fromROSMsg(*msg, *input_cloud);

    const int input_points = static_cast<int>(input_cloud->size());

    CloudT::Ptr roi_cloud = applyROI(input_cloud);
    const int roi_points = static_cast<int>(roi_cloud->size());

    std::vector<ObjectBox> objects;

    int downsampled_points = 0;
    int nonground_points = 0;
    std::size_t cluster_count = 0;

    if (roi_cloud->size() >= static_cast<std::size_t>(min_cluster_size_)) {
      CloudT::Ptr downsampled_cloud = downsampleCloud(roi_cloud);
      downsampled_points = static_cast<int>(downsampled_cloud->size());

      if (downsampled_cloud->size() >= static_cast<std::size_t>(min_cluster_size_)) {
        CloudT::Ptr work_cloud;

        if (use_ground_removal_) {
          work_cloud = removeGround(downsampled_cloud);
        } else {
          work_cloud = downsampled_cloud;
        }

        nonground_points = static_cast<int>(work_cloud->size());

        if (work_cloud->size() >= static_cast<std::size_t>(min_cluster_size_)) {
          const auto cluster_indices = clusterCloud(work_cloud);
          cluster_count = cluster_indices.size();
          objects = clustersToObjects(work_cloud, cluster_indices);
        }
      }
    }

    const double processing_ms = elapsedMs(t0);

    publishObjects(
      objects,
      source_time_sec,
      processing_ms,
      static_cast<float>(frame_count_));

    if (publish_markers_) {
      publishMarkers(objects, msg->header.frame_id, msg->header.stamp);
    }

    if (debug_ && (frame_count_ % debug_every_n_frames_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Frame %ld | input=%d roi=%d down=%d nonground=%d clusters=%zu objects=%zu detector_ms=%.3f source_time=%.6f",
        frame_count_,
        input_points,
        roi_points,
        downsampled_points,
        nonground_points,
        cluster_count,
        objects.size(),
        processing_ms,
        source_time_sec);

      if (debug_objects_) {
        for (const auto & obj : objects) {
          RCLCPP_INFO(
            this->get_logger(),
            "  Det %d: c=[%.2f %.2f %.2f], size=[%.2f %.2f %.2f], points=%d",
            obj.id,
            obj.x,
            obj.y,
            obj.z,
            obj.dx,
            obj.dy,
            obj.dz,
            obj.num_points);
        }
      }
    }
  }

  CloudT::Ptr applyROI(const CloudT::Ptr & cloud) const
  {
    CloudT::Ptr filtered(new CloudT);
    filtered->reserve(cloud->size());

    for (const auto & p : cloud->points) {
      if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
        continue;
      }

      if (p.x < roi_x_min_ || p.x > roi_x_max_) continue;
      if (p.y < roi_y_min_ || p.y > roi_y_max_) continue;
      if (p.z < roi_z_min_ || p.z > roi_z_max_) continue;

      filtered->points.push_back(p);
    }

    filtered->width = static_cast<uint32_t>(filtered->points.size());
    filtered->height = 1;
    filtered->is_dense = true;

    return filtered;
  }

  CloudT::Ptr downsampleCloud(const CloudT::Ptr & cloud) const
  {
    if (voxel_size_ <= 0.0) {
      return cloud;
    }

    CloudT::Ptr filtered(new CloudT);

    pcl::VoxelGrid<PointT> voxel;
    voxel.setInputCloud(cloud);

    const auto leaf = static_cast<float>(voxel_size_);
    voxel.setLeafSize(leaf, leaf, leaf);

    voxel.filter(*filtered);

    return filtered;
  }

  CloudT::Ptr removeGround(const CloudT::Ptr & cloud) const
  {
    if (cloud->size() < 20) {
      return cloud;
    }

    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);

    pcl::SACSegmentation<PointT> seg;
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_PERPENDICULAR_PLANE);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(ground_max_iterations_);
    seg.setDistanceThreshold(ground_max_distance_);
    seg.setAxis(Eigen::Vector3f(0.0f, 0.0f, 1.0f));
    seg.setEpsAngle(static_cast<float>(ground_max_angle_deg_ * pi_ / 180.0));
    seg.setInputCloud(cloud);
    seg.segment(*inliers, *coefficients);

    if (inliers->indices.empty()) {
      return cloud;
    }

    CloudT::Ptr nonground(new CloudT);

    pcl::ExtractIndices<PointT> extract;
    extract.setInputCloud(cloud);
    extract.setIndices(inliers);
    extract.setNegative(true);
    extract.filter(*nonground);

    return nonground;
  }

  std::vector<pcl::PointIndices> clusterCloud(const CloudT::Ptr & cloud) const
  {
    std::vector<pcl::PointIndices> cluster_indices;

    if (cloud->size() < static_cast<std::size_t>(min_cluster_size_)) {
      return cluster_indices;
    }

    pcl::search::KdTree<PointT>::Ptr tree(new pcl::search::KdTree<PointT>);
    tree->setInputCloud(cloud);

    pcl::EuclideanClusterExtraction<PointT> ec;
    ec.setClusterTolerance(cluster_tolerance_);
    ec.setMinClusterSize(min_cluster_size_);
    ec.setMaxClusterSize(max_cluster_size_);
    ec.setSearchMethod(tree);
    ec.setInputCloud(cloud);
    ec.extract(cluster_indices);

    return cluster_indices;
  }

  std::vector<ObjectBox> clustersToObjects(
    const CloudT::Ptr & cloud,
    const std::vector<pcl::PointIndices> & cluster_indices) const
  {
    std::vector<ObjectBox> objects;
    objects.reserve(cluster_indices.size());

    int detection_id = 0;

    for (const auto & cluster : cluster_indices) {
      float min_x = std::numeric_limits<float>::max();
      float min_y = std::numeric_limits<float>::max();
      float min_z = std::numeric_limits<float>::max();

      float max_x = std::numeric_limits<float>::lowest();
      float max_y = std::numeric_limits<float>::lowest();
      float max_z = std::numeric_limits<float>::lowest();

      for (const int idx : cluster.indices) {
        const auto & p = cloud->points[idx];

        min_x = std::min(min_x, p.x);
        min_y = std::min(min_y, p.y);
        min_z = std::min(min_z, p.z);

        max_x = std::max(max_x, p.x);
        max_y = std::max(max_y, p.y);
        max_z = std::max(max_z, p.z);
      }

      const float dx = max_x - min_x;
      const float dy = max_y - min_y;
      const float dz = max_z - min_z;

      if (dx < min_x_size_ || dy < min_y_size_ || dz < min_z_size_) {
        continue;
      }

      if (dx > max_x_size_ || dy > max_y_size_ || dz > max_z_size_) {
        continue;
      }

      ObjectBox obj;
      obj.id = detection_id++;

      obj.x = 0.5f * (min_x + max_x);
      obj.y = 0.5f * (min_y + max_y);
      obj.z = 0.5f * (min_z + max_z);

      obj.dx = dx;
      obj.dy = dy;
      obj.dz = dz;

      obj.num_points = static_cast<int>(cluster.indices.size());

      objects.push_back(obj);
    }

    return objects;
  }

  void publishObjects(
    const std::vector<ObjectBox> & objects,
    double source_stamp_sec,
    double detector_processing_ms,
    float detector_frame)
  {
    std_msgs::msg::Float32MultiArray msg;

    msg.layout.dim.resize(2);

    msg.layout.dim[0].label = "objects";
    msg.layout.dim[0].size = static_cast<uint32_t>(objects.size());
    msg.layout.dim[0].stride =
      static_cast<uint32_t>(objects.size() * fields_per_object_);

    msg.layout.dim[1].label = "fields";
    msg.layout.dim[1].size = fields_per_object_;
    msg.layout.dim[1].stride = fields_per_object_;

    msg.data.reserve(objects.size() * fields_per_object_);

    for (const auto & obj : objects) {
      msg.data.push_back(static_cast<float>(obj.id));

      msg.data.push_back(obj.x);
      msg.data.push_back(obj.y);
      msg.data.push_back(obj.z);

      msg.data.push_back(obj.dx);
      msg.data.push_back(obj.dy);
      msg.data.push_back(obj.dz);

      msg.data.push_back(static_cast<float>(obj.num_points));

      msg.data.push_back(static_cast<float>(source_stamp_sec));
      msg.data.push_back(static_cast<float>(detector_processing_ms));
      msg.data.push_back(detector_frame);
    }

    object_pub_->publish(msg);
  }

  void publishMarkers(
    const std::vector<ObjectBox> & objects,
    const std::string & frame_id,
    const builtin_interfaces::msg::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray marker_array;

    visualization_msgs::msg::Marker delete_marker;
    delete_marker.header.frame_id = frame_id;
    delete_marker.header.stamp = stamp;
    delete_marker.ns = "detected_objects";
    delete_marker.id = 0;
    delete_marker.action = visualization_msgs::msg::Marker::DELETEALL;

    marker_array.markers.push_back(delete_marker);

    int marker_id = 1;

    for (const auto & obj : objects) {
      visualization_msgs::msg::Marker box;
      box.header.frame_id = frame_id;
      box.header.stamp = stamp;
      box.ns = "detected_objects";
      box.id = marker_id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;

      box.pose.position.x = obj.x;
      box.pose.position.y = obj.y;
      box.pose.position.z = obj.z;
      box.pose.orientation.w = 1.0;

      box.scale.x = std::max(obj.dx, 0.01f);
      box.scale.y = std::max(obj.dy, 0.01f);
      box.scale.z = std::max(obj.dz, 0.01f);

      box.color.r = 0.0f;
      box.color.g = 1.0f;
      box.color.b = 0.0f;
      box.color.a = 0.25f;

      setMarkerLifetime(box);

      marker_array.markers.push_back(box);
    }

    marker_pub_->publish(marker_array);
  }

  void setMarkerLifetime(visualization_msgs::msg::Marker & marker) const
  {
    const int sec = static_cast<int>(marker_lifetime_);

    const int nanosec =
      static_cast<int>((marker_lifetime_ - static_cast<double>(sec)) * 1e9);

    marker.lifetime.sec = sec;
    marker.lifetime.nanosec = nanosec;
  }

  static double stampToSec(const builtin_interfaces::msg::Time & stamp)
  {
    return static_cast<double>(stamp.sec) +
           1e-9 * static_cast<double>(stamp.nanosec);
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point & start)
  {
    const auto end = std::chrono::steady_clock::now();

    return std::chrono::duration<double, std::milli>(end - start).count();
  }

private:
  rclcpp::Subscription<sensor_msgs::msg::PointCloud2>::SharedPtr cloud_sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr object_pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;

  int qos_depth_ = 10;
  bool reliable_qos_ = false;
  int process_every_n_frames_ = 1;

  double roi_x_min_ = 0.0;
  double roi_x_max_ = 10.0;
  double roi_y_min_ = -10.0;
  double roi_y_max_ = 10.0;
  double roi_z_min_ = -3.0;
  double roi_z_max_ = 3.0;

  double voxel_size_ = 0.05;

  bool use_ground_removal_ = true;
  double ground_max_distance_ = 0.10;
  double ground_max_angle_deg_ = 15.0;
  int ground_max_iterations_ = 50;

  double cluster_tolerance_ = 0.25;
  int min_cluster_size_ = 8;
  int max_cluster_size_ = 2000;

  double min_x_size_ = 0.15;
  double min_y_size_ = 0.15;
  double min_z_size_ = 0.70;

  double max_x_size_ = 1.20;
  double max_y_size_ = 1.20;
  double max_z_size_ = 2.30;

  bool debug_ = false;
  int debug_every_n_frames_ = 30;
  bool debug_objects_ = false;

  bool publish_markers_ = false;
  double marker_lifetime_ = 0.25;

  double first_source_stamp_sec_ = -1.0;
  long frame_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  rclcpp::spin(std::make_shared<LidarPersonDetector>());

  rclcpp::shutdown();

  return 0;
}
