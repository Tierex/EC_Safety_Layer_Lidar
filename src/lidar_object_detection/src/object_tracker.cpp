// ============================================================================
// object_tracker.cpp
// ============================================================================
// Tracks detected objects from /detected_objects and publishes /tracked_objects.
//
// Input /detected_objects:
//   stride 11:
//     0 id
//     1 x
//     2 y
//     3 z
//     4 dx
//     5 dy
//     6 dz
//     7 num_points
//     8 source_time_sec
//     9 detector_processing_ms
//     10 detector_frame
//
// Output /tracked_objects:
//   stride 19:
//     0 id
//     1 x
//     2 y
//     3 z
//     4 vx
//     5 vy
//     6 vz
//     7 closing_speed
//     8 dx
//     9 dy
//     10 dz
//     11 distance_xy
//     12 num_points
//     13 hits
//     14 misses
//     15 age
//     16 track_timestamp_sec
//     17 tracker_processing_ms
//     18 detector_processing_ms
//
// Important:
// - Detection source_time_sec may be relative time from the detector.
// - The published track timestamp uses this->now().seconds(), so it follows
//   the node clock. With rosbag --clock, start this node with use_sim_time=true.
// ============================================================================

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdio>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "visualization_msgs/msg/marker.hpp"
#include "visualization_msgs/msg/marker_array.hpp"

using std::placeholders::_1;

struct Detection
{
  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;
  float dx = 0.0f;
  float dy = 0.0f;
  float dz = 0.0f;
  int num_points = 0;
  double source_time_sec = 0.0;
  double detector_processing_ms = 0.0;
};

struct Track
{
  int id = 0;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float vx = 0.0f;
  float vy = 0.0f;
  float vz = 0.0f;

  float dx = 0.0f;
  float dy = 0.0f;
  float dz = 0.0f;

  int num_points = 0;
  int hits = 0;
  int misses = 0;
  int age = 0;

  double last_track_stamp_sec = 0.0;
  double last_detector_processing_ms = 0.0;
};

struct MatchCandidate
{
  std::size_t track_idx = 0;
  std::size_t det_idx = 0;
  double cost = 0.0;
};

class ObjectTracker : public rclcpp::Node
{
public:
  ObjectTracker()
  : Node("object_tracker")
  {
    declareParameters();
    loadParameters();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(qos_depth_))
      .reliable()
      .durability_volatile();

    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_,
      qos,
      std::bind(&ObjectTracker::callback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      output_topic_,
      10);

    marker_pub_ = this->create_publisher<visualization_msgs::msg::MarkerArray>(
      marker_topic_,
      10);

    last_node_time_ = this->now();

    RCLCPP_INFO(this->get_logger(), "Object tracker started");
    RCLCPP_INFO(this->get_logger(), "Input:  %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Fields per track: %u", fields_per_track_);
  }

private:
  static constexpr uint32_t fields_per_track_ = 19;

  void declareParameters()
  {
    this->declare_parameter<std::string>("input_topic", "/detected_objects");
    this->declare_parameter<std::string>("output_topic", "/tracked_objects");
    this->declare_parameter<std::string>("marker_topic", "/tracked_object_markers");
    this->declare_parameter<std::string>("frame_id", "velodyne");

    this->declare_parameter<int>("qos_depth", 20);

    this->declare_parameter<double>("max_match_distance", 1.0);
    this->declare_parameter<double>("max_z_match_distance", 1.0);
    this->declare_parameter<double>("max_size_change_ratio", 2.5);

    this->declare_parameter<int>("max_missed_frames", 5);
    this->declare_parameter<int>("min_hits_to_publish", 1);

    this->declare_parameter<double>("velocity_alpha", 0.35);
    this->declare_parameter<double>("max_velocity", 15.0);

    this->declare_parameter<bool>("predict_missed_tracks", true);

    this->declare_parameter<double>("max_dt", 0.5);
    this->declare_parameter<double>("min_dt", 0.001);

    this->declare_parameter<bool>("publish_markers", true);
    this->declare_parameter<double>("marker_lifetime", 0.25);

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<int>("debug_every_n_frames", 30);
  }

  void loadParameters()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    marker_topic_ = this->get_parameter("marker_topic").as_string();
    frame_id_ = this->get_parameter("frame_id").as_string();

    qos_depth_ = this->get_parameter("qos_depth").as_int();

    max_match_distance_ = this->get_parameter("max_match_distance").as_double();
    max_z_match_distance_ = this->get_parameter("max_z_match_distance").as_double();
    max_size_change_ratio_ = this->get_parameter("max_size_change_ratio").as_double();

    max_missed_frames_ = this->get_parameter("max_missed_frames").as_int();
    min_hits_to_publish_ = this->get_parameter("min_hits_to_publish").as_int();

    velocity_alpha_ = this->get_parameter("velocity_alpha").as_double();
    max_velocity_ = this->get_parameter("max_velocity").as_double();

    predict_missed_tracks_ = this->get_parameter("predict_missed_tracks").as_bool();

    max_dt_ = this->get_parameter("max_dt").as_double();
    min_dt_ = this->get_parameter("min_dt").as_double();

    publish_markers_ = this->get_parameter("publish_markers").as_bool();
    marker_lifetime_ = this->get_parameter("marker_lifetime").as_double();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ = this->get_parameter("debug_every_n_frames").as_int();

    if (qos_depth_ < 1) {
      qos_depth_ = 1;
    }

    if (max_missed_frames_ < 0) {
      max_missed_frames_ = 0;
    }

    if (min_hits_to_publish_ < 1) {
      min_hits_to_publish_ = 1;
    }

    velocity_alpha_ = std::clamp(velocity_alpha_, 0.0, 1.0);

    if (min_dt_ <= 0.0) {
      min_dt_ = 0.001;
    }

    if (max_dt_ < min_dt_) {
      max_dt_ = min_dt_;
    }

    if (debug_every_n_frames_ < 1) {
      debug_every_n_frames_ = 1;
    }

    if (marker_lifetime_ < 0.0) {
      marker_lifetime_ = 0.25;
    }
  }

  std::vector<Detection> parseDetections(
    const std_msgs::msg::Float32MultiArray & msg)
  {
    std::vector<Detection> dets;

    if (msg.data.empty()) {
      return dets;
    }

    std::size_t stride = 0;

    if (msg.layout.dim.size() >= 2 && msg.layout.dim[1].size > 0) {
      stride = msg.layout.dim[1].size;
    }

    if (stride != 11 && stride != 8) {
      if (msg.data.size() % 11 == 0) {
        stride = 11;
      } else if (msg.data.size() % 8 == 0) {
        stride = 8;
      } else {
        return dets;
      }
    }

    const std::size_t n = msg.data.size() / stride;
    dets.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t k = i * stride;

      Detection d;
      d.x = msg.data[k + 1];
      d.y = msg.data[k + 2];
      d.z = msg.data[k + 3];
      d.dx = msg.data[k + 4];
      d.dy = msg.data[k + 5];
      d.dz = msg.data[k + 6];
      d.num_points = static_cast<int>(msg.data[k + 7]);

      if (stride >= 11) {
        d.source_time_sec = static_cast<double>(msg.data[k + 8]);
        d.detector_processing_ms = static_cast<double>(msg.data[k + 9]);
      }

      dets.push_back(d);
    }

    return dets;
  }

  static double elapsedMs(const std::chrono::steady_clock::time_point & start)
  {
    const auto end = std::chrono::steady_clock::now();
    return std::chrono::duration<double, std::milli>(end - start).count();
  }

  static float distanceXY(const Track & t)
  {
    return std::sqrt(t.x * t.x + t.y * t.y);
  }

  static float closingSpeed(const Track & t)
  {
    const float r = distanceXY(t);

    if (r < 1e-3f) {
      return 0.0f;
    }

    return -((t.x * t.vx + t.y * t.vy) / r);
  }

  float clampVelocity(float v) const
  {
    if (max_velocity_ <= 0.0) {
      return v;
    }

    const float max_v = static_cast<float>(max_velocity_);
    return std::clamp(v, -max_v, max_v);
  }

  bool sizeCompatible(float a, float b) const
  {
    if (max_size_change_ratio_ <= 1.0) {
      return true;
    }

    const double aa = std::max(0.05, static_cast<double>(a));
    const double bb = std::max(0.05, static_cast<double>(b));

    const double ratio = std::max(aa, bb) / std::min(aa, bb);

    return ratio <= max_size_change_ratio_;
  }

  static double xyDistance2Predicted(
    const Track & t,
    const Detection & d,
    double dt)
  {
    const double px =
      static_cast<double>(t.x) +
      static_cast<double>(t.vx) * dt;

    const double py =
      static_cast<double>(t.y) +
      static_cast<double>(t.vy) * dt;

    const double dx = px - static_cast<double>(d.x);
    const double dy = py - static_cast<double>(d.y);

    return dx * dx + dy * dy;
  }

  bool passesGates(
    const Track & t,
    const Detection & d,
    double dt) const
  {
    const double xy2 = xyDistance2Predicted(t, d, dt);

    if (xy2 > max_match_distance_ * max_match_distance_) {
      return false;
    }

    const double z_diff =
      std::abs(static_cast<double>(t.z) - static_cast<double>(d.z));

    if (z_diff > max_z_match_distance_) {
      return false;
    }

    if (!sizeCompatible(t.dx, d.dx) ||
        !sizeCompatible(t.dy, d.dy) ||
        !sizeCompatible(t.dz, d.dz))
    {
      return false;
    }

    return true;
  }

  void callback(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    ++frame_count_;

    const auto t0 = std::chrono::steady_clock::now();

    const rclcpp::Time node_now = this->now();

    double wall_dt = (node_now - last_node_time_).seconds();
    last_node_time_ = node_now;

    auto dets = parseDetections(*msg);

    double source_time = 0.0;
    double detector_ms = 0.0;

    if (!dets.empty() && dets.front().source_time_sec > 0.0) {
      source_time = dets.front().source_time_sec;
      detector_ms = dets.front().detector_processing_ms;
    }

    double dt = wall_dt;

    if (source_time > 0.0 && last_detection_source_time_sec_ > 0.0) {
      const double source_dt = source_time - last_detection_source_time_sec_;

      if (source_dt > 0.0) {
        dt = source_dt;
      }
    }

    if (source_time > 0.0) {
      last_detection_source_time_sec_ = source_time;
    }

    if (!std::isfinite(dt)) {
      dt = min_dt_;
    }

    dt = std::clamp(dt, min_dt_, max_dt_);

    for (auto & t : tracks_) {
      ++t.age;
      ++t.misses;

      if (predict_missed_tracks_) {
        t.x += static_cast<float>(t.vx * dt);
        t.y += static_cast<float>(t.vy * dt);
        t.z += static_cast<float>(t.vz * dt);
      }
    }

    std::vector<MatchCandidate> candidates;
    candidates.reserve(tracks_.size() * dets.size());

    for (std::size_t ti = 0; ti < tracks_.size(); ++ti) {
      for (std::size_t di = 0; di < dets.size(); ++di) {
        if (!passesGates(tracks_[ti], dets[di], dt)) {
          continue;
        }

        candidates.push_back(
          MatchCandidate{
            ti,
            di,
            xyDistance2Predicted(tracks_[ti], dets[di], dt)});
      }
    }

    std::sort(
      candidates.begin(),
      candidates.end(),
      [](const MatchCandidate & a, const MatchCandidate & b) {
        return a.cost < b.cost;
      });

    std::vector<bool> track_used(tracks_.size(), false);
    std::vector<bool> det_used(dets.size(), false);

    for (const auto & c : candidates) {
      if (track_used[c.track_idx] || det_used[c.det_idx]) {
        continue;
      }

      updateMatchedTrack(tracks_[c.track_idx], dets[c.det_idx], dt);

      track_used[c.track_idx] = true;
      det_used[c.det_idx] = true;
    }

    for (std::size_t di = 0; di < dets.size(); ++di) {
      if (!det_used[di]) {
        createTrack(dets[di]);
      }
    }

    tracks_.erase(
      std::remove_if(
        tracks_.begin(),
        tracks_.end(),
        [&](const Track & t) {
          return t.misses > max_missed_frames_;
        }),
      tracks_.end());

    const double tracker_ms = elapsedMs(t0);

    publishTracks(tracker_ms, detector_ms);

    if (publish_markers_) {
      publishMarkers(node_now);
    }

    if (debug_ && (frame_count_ % debug_every_n_frames_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Tracker frame %ld | detections=%zu tracks=%zu dt=%.4f detector_ms=%.3f tracker_ms=%.3f",
        frame_count_,
        dets.size(),
        tracks_.size(),
        dt,
        detector_ms,
        tracker_ms);
    }
  }

  void updateMatchedTrack(
    Track & t,
    const Detection & d,
    double dt)
  {
    const float vx_meas =
      clampVelocity(static_cast<float>((d.x - t.x) / dt));

    const float vy_meas =
      clampVelocity(static_cast<float>((d.y - t.y) / dt));

    const float vz_meas =
      clampVelocity(static_cast<float>((d.z - t.z) / dt));

    t.vx =
      static_cast<float>(
        (1.0 - velocity_alpha_) * t.vx +
        velocity_alpha_ * vx_meas);

    t.vy =
      static_cast<float>(
        (1.0 - velocity_alpha_) * t.vy +
        velocity_alpha_ * vy_meas);

    t.vz =
      static_cast<float>(
        (1.0 - velocity_alpha_) * t.vz +
        velocity_alpha_ * vz_meas);

    t.x = d.x;
    t.y = d.y;
    t.z = d.z;

    t.dx = d.dx;
    t.dy = d.dy;
    t.dz = d.dz;

    t.num_points = d.num_points;

    ++t.hits;
    t.misses = 0;

    // Important: use node clock. With use_sim_time=true this follows /clock.
    t.last_track_stamp_sec = this->now().seconds();
    t.last_detector_processing_ms = d.detector_processing_ms;
  }

  void createTrack(const Detection & d)
  {
    Track t;

    t.id = next_id_++;

    t.x = d.x;
    t.y = d.y;
    t.z = d.z;

    t.vx = 0.0f;
    t.vy = 0.0f;
    t.vz = 0.0f;

    t.dx = d.dx;
    t.dy = d.dy;
    t.dz = d.dz;

    t.num_points = d.num_points;

    t.hits = 1;
    t.misses = 0;
    t.age = 1;

    // Important: use node clock. With use_sim_time=true this follows /clock.
    t.last_track_stamp_sec = this->now().seconds();
    t.last_detector_processing_ms = d.detector_processing_ms;

    tracks_.push_back(t);
  }

  std::size_t countPublishableTracks() const
  {
    std::size_t count = 0;

    for (const auto & t : tracks_) {
      if (t.hits >= min_hits_to_publish_) {
        ++count;
      }
    }

    return count;
  }

  void publishTracks(
    double tracker_processing_ms,
    double detector_processing_ms)
  {
    std_msgs::msg::Float32MultiArray out;

    const std::size_t publish_count = countPublishableTracks();

    out.layout.dim.resize(2);

    out.layout.dim[0].label = "tracks";
    out.layout.dim[0].size = static_cast<uint32_t>(publish_count);
    out.layout.dim[0].stride =
      static_cast<uint32_t>(publish_count * fields_per_track_);

    out.layout.dim[1].label = "fields";
    out.layout.dim[1].size = fields_per_track_;
    out.layout.dim[1].stride = fields_per_track_;

    out.data.reserve(publish_count * fields_per_track_);

    const double now_stamp = this->now().seconds();

    for (const auto & t : tracks_) {
      if (t.hits < min_hits_to_publish_) {
        continue;
      }

      const double stamp =
        t.last_track_stamp_sec > 0.0 ?
        t.last_track_stamp_sec :
        now_stamp;

      const double det_ms =
        t.last_detector_processing_ms > 0.0 ?
        t.last_detector_processing_ms :
        detector_processing_ms;

      out.data.push_back(static_cast<float>(t.id));
      out.data.push_back(t.x);
      out.data.push_back(t.y);
      out.data.push_back(t.z);

      out.data.push_back(t.vx);
      out.data.push_back(t.vy);
      out.data.push_back(t.vz);

      out.data.push_back(closingSpeed(t));

      out.data.push_back(t.dx);
      out.data.push_back(t.dy);
      out.data.push_back(t.dz);

      out.data.push_back(distanceXY(t));

      out.data.push_back(static_cast<float>(t.num_points));
      out.data.push_back(static_cast<float>(t.hits));
      out.data.push_back(static_cast<float>(t.misses));
      out.data.push_back(static_cast<float>(t.age));

      out.data.push_back(static_cast<float>(stamp));
      out.data.push_back(static_cast<float>(tracker_processing_ms));
      out.data.push_back(static_cast<float>(det_ms));
    }

    pub_->publish(out);
  }

  void setLifetime(visualization_msgs::msg::Marker & m) const
  {
    const int sec = static_cast<int>(marker_lifetime_);
    const int nsec =
      static_cast<int>(
        (marker_lifetime_ - static_cast<double>(sec)) * 1e9);

    m.lifetime.sec = sec;
    m.lifetime.nanosec = nsec;
  }

  void publishMarkers(const rclcpp::Time & stamp)
  {
    visualization_msgs::msg::MarkerArray arr;

    visualization_msgs::msg::Marker del;
    del.header.frame_id = frame_id_;
    del.header.stamp = stamp;
    del.ns = "tracked_objects";
    del.id = 0;
    del.action = visualization_msgs::msg::Marker::DELETEALL;

    arr.markers.push_back(del);

    int id = 1;

    for (const auto & t : tracks_) {
      if (t.hits < min_hits_to_publish_) {
        continue;
      }

      visualization_msgs::msg::Marker box;
      box.header.frame_id = frame_id_;
      box.header.stamp = stamp;
      box.ns = "tracked_boxes";
      box.id = id++;
      box.type = visualization_msgs::msg::Marker::CUBE;
      box.action = visualization_msgs::msg::Marker::ADD;

      box.pose.position.x = t.x;
      box.pose.position.y = t.y;
      box.pose.position.z = t.z;
      box.pose.orientation.w = 1.0;

      box.scale.x = std::max(t.dx, 0.05f);
      box.scale.y = std::max(t.dy, 0.05f);
      box.scale.z = std::max(t.dz, 0.05f);

      box.color.r = 0.0f;
      box.color.g = 0.5f;
      box.color.b = 1.0f;
      box.color.a = t.misses == 0 ? 0.25f : 0.12f;

      setLifetime(box);
      arr.markers.push_back(box);

      visualization_msgs::msg::Marker text;
      text.header = box.header;
      text.ns = "tracked_labels";
      text.id = id++;
      text.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
      text.action = visualization_msgs::msg::Marker::ADD;

      text.pose.position.x = t.x;
      text.pose.position.y = t.y;
      text.pose.position.z = t.z + 0.5f * t.dz + 0.25f;
      text.pose.orientation.w = 1.0;

      text.scale.z = 0.25;

      text.color.r = 1.0f;
      text.color.g = 1.0f;
      text.color.b = 1.0f;
      text.color.a = 1.0f;

      char range_text[16];
      char speed_text[16];

      std::snprintf(
        range_text,
        sizeof(range_text),
        "%.2f",
        distanceXY(t));

      std::snprintf(
        speed_text,
        sizeof(speed_text),
        "%.2f",
        closingSpeed(t));

      text.text =
        "id " + std::to_string(t.id) +
        "\nr=" + std::string(range_text) + " m" +
        "\nv=" + std::string(speed_text) + " m/s";

      setLifetime(text);
      arr.markers.push_back(text);
    }

    marker_pub_->publish(arr);
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr pub_;
  rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr marker_pub_;

  std::string input_topic_;
  std::string output_topic_;
  std::string marker_topic_;
  std::string frame_id_;

  int qos_depth_ = 20;

  double max_match_distance_ = 1.0;
  double max_z_match_distance_ = 1.0;
  double max_size_change_ratio_ = 2.5;

  int max_missed_frames_ = 5;
  int min_hits_to_publish_ = 1;

  double velocity_alpha_ = 0.35;
  double max_velocity_ = 15.0;

  bool predict_missed_tracks_ = true;

  double max_dt_ = 0.5;
  double min_dt_ = 0.001;

  bool publish_markers_ = true;
  double marker_lifetime_ = 0.25;

  bool debug_ = false;
  int debug_every_n_frames_ = 30;

  rclcpp::Time last_node_time_;
  double last_detection_source_time_sec_ = 0.0;

  long frame_count_ = 0;

  std::vector<Track> tracks_;
  int next_id_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<ObjectTracker>());
  rclcpp::shutdown();
  return 0;
}