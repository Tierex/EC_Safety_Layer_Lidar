#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/int16.hpp"

using std::placeholders::_1;

struct TrackInput
{
  int id = -1;

  float x = 0.0f;
  float y = 0.0f;
  float z = 0.0f;

  float vx = 0.0f;
  float vy = 0.0f;
  float vz = 0.0f;

  float closing_speed = 0.0f;

  float dx = 0.0f;
  float dy = 0.0f;
  float dz = 0.0f;

  float distance_xy = 0.0f;

  int num_points = 0;
  int hits = 0;
  int misses = 0;
  int age = 0;

  double source_time_sec = 0.0;
  double tracker_processing_ms = 0.0;
  double detector_processing_ms = 0.0;
};

class SafetySupervisor : public rclcpp::Node
{
public:
  SafetySupervisor()
  : Node("safety_supervisor")
  {
    declareParameters();
    loadParameters();

    auto qos = rclcpp::QoS(rclcpp::KeepLast(static_cast<size_t>(qos_depth_)))
                 .reliable()
                 .durability_volatile();

    sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      input_topic_,
      qos,
      std::bind(&SafetySupervisor::trackedObjectsCallback, this, _1));

    ego_speed_sub_ = this->create_subscription<std_msgs::msg::Float32>(
      ego_speed_topic_,
      10,
      std::bind(&SafetySupervisor::egoSpeedCallback, this, _1));

    pub_ = this->create_publisher<std_msgs::msg::Int16>(output_topic_, 10);

    const double period = 1.0 / publish_rate_hz_;
    timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(
        std::chrono::duration<double>(period)),
      std::bind(&SafetySupervisor::timerCallback, this));

    const auto now = this->now();
    last_msg_time_ = now;
    last_ego_speed_time_ = now;
    emergency_until_ = now;
    hazard_until_ = now;

    RCLCPP_INFO(this->get_logger(), "Safety supervisor started");
    RCLCPP_INFO(this->get_logger(), "Input:  %s", input_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Output: %s", output_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Ego speed input: %s", ego_speed_topic_.c_str());
    RCLCPP_INFO(this->get_logger(), "Signals: 0=Emergency, 1=Hazard, 2=Free");
    RCLCPP_INFO(
      this->get_logger(),
      "Sensor offsets: x=%.3f m, y=%.3f m, z=%.3f m",
      sensor_offset_x_,
      sensor_offset_y_,
      sensor_offset_z_);
  }

private:
  void declareParameters()
  {
    this->declare_parameter<std::string>("input_topic", "/tracked_objects");
    this->declare_parameter<std::string>("output_topic", "/safety_signal");
    this->declare_parameter<std::string>("ego_speed_topic", "/ego_speed");

    this->declare_parameter<int>("qos_depth", 20);
    this->declare_parameter<double>("publish_rate_hz", 20.0);

    this->declare_parameter<double>("stale_timeout_sec", 0.5);
    this->declare_parameter<int>("stale_signal", 0);

    this->declare_parameter<double>("ego_speed_stale_timeout_sec", 1.0);

    this->declare_parameter<double>("corridor_half_width", 0.75);
    this->declare_parameter<double>("min_x_consider", 0.0);
    this->declare_parameter<double>("max_x_consider", 20.0);

    this->declare_parameter<double>("emergency_distance", 0.50);
    this->declare_parameter<double>("hazard_distance", 1.50);

    this->declare_parameter<bool>("enable_ttc", true);
    this->declare_parameter<double>("emergency_ttc", 0.50);
    this->declare_parameter<double>("hazard_ttc", 1.50);
    this->declare_parameter<double>("min_closing_speed", 0.10);

    this->declare_parameter<int>("min_track_hits", 1);
    this->declare_parameter<int>("max_track_misses", 2);

    this->declare_parameter<bool>("enable_cut_in_prediction", true);
    this->declare_parameter<double>("hazard_prediction_horizon", 1.5);
    this->declare_parameter<double>("emergency_prediction_horizon", 0.5);
    this->declare_parameter<double>("min_lateral_speed", 0.10);

    this->declare_parameter<double>("emergency_hold_sec", 0.30);
    this->declare_parameter<double>("hazard_hold_sec", 0.50);

    this->declare_parameter<bool>("enable_brake_model", true);
    this->declare_parameter<double>("ego_speed_mps", 1.3889);
    this->declare_parameter<double>("max_decel_mps2", 1.5);
    this->declare_parameter<double>("system_delay_sec", 0.30);
    this->declare_parameter<double>("emergency_margin_m", 0.10);
    this->declare_parameter<double>("hazard_margin_m", 0.25);

    this->declare_parameter<double>("sensor_offset_x", -0.5);
    this->declare_parameter<double>("sensor_offset_y", 0.0);
    this->declare_parameter<double>("sensor_offset_z", 1.8);

    this->declare_parameter<bool>("debug", false);
    this->declare_parameter<int>("debug_every_n_frames", 10);
    this->declare_parameter<bool>("debug_timer", true);
  }

  void loadParameters()
  {
    input_topic_ = this->get_parameter("input_topic").as_string();
    output_topic_ = this->get_parameter("output_topic").as_string();
    ego_speed_topic_ = this->get_parameter("ego_speed_topic").as_string();

    qos_depth_ = this->get_parameter("qos_depth").as_int();
    publish_rate_hz_ = this->get_parameter("publish_rate_hz").as_double();

    stale_timeout_sec_ = this->get_parameter("stale_timeout_sec").as_double();
    stale_signal_ = this->get_parameter("stale_signal").as_int();

    ego_speed_stale_timeout_sec_ =
      this->get_parameter("ego_speed_stale_timeout_sec").as_double();

    corridor_half_width_ = this->get_parameter("corridor_half_width").as_double();
    min_x_consider_ = this->get_parameter("min_x_consider").as_double();
    max_x_consider_ = this->get_parameter("max_x_consider").as_double();

    emergency_distance_ = this->get_parameter("emergency_distance").as_double();
    hazard_distance_ = this->get_parameter("hazard_distance").as_double();

    enable_ttc_ = this->get_parameter("enable_ttc").as_bool();
    emergency_ttc_ = this->get_parameter("emergency_ttc").as_double();
    hazard_ttc_ = this->get_parameter("hazard_ttc").as_double();
    min_closing_speed_ = this->get_parameter("min_closing_speed").as_double();

    min_track_hits_ = this->get_parameter("min_track_hits").as_int();
    max_track_misses_ = this->get_parameter("max_track_misses").as_int();

    enable_cut_in_prediction_ =
      this->get_parameter("enable_cut_in_prediction").as_bool();

    hazard_prediction_horizon_ =
      this->get_parameter("hazard_prediction_horizon").as_double();

    emergency_prediction_horizon_ =
      this->get_parameter("emergency_prediction_horizon").as_double();

    min_lateral_speed_ =
      this->get_parameter("min_lateral_speed").as_double();

    emergency_hold_sec_ =
      this->get_parameter("emergency_hold_sec").as_double();

    hazard_hold_sec_ =
      this->get_parameter("hazard_hold_sec").as_double();

    enable_brake_model_ =
      this->get_parameter("enable_brake_model").as_bool();

    ego_speed_mps_ =
      this->get_parameter("ego_speed_mps").as_double();

    max_decel_mps2_ =
      this->get_parameter("max_decel_mps2").as_double();

    system_delay_sec_ =
      this->get_parameter("system_delay_sec").as_double();

    emergency_margin_m_ =
      this->get_parameter("emergency_margin_m").as_double();

    hazard_margin_m_ =
      this->get_parameter("hazard_margin_m").as_double();

    sensor_offset_x_ =
      this->get_parameter("sensor_offset_x").as_double();

    sensor_offset_y_ =
      this->get_parameter("sensor_offset_y").as_double();

    sensor_offset_z_ =
      this->get_parameter("sensor_offset_z").as_double();

    debug_ = this->get_parameter("debug").as_bool();
    debug_every_n_frames_ =
      this->get_parameter("debug_every_n_frames").as_int();

    debug_timer_ =
      this->get_parameter("debug_timer").as_bool();

    if (qos_depth_ < 1) qos_depth_ = 1;
    if (publish_rate_hz_ <= 0.0) publish_rate_hz_ = 20.0;

    stale_signal_ = std::clamp(stale_signal_, 0, 2);

    if (stale_timeout_sec_ < 0.0) stale_timeout_sec_ = 0.0;
    if (ego_speed_stale_timeout_sec_ < 0.0) ego_speed_stale_timeout_sec_ = 0.0;

    if (corridor_half_width_ < 0.0) corridor_half_width_ = 0.0;
    if (max_x_consider_ < min_x_consider_) max_x_consider_ = min_x_consider_;

    if (hazard_distance_ < emergency_distance_) hazard_distance_ = emergency_distance_;
    if (hazard_ttc_ < emergency_ttc_) hazard_ttc_ = emergency_ttc_;

    if (min_closing_speed_ < 0.0) min_closing_speed_ = 0.0;
    if (min_track_hits_ < 1) min_track_hits_ = 1;
    if (max_track_misses_ < 0) max_track_misses_ = 0;

    if (hazard_prediction_horizon_ < 0.0) hazard_prediction_horizon_ = 0.0;
    if (emergency_prediction_horizon_ < 0.0) emergency_prediction_horizon_ = 0.0;
    if (min_lateral_speed_ < 0.0) min_lateral_speed_ = 0.0;

    if (emergency_hold_sec_ < 0.0) emergency_hold_sec_ = 0.0;
    if (hazard_hold_sec_ < 0.0) hazard_hold_sec_ = 0.0;

    if (ego_speed_mps_ < 0.0) ego_speed_mps_ = 0.0;
    if (max_decel_mps2_ <= 0.0) max_decel_mps2_ = 1.0;
    if (system_delay_sec_ < 0.0) system_delay_sec_ = 0.0;
    if (emergency_margin_m_ < 0.0) emergency_margin_m_ = 0.0;
    if (hazard_margin_m_ < 0.0) hazard_margin_m_ = 0.0;

    if (debug_every_n_frames_ < 1) debug_every_n_frames_ = 1;
  }

  void egoSpeedCallback(const std_msgs::msg::Float32::SharedPtr msg)
  {
    ego_speed_live_mps_ = std::max(0.0f, msg->data);
    last_ego_speed_time_ = this->now();
    have_ego_speed_ = true;
  }

  double getCurrentEgoSpeed() const
  {
    if (have_ego_speed_) {
      const double age = (this->now() - last_ego_speed_time_).seconds();

      if (age <= ego_speed_stale_timeout_sec_) {
        return static_cast<double>(ego_speed_live_mps_);
      }
    }

    return ego_speed_mps_;
  }

  std::vector<TrackInput> parseTracks(
    const std_msgs::msg::Float32MultiArray & msg)
  {
    std::vector<TrackInput> tracks;

    if (msg.data.empty()) {
      return tracks;
    }

    std::size_t stride = 0;

    if (msg.layout.dim.size() >= 2 && msg.layout.dim[1].size > 0) {
      stride = msg.layout.dim[1].size;
    }

    if (stride != 13 && stride != 19) {
      if (msg.data.size() % 13 == 0) {
        stride = 13;
      } else if (msg.data.size() % 19 == 0) {
        stride = 19;
      } else {
        return tracks;
      }
    }

    const std::size_t n = msg.data.size() / stride;
    tracks.reserve(n);

    for (std::size_t i = 0; i < n; ++i) {
      const std::size_t k = i * stride;

      TrackInput t;
      t.id = static_cast<int>(msg.data[k + 0]);

      t.x = msg.data[k + 1];
      t.y = msg.data[k + 2];
      t.z = msg.data[k + 3];

      t.vx = msg.data[k + 4];
      t.vy = msg.data[k + 5];
      t.vz = msg.data[k + 6];

      if (stride == 13) {
        t.dx = msg.data[k + 7];
        t.dy = msg.data[k + 8];
        t.dz = msg.data[k + 9];

        t.num_points = static_cast<int>(msg.data[k + 10]);
        t.hits = static_cast<int>(msg.data[k + 11]);
        t.misses = static_cast<int>(msg.data[k + 12]);

        t.distance_xy = static_cast<float>(std::hypot(t.x, t.y));
        t.closing_speed = std::max(0.0f, -t.vx);
        t.age = t.hits + t.misses;
      } else {
        t.closing_speed = msg.data[k + 7];

        t.dx = msg.data[k + 8];
        t.dy = msg.data[k + 9];
        t.dz = msg.data[k + 10];

        t.distance_xy = msg.data[k + 11];

        t.num_points = static_cast<int>(msg.data[k + 12]);
        t.hits = static_cast<int>(msg.data[k + 13]);
        t.misses = static_cast<int>(msg.data[k + 14]);
        t.age = static_cast<int>(msg.data[k + 15]);

        t.source_time_sec =
          static_cast<double>(msg.data[k + 16]);

        t.tracker_processing_ms =
          static_cast<double>(msg.data[k + 17]);

        t.detector_processing_ms =
          static_cast<double>(msg.data[k + 18]);
      }

      tracks.push_back(t);
    }

    return tracks;
  }

  inline void toVehicleFrame(
    const TrackInput & t,
    double & x_v,
    double & y_v,
    double & z_v) const
  {
    x_v = static_cast<double>(t.x) + sensor_offset_x_;
    y_v = static_cast<double>(t.y) + sensor_offset_y_;
    z_v = static_cast<double>(t.z) + sensor_offset_z_;
  }

  bool willEnterCorridor(
    double y_v,
    double vy,
    double half_y,
    double prediction_horizon,
    double & time_to_corridor) const
  {
    const double lateral_limit = corridor_half_width_ + half_y;

    time_to_corridor =
      std::numeric_limits<double>::infinity();

    if (std::abs(y_v) <= lateral_limit) {
      time_to_corridor = 0.0;
      return true;
    }

    if (std::abs(vy) < min_lateral_speed_) {
      return false;
    }

    if (y_v > lateral_limit && vy >= 0.0) {
      return false;
    }

    if (y_v < -lateral_limit && vy <= 0.0) {
      return false;
    }

    const double distance_to_corridor =
      std::abs(y_v) - lateral_limit;

    time_to_corridor =
      distance_to_corridor / std::abs(vy);

    return time_to_corridor <= prediction_horizon;
  }

  double computeStoppingDistance(
    double v,
    double a,
    double delay,
    double margin) const
  {
    if (a <= 0.0) {
      return std::numeric_limits<double>::infinity();
    }

    return (v * v) / (2.0 * a) + v * delay + margin;
  }

  int computeSafetySignal(
    const std::vector<TrackInput> & tracks,
    TrackInput & worst_track,
    double & worst_front_distance,
    double & worst_ttc,
    double & worst_time_to_corridor)
  {
    int signal = 2;

    worst_front_distance =
      std::numeric_limits<double>::infinity();

    worst_ttc =
      std::numeric_limits<double>::infinity();

    worst_time_to_corridor =
      std::numeric_limits<double>::infinity();

    bool found_worst = false;

    const double v_ego = getCurrentEgoSpeed();

    const double d_emerg_stop =
      enable_brake_model_
        ? computeStoppingDistance(
            v_ego,
            max_decel_mps2_,
            system_delay_sec_,
            emergency_margin_m_)
        : 0.0;

    const double d_hazard_stop =
      enable_brake_model_
        ? computeStoppingDistance(
            v_ego,
            max_decel_mps2_,
            system_delay_sec_,
            hazard_margin_m_)
        : 0.0;

    const double emergency_distance_eff =
      std::max(emergency_distance_, d_emerg_stop);

    const double hazard_distance_eff =
      std::max(hazard_distance_, d_hazard_stop);

    for (const auto & t : tracks) {
      if (t.hits < min_track_hits_) continue;
      if (t.misses > max_track_misses_) continue;

      const double half_x = 0.5 * std::max(0.0f, t.dx);
      const double half_y = 0.5 * std::max(0.0f, t.dy);

      double x_v = 0.0;
      double y_v = 0.0;
      double z_v = 0.0;

      toVehicleFrame(t, x_v, y_v, z_v);
      (void)z_v;

      const double object_front_edge = x_v - half_x;
      const double object_rear_edge  = x_v + half_x;

      if (object_rear_edge < min_x_consider_) continue;
      if (object_front_edge > max_x_consider_) continue;

      const double lateral_limit = corridor_half_width_ + half_y;
      const bool in_corridor = std::abs(y_v) <= lateral_limit;

      double time_to_corridor_hazard =
        std::numeric_limits<double>::infinity();

      double time_to_corridor_emergency =
        std::numeric_limits<double>::infinity();

      bool will_enter_hazard_corridor = false;
      bool will_enter_emergency_corridor = false;

      if (enable_cut_in_prediction_) {
        will_enter_hazard_corridor =
          willEnterCorridor(
            y_v,
            static_cast<double>(t.vy),
            half_y,
            hazard_prediction_horizon_,
            time_to_corridor_hazard);

        will_enter_emergency_corridor =
          willEnterCorridor(
            y_v,
            static_cast<double>(t.vy),
            half_y,
            emergency_prediction_horizon_,
            time_to_corridor_emergency);
      }

      if (!in_corridor && !will_enter_hazard_corridor) {
        continue;
      }

      const double front_distance =
        std::max(0.0, object_front_edge);

      double closing_speed =
        static_cast<double>(t.closing_speed);

      if (!std::isfinite(closing_speed)) {
        closing_speed = 0.0;
      }

      double ttc =
        std::numeric_limits<double>::infinity();

      if (enable_ttc_ && closing_speed > min_closing_speed_) {
        ttc = front_distance / closing_speed;
      }

      const bool emergency_by_distance =
        in_corridor &&
        (front_distance <= emergency_distance_eff);

      const bool hazard_by_distance =
        in_corridor &&
        (front_distance <= hazard_distance_eff);

      const bool emergency_by_ttc =
        in_corridor &&
        enable_ttc_ &&
        (ttc <= emergency_ttc_);

      const bool hazard_by_ttc =
        in_corridor &&
        enable_ttc_ &&
        (ttc <= hazard_ttc_);

      const bool emergency_by_cut_in =
        !in_corridor &&
        will_enter_emergency_corridor &&
        (front_distance <= hazard_distance_eff);

      const bool hazard_by_cut_in =
        !in_corridor &&
        will_enter_hazard_corridor &&
        (front_distance <= max_x_consider_);

      const bool is_emergency =
        emergency_by_distance ||
        emergency_by_ttc ||
        emergency_by_cut_in;

      const bool is_hazard =
        hazard_by_distance ||
        hazard_by_ttc ||
        hazard_by_cut_in;

      const double best_time_to_corridor =
        std::min(
          time_to_corridor_hazard,
          time_to_corridor_emergency);

      if (!found_worst ||
          front_distance < worst_front_distance ||
          ttc < worst_ttc ||
          best_time_to_corridor < worst_time_to_corridor) {
        worst_track = t;
        worst_front_distance = front_distance;
        worst_ttc = ttc;
        worst_time_to_corridor = best_time_to_corridor;
        found_worst = true;
      }

      if (is_emergency) {
        return 0;
      }

      if (is_hazard) {
        signal = 1;
      }
    }

    return signal;
  }

  void trackedObjectsCallback(
    const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    ++frame_count_;
    last_msg_time_ = this->now();

    const auto tracks = parseTracks(*msg);

    TrackInput worst_track;

    double worst_front_distance =
      std::numeric_limits<double>::infinity();

    double worst_ttc =
      std::numeric_limits<double>::infinity();

    double worst_time_to_corridor =
      std::numeric_limits<double>::infinity();

    const int computed_signal =
      computeSafetySignal(
        tracks,
        worst_track,
        worst_front_distance,
        worst_ttc,
        worst_time_to_corridor);

    latest_signal_ = computed_signal;

    const auto now = this->now();

    if (computed_signal == 0) {
      emergency_until_ =
        now + rclcpp::Duration::from_seconds(emergency_hold_sec_);
    } else if (computed_signal == 1) {
      hazard_until_ =
        now + rclcpp::Duration::from_seconds(hazard_hold_sec_);
    }

    if (debug_ && (frame_count_ % debug_every_n_frames_) == 0) {
      RCLCPP_INFO(
        this->get_logger(),
        "Safety frame %ld | tracks=%zu | raw_signal=%d | ego_speed=%.3f m/s | worst_track=%d | front=%.2f m | ttc=%.2f s | t_corridor=%.2f s",
        frame_count_,
        tracks.size(),
        computed_signal,
        getCurrentEgoSpeed(),
        worst_track.id,
        worst_front_distance,
        worst_ttc,
        worst_time_to_corridor);
    }
  }

  int applyHoldAndStaleLogic()
  {
    const auto now = this->now();
    const double age_sec = (now - last_msg_time_).seconds();

    if (age_sec > stale_timeout_sec_) {
      return stale_signal_;
    }

    if (now < emergency_until_) {
      return 0;
    }

    if (now < hazard_until_) {
      return std::min(latest_signal_, 1);
    }

    return latest_signal_;
  }

  void timerCallback()
  {
    const int signal = applyHoldAndStaleLogic();

    std_msgs::msg::Int16 msg;
    msg.data = static_cast<int16_t>(signal);
    pub_->publish(msg);

    if (debug_ && debug_timer_) {
      const double age_sec =
        (this->now() - last_msg_time_).seconds();

      RCLCPP_INFO_THROTTLE(
        this->get_logger(),
        *this->get_clock(),
        1000,
        "Safety timer | signal=%d | latest_signal=%d | tracked_msg_age=%.3f s | ego_speed=%.3f m/s",
        signal,
        latest_signal_,
        age_sec,
        getCurrentEgoSpeed());
    }
  }

private:
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr sub_;
  rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr ego_speed_sub_;

  rclcpp::Publisher<std_msgs::msg::Int16>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  std::string input_topic_;
  std::string output_topic_;
  std::string ego_speed_topic_;

  int qos_depth_ = 20;
  double publish_rate_hz_ = 20.0;

  double stale_timeout_sec_ = 0.5;
  int stale_signal_ = 0;

  double ego_speed_stale_timeout_sec_ = 1.0;

  double corridor_half_width_ = 0.75;
  double min_x_consider_ = 0.0;
  double max_x_consider_ = 20.0;

  double emergency_distance_ = 0.50;
  double hazard_distance_ = 1.50;

  bool enable_ttc_ = true;
  double emergency_ttc_ = 0.50;
  double hazard_ttc_ = 1.50;
  double min_closing_speed_ = 0.10;

  int min_track_hits_ = 1;
  int max_track_misses_ = 2;

  bool enable_cut_in_prediction_ = true;
  double hazard_prediction_horizon_ = 1.5;
  double emergency_prediction_horizon_ = 0.5;
  double min_lateral_speed_ = 0.10;

  double emergency_hold_sec_ = 0.30;
  double hazard_hold_sec_ = 0.50;

  bool enable_brake_model_ = true;
  double ego_speed_mps_ = 1.3889;
  double max_decel_mps2_ = 1.5;
  double system_delay_sec_ = 0.30;
  double emergency_margin_m_ = 0.10;
  double hazard_margin_m_ = 0.25;

  double sensor_offset_x_ = -0.5;
  double sensor_offset_y_ = 0.0;
  double sensor_offset_z_ = 1.8;

  bool have_ego_speed_ = false;
  float ego_speed_live_mps_ = 0.0f;
  rclcpp::Time last_ego_speed_time_;

  bool debug_ = false;
  int debug_every_n_frames_ = 10;
  bool debug_timer_ = true;

  rclcpp::Time last_msg_time_;
  rclcpp::Time emergency_until_;
  rclcpp::Time hazard_until_;

  int latest_signal_ = 0;
  long frame_count_ = 0;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<SafetySupervisor>());
  rclcpp::shutdown();
  return 0;
}
