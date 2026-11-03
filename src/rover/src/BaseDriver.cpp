#include <chrono>
#include <cmath>
#include <rcl/time.h>
#include <rclcpp/time.hpp>

#include "BaseDriver.h"

#include "MultiWii/protocol.h"
#include "serial/serial.h"
#include "util/clamp.h"

static const double V[4][3] = {
    // Motor Layout (UP is forward)
    // 4 2
    // 3 1
    {1, -1, -1},
    {1, 1, -1},
    {1, 1, 1},
    {1, -1, 1}};

#define DSHOT_NEUTRAL 1500
#define BACKUP_COMP 2
#define MAX_VELOCITY 100
static const double max_velocity = MAX_VELOCITY;
// Single wheel can top to 2 times of the maximum velocity
// This ensures the base can do same translation speed in 360 degrees
#define COMB_LIMIT 2
static const double comb_limit = COMB_LIMIT;

void BaseDriver::halt() {
  MSP::SET_MOTOR command;
  for (size_t i = 0; i < (sizeof(command.motor) / sizeof(*command.motor)); i++)
    command.motor[i] = DSHOT_NEUTRAL;
  device->send(command);
  odom_out.twist.twist.linear.x = 0;
  odom_out.twist.twist.linear.y = 0;
  odom_out.twist.twist.linear.z = 0;
  integrate_odometry();
  // odom_out();
}

// Velocity command
void BaseDriver::motion(double vx, double vy, double vr) {
  if (halted)
    return;
  integrate_odometry();
  // clamp() RETURNS its result and does not mutate its argument, so the
  // three bare clamp(vx, ...) statements below are no-ops and vx/vy/vr
  // reach the mixer unclamped. The motor output stays bounded regardless
  // (the comb_limit renormalisation caps |v[i]| at 2, so DSHOT stays in
  // [1298, 1700]), so that is left alone here -- changing it would change
  // how the base responds to an out-of-range command.
  //
  // The odometry must not inherit the bug though: without this, a command
  // of vx = 3.0 would be reported as 3.84 m/s while the wheels turn at the
  // same rate as vx = 1.0, and the EKF would fuse the lie.
  odom_out.twist.twist.linear.x = clamp(vx, -1.0, 1.0) * odom_k_x;
  odom_out.twist.twist.linear.y = clamp(vy, -1.0, 1.0) * odom_k_y;
  // odom_out();
  MSP::SET_MOTOR command;
  clamp(vx, -1.0, 1.0);
  clamp(vy, -1.0, 1.0);
  clamp(vr, -1.0, 1.0);
  double v[4];
  double amplitude = 0;
  for (int i = 0; i < 4; i++) {
    v[i] = vx * V[i][0] + vy * V[i][1] + vr * V[i][2];
    amplitude = std::max(amplitude, abs(v[i]));
  }
  // Throttle to conform to the speed limit
  amplitude /= comb_limit;
  if (amplitude > 1) {
    for (int i = 0; i < 4; i++)
      v[i] /= amplitude;
  }
  // Map to DSHOT values centered at 1500
  for (int i = 0; i < 4; i++) {
    double cmd_vel = std::round(DSHOT_NEUTRAL + max_velocity * v[i]);
    if (cmd_vel < DSHOT_NEUTRAL)
      cmd_vel -= BACKUP_COMP;
    clamp(cmd_vel, DSHOT_NEUTRAL - max_velocity * comb_limit - BACKUP_COMP,
          DSHOT_NEUTRAL + max_velocity * comb_limit);
    command.motor[i] = round(cmd_vel);
  }
  device->send(command);
}

// https://github.com/ros2/launch/issues/797
std::string hex_string(std::string s) {
  if (s.starts_with("0x") || s.starts_with("0X"))
    return s.substr(2);
  if (s.starts_with("x") || s.starts_with("X"))
    return s.substr(1);
  return s;
}

// Data handlers
void BaseDriver::update(MSP::RAW_IMU data,
                        std::chrono::system_clock::time_point timestamp) {
  data.accX *= 1;
  data.accY *= -1;
  data.accZ *= 1;
  data.gyrX *= 1;
  data.gyrY *= -1;
  data.gyrZ *= 1;
  auto &msg = imu_out;
  static const double g = 9.80 / 510.0;
  msg.linear_acceleration.x = data.accX * g;
  msg.linear_acceleration.y = data.accY * g;
  msg.linear_acceleration.z = data.accZ * g;
  // Raw MSP gyro units -> rad/s. The raw values are NOT deg/s on this
  // flight controller, though this code used to assume so: measured
  // against rf2o on 2026-09-24 they read ~14-16x too high, i.e. raw
  // sensor counts at 16.4 LSB per deg/s (the +-2000 dps range).
  const double r = gyro_scale * 3.1415926 / 180.0;
  msg.angular_velocity.x = data.gyrX * r;
  msg.angular_velocity.y = data.gyrY * r;
  msg.angular_velocity.z = data.gyrZ * r;
  odom_out.twist.twist.angular = msg.angular_velocity;
  // Only z reaches the EKF (vyaw). The sign is applied here, in the
  // odometry only: /rover/base/imu keeps the raw flight-controller sign.
  odom_out.twist.twist.angular.z = odom_gyro_z_sign * data.gyrZ * r;
  SET_ROS_STAMP(msg, timestamp);
  imu_status.acc_updated = true;
  if (imu_status.updated()) {
    msg();
    imu_status.reset();
  }
}

#define deg2rad(deg) (static_cast<double>(deg) * 3.1415926 / 180.0)
// Data handlers
void BaseDriver::update(MSP::ATTITUDE data,
                        std::chrono::system_clock::time_point timestamp) {
  const double x = deg2rad(data.angx / 10.0);   // roll
  const double y = deg2rad(-data.angy / 10.0);  // pitch
  const double z = deg2rad(data.heading / 1.0); // yaw
  // Convert to quaternion
  const double cx = cos(x / 2), sx = sin(x / 2);
  const double cy = cos(y / 2), sy = sin(y / 2);
  const double cz = cos(z / 2), sz = sin(z / 2);
  auto &msg = imu_out;
  msg.orientation.w = cx * cy * cz + sx * sy * sz;
  msg.orientation.x = sx * cy * cz - cx * sy * sz;
  msg.orientation.y = cx * sy * cz + sx * cy * sz;
  msg.orientation.z = cx * cy * sz - sx * sy * cz;
  odom_out.pose.pose.orientation = msg.orientation;
  integrate_odometry(z);
  odom_out();
  SET_ROS_STAMP(msg, timestamp);
  imu_status.att_updated = true;
  if (imu_status.updated()) {
    msg();
    imu_status.reset();
  }
}

// Speed-scaled twist covariance, written into the two entries the EKF
// actually reads: [0] = vx, [7] = vy of the row-major 6x6. Faster commands
// slip more, so the variance grows with speed; at a standstill it collapses
// to sigma_floor^2 and the message becomes a zero-velocity update.
void BaseDriver::set_twist_covariance() {
  const double sx = odom_slip_frac_x * odom_out.twist.twist.linear.x;
  const double sy = odom_slip_frac_y * odom_out.twist.twist.linear.y;
  const double floor_var = odom_sigma_floor * odom_sigma_floor;
  odom_out.twist.covariance[0] = sx * sx + floor_var;
  odom_out.twist.covariance[7] = sy * sy + floor_var;
}

void BaseDriver::integrate_odometry() {
  if (std::isnan(odom_last_heading)) {
    // No IMU heading yet, so there is no rotation to integrate through.
    // Still advance the stamp: a message frozen at construction time falls
    // outside robot_localization's sensor_timeout window and is dropped
    // silently, which looks exactly like the filter ignoring this input.
    odom_out.header.stamp = rclcpp::now();
    return;
  }
  integrate_odometry(odom_last_heading);
}

void BaseDriver::integrate_odometry(double current_heading) {
  auto t1 = rclcpp::now();
  auto t0 = rclcpp::Time(odom_out.header.stamp);
  // Get delta time in seconds
  const double dt = (t1 - t0).seconds();
  odom_out.header.stamp = t1;
  // Initialize heading
  if (std::isnan(odom_initial_heading))
    odom_initial_heading = current_heading;
  if (std::isnan(odom_last_heading))
    odom_last_heading = current_heading;
  // Calculate average heading between two updates
  const double r =
      (odom_last_heading + current_heading) / 2.f - odom_initial_heading;
  odom_last_heading = current_heading;
  // Calculate distance traveled
  const double dx = odom_out.twist.twist.linear.x * dt;
  const double dy = odom_out.twist.twist.linear.y * dt;
  // Calculate new position
  const double c = cos(r), s = sin(r);
  odom_out.pose.pose.position.x += dx * c - dy * s;
  odom_out.pose.pose.position.y += dx * s + dy * c;
}

void BaseDriver::open_device(unsigned max_attempts, bool log) {
  while (max_attempts > 0) {
    max_attempts--;
    device = nullptr;
    const std::string name = (param.vid.empty() ? "----" : param.vid) + ":" +
                             (param.pid.empty() ? "----" : param.pid);
    const auto ports = serial::locate(param.vid, param.pid);
    if (ports.empty()) {
      if (log)
        RCLCPP_WARN(get_logger(), "No serial port found for device %s",
                    name.c_str());
      std::this_thread::sleep_for(1000ms);
      continue;
    } else if (ports.size() > 1) {
      if (log)
        RCLCPP_WARN(get_logger(), "Multiple serial ports found for device %s",
                    name.c_str());
    }
    const auto port = ports[0];
    if (log)
      RCLCPP_INFO(get_logger(), "Opening serial port %s (%s:%s), baudrate %d",
                  port.path.c_str(), port.vid.c_str(), port.pid.c_str(),
                  param.baud);
    device = std::make_unique<MultiWii::Device>(port.path, param.baud);
    // Check connection
    try {
      for (unsigned i = 0; i < 10; i++) {
        device->query<MSP::IDENT>();
        std::this_thread::sleep_for(50ms);
      }
    } catch (const std::exception &e) {
      if (log)
        RCLCPP_ERROR(get_logger(), "MSP Device Error: %s", e.what());
      device = nullptr;
      std::this_thread::sleep_for(200ms);
      continue;
    }
    break;
  }
  if (device == nullptr)
    throw std::runtime_error("Failed to open serial port");
}

BaseDriver::BaseDriver() : Node("base") {
  // Initialize serial connection
  declare_parameter<std::string>("pid", "");
  declare_parameter<std::string>("vid", "");
  declare_parameter<int>("baud", 115200);
  // Per-axis throttle -> m/s gains, MEASURED against rf2o with
  // scripts/odom_diagnose.py (2026-09-24, teleop throttle 0.25).
  // rf2o's own scale was checked against a 90 in (2.286 m) tape run: 1%.
  //
  //   k_x 0.40  forward: rf2o/command slope 0.29-0.33 of the old 1.28,
  //             r 0.88-0.98, ~600 scans. Not lag, not stalls. Confirmed
  //             at 0.40: slope 0.96, r 0.93, 349 scans (run 190947).
  //   k_y 0.30  strafe: slope 1.68 left / 1.69 right against 0.18, r
  //             0.94 / 0.88, 177 pure-strafe scans (run 190947). An
  //             earlier 0.18 came from runs where the few strafe samples
  //             were mixed with other motion (r 0.57-0.65).
  //
  // The motors are near their deadband at throttle 0.25, so the real curve
  // is not linear: these gains are right around that throttle and will
  // UNDER-report at high throttle. Re-run the diagnose script at the speed
  // you actually drive if that changes.
  declare_parameter<double>("k_x", 0.40);
  declare_parameter<double>("k_y", 0.30);
  // Deliberately pessimistic starting point. Over-trusting velocity
  // integrated from a command is the one way the EKF ends up worse than
  // scan matching alone; tighten these only after k_x/k_y are calibrated.
  declare_parameter<double>("slip_frac_x", 0.30);
  declare_parameter<double>("slip_frac_y", 0.50);
  declare_parameter<double>("sigma_floor", 0.01);
  // Gyro Z -> REP-103 sign. MEASURED -1 (2026-09-24, scripts/odom_diagnose.py:
  // gyro vs rf2o yaw rate slope -14.4, r -0.88 over 539 turning scans) --
  // the flight controller counts yaw clockwise-positive.
  declare_parameter<double>("gyro_z_sign", -1.0);
  // deg/s per raw gyro unit. 1/16.4 = +-2000 dps sensor counts; the same
  // run put the raw-to-rf2o ratio at 14.4 (and ~16 against AMCL's heading).
  declare_parameter<double>("gyro_scale", 1.0 / 16.4);
  // Yaw-rate std dev, rad/s. Parked bias measured ~0.006 rad/s; 0.03 leaves
  // room for that and for gyro_scale being a few percent off.
  declare_parameter<double>("gyro_sigma", 0.03);
  odom_k_x = get_parameter("k_x").as_double();
  odom_k_y = get_parameter("k_y").as_double();
  odom_slip_frac_x = get_parameter("slip_frac_x").as_double();
  odom_slip_frac_y = get_parameter("slip_frac_y").as_double();
  odom_sigma_floor = get_parameter("sigma_floor").as_double();
  odom_gyro_z_sign = get_parameter("gyro_z_sign").as_double();
  odom_gyro_sigma = get_parameter("gyro_sigma").as_double();
  gyro_scale = get_parameter("gyro_scale").as_double();
  param.vid = hex_string(get_parameter("vid").as_string());
  param.pid = hex_string(get_parameter("pid").as_string());
  param.baud = get_parameter("baud").as_int();
  open_device();
  // Halt all motors at startup (NEUTRAL)
  halt();
  // Initialize topics
  RCLCPP_INFO(get_logger(), "Initializing ROS2 topics");
  halt_signal.subscribe(this, "base/halt", 10, [this](auto msg) {
    halted = msg.data;
    if (halted)
      halt();
  });
  velocity_io.subscribe(this, "base/velocity/set");
  velocity_io.publish(this, "base/velocity/get");
  imu_out.publish(this, "base/imu");
  odom_out.publish(this, "base/odometry");
  odom_out.header.frame_id = "odom";
  odom_out.child_frame_id = "base_footprint";
  odom_out.header.stamp = rclcpp::now();
  // The pose here is open-loop dead reckoning with unbounded error and is
  // never fused (see config/ekf_fused_odom.yaml, which takes twist only).
  // Say so: an all-zero covariance means "known exactly" to REP-103
  // consumers, which is the opposite of the truth.
  for (int i = 0; i < 6; i++)
    odom_out.pose.covariance[i * 7] = 1e6;
  // Of the twist, vx and vy come from the command and yaw rate from the
  // gyro. vz/roll/pitch rates are not measured on this base.
  for (int i = 2; i < 5; i++)
    odom_out.twist.covariance[i * 7] = 1e6;
  odom_out.twist.covariance[35] = odom_gyro_sigma * odom_gyro_sigma;
  // Timer loop for serial I/O
  timers.push_back(create_wall_timer(50ms, [this]() {
    if (imu_status.query())
      device->query<MSP::RAW_IMU>();
    else
      device->query<MSP::ATTITUDE>();
  }));
  timers.push_back(
      create_wall_timer(2000ms, [this]() { device->query<MSP::ANALOG>(); }));
  // Debounced velocity command
  timers.push_back(create_wall_timer(10ms, [this]() {
    if (velocity_io.updated) {
      motion(velocity_io.linear.x, velocity_io.linear.y, velocity_io.angular.z);
      velocity_io();
      velocity_io.updated = false;
    } else {
      using clock = std::chrono::system_clock;
      const auto time_since_last_cmd = clock::now() - velocity_io.timestamp;
      if (time_since_last_cmd > vel_timeout) {
        halt();
      }
    }
    // Odometry publish, ~50 Hz (every other tick of this 10 ms timer).
    //
    // Deliberately NOT the commented-out odom_out() calls in motion() and
    // halt(): halt() is re-entered every 10 ms once the watchdog above
    // expires, so publishing from inside it would emit 100 Hz odometry for
    // as long as the rover sits idle. Driving it from the timer covers the
    // moving and the halted case at one predictable rate.
    if (++odom_publish_tick >= 2) {
      odom_publish_tick = 0;
      integrate_odometry();
      set_twist_covariance();
      odom_out();
    }
  }));
}

BaseDriver::~BaseDriver() {
  // RCLCPP_INFO(get_logger(), "Stopping all motors");
  halt();
  // RCLCPP_INFO(get_logger(), "Resetting device");
  device->query<MSP::REBOOT>();
  std::this_thread::sleep_for(2000ms);
  device = nullptr;
  // RCLCPP_INFO(get_logger(), "Exiting ...");
}

bool BaseDriver::communicate() {
  const auto raw_packet = device->read();
  if (raw_packet == nullptr)
    return false;
  try {
    if (raw_packet->is<MSP::RAW_IMU>())
      update(raw_packet->as<MSP::RAW_IMU>(), raw_packet->timestamp);
    else if (raw_packet->is<MSP::ATTITUDE>())
      update(raw_packet->as<MSP::ATTITUDE>(), raw_packet->timestamp);
    else if (raw_packet->is<MSP::ANALOG>()) {
      const auto &msg = raw_packet->as<MSP::ANALOG>();
      double volt = static_cast<double>(msg.vbat) * 0.01;
      RCLCPP_INFO(get_logger(), "Battery voltage: %.2f V", volt);
    }
  } catch (const std::exception &e) {
    RCLCPP_ERROR(get_logger(), "MSP Error: %s", e.what());
  }
  return true;
}
