#include "Message.h"
#include "Types.h"

#include "MultiWii/protocol.h"

class BaseDriver : public Node {
private:
  std::vector<Timer::SharedPtr> timers;
  Message<Bool> halt_signal;
  Message<Twist> velocity_io;
  Message<Odometry> odom_out;
  Message<Imu> imu_out;
  // Configurable parameters
  struct {
    std::string vid, pid;
    int baud;
  } param;
  void open_device(unsigned max_attempts = 10, bool log = true);
  // MSP inbound receiver
  MultiWii::Device::Ptr device = nullptr;
  // Timeout since last velocity command to halt motors
  const std::chrono::milliseconds vel_timeout = 2000ms;
  // Halt flag
  bool halted = false;
  void halt();
  void motion(double vx, double vy, double vr);
  struct {
    uint8_t counter = 0;
    bool acc_updated = false;
    bool att_updated = false;
    inline bool query() {
      counter = (counter + 1) % 2;
      return counter == 0;
    }
    inline bool updated() { return acc_updated && att_updated; }
    inline void reset() { acc_updated = att_updated = false; }
  } imu_status;
  void update(MSP::RAW_IMU data, time_point timestamp);
  void update(MSP::ATTITUDE data, time_point timestamp);

  // Throttle [-1,1] -> m/s normalization, one factor per axis. A mecanum
  // base loses far more to roller slip when strafing than when driving
  // forward, so a single gain cannot describe both.
  double odom_k_x, odom_k_y;
  // Twist covariance model: var = (slip_frac * v)^2 + sigma_floor^2.
  // At rest this collapses to sigma_floor^2, which is the zero-velocity
  // update -- the strongest statement available on a base with no encoders.
  double odom_slip_frac_x, odom_slip_frac_y, odom_sigma_floor;
  // Gyro yaw rate as published in odom twist.angular.z. The sign maps the
  // flight controller's gyro Z onto REP-103 (CCW positive) and is
  // UNVERIFIED -- scripts/odom_diagnose.py measures it against rf2o.
  double odom_gyro_z_sign, odom_gyro_sigma;
  // deg/s per raw MSP gyro unit (see the gyro_scale parameter).
  double gyro_scale;
  void set_twist_covariance();
  // Decimates the 10 ms command timer down to the odometry publish rate.
  uint8_t odom_publish_tick = 0;
  double odom_initial_heading = NAN;
  double odom_last_heading = NAN;
  void integrate_odometry();
  void integrate_odometry(double current_heading);

public:
  BaseDriver();
  ~BaseDriver();
  // Handle packets sent from serial device
  // Handles one queued MSP packet; returns false when the queue is empty.
  bool communicate();
};
