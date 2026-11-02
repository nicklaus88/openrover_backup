# Everything robot_controller_node actually reasoned through to produce
# one velocity command -- published once per command, alongside it. This
# is the "why" that only ever existed as logger.warn() text before: none
# of it was structured/bag-able. Meant to be recorded (see
# record_bag.launch.py) and later turned into language descriptions of
# what the robot was doing and why, alongside detections/tracks/target
# for the bounding-box/state side of that.

std_msgs/Header header

# Mode this decision was made under -- mirrors FollowTarget.state, but
# captured at the moment robot_controller_node acted on it (e.g. still
# says "TRACKING" for the frame where the target track just vanished and
# this decision was to stop).
string mode

# The person-following command computed from the target's bearing/
# distance, BEFORE any obstacle-avoidance nudge. Meaningless (0,0,0) when
# mode isn't TRACKING.
#
# This base is HOLONOMIC: base_vx/base_vy together are a straight-line
# translation at the target's standoff point (the robot does not have to
# be facing the target to close distance), and base_va is a separate,
# deadbanded "keep them in frame" yaw term that is 0 whenever the target
# is already comfortably centered. On the differential-drive version
# these were coupled -- vx was gated on being roughly aligned, and every
# lateral correction had to be spent as rotation. See
# robot_controller_node.py's module docstring.
#
# All three are NORMALIZED throttle in [-1, 1] (what
# /rover/base/velocity/set expects), not m/s and rad/s.
float32 base_vx
float32 base_vy
float32 base_va

# Obstacle avoidance (see robot_controller_node.py's
# _apply_obstacle_avoidance docstring). obstacle_min_forward_m/
# free_space_*_m are NaN and obstacle_direction/severity are 0 when
# obstacle_detected is false.
bool obstacle_detected
float32 obstacle_min_forward_m
# Continuous in [-1, 1] (positive = towards the robot's left), a
# low-pass-filtered blend of free-space and target-bearing preference --
# NOT a discrete +1.0/-1.0 left-or-right choice. See
# robot_controller_node.py's module docstring for why (a discrete version
# kept oscillating).
#
# On this omnidirectional base this is a STRAFE bias applied to vy, not a
# steer bias applied to va: the rover sidesteps an obstacle while still
# facing the person, rather than turning away from it. Field name kept
# for bag compatibility with the differential-drive recordings.
float32 obstacle_direction
float32 obstacle_severity         # 0.0 (at threshold) .. 1.0 (at/past critical distance)
float32 free_space_left_m
float32 free_space_right_m

# Search-by-rotating (REACQUIRE only). Empty/0 outside REACQUIRE.
string search_phase               # "rotate", "pause", or "" if not searching
float32 search_direction          # +1.0 left, -1.0 right, 0.0 if not searching

# The command actually published to the base this cycle, after the
# obstacle nudge and clamping. Normalized [-1, 1], same as base_* above.
float32 final_vx
float32 final_vy
float32 final_va
