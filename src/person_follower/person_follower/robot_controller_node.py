"""robot_controller_node: FollowTarget + TrackedPersonArray(+ LaserScan)
-> /rover/base/velocity/set.

Adapted from the differential-drive (TurtleBot4) version of this node for
the RoverMaster omnidirectional base. The person-following intent is
unchanged -- hold a fixed standoff distance from an imprinted target,
avoid what is in the way, sweep to reacquire when lost -- but every place
the old version had to spend ROTATION to correct a LATERAL error now
strafes instead. See OMNIDIRECTIONAL CONTROL below for what that changed
and why it also retired a whole class of oscillation bug.

UNITS. This node's velocity outputs are NORMALIZED THROTTLE in [-1, 1],
not m/s and rad/s. rover/BaseDriver.cpp mixes linear.x/linear.y/angular.z
straight into its four wheel commands and clamps each axis to +-1, so
max_vx/max_vy/max_va and every gain here are fractions of full motor
authority. (For a rough physical sense: BaseDriver's k_x is 1.28, so
vx=0.3 is roughly 0.38 m/s.) Do NOT port gains over from the
differential-drive config expecting them to mean the same thing -- there,
max_va=1.5 meant 1.5 rad/s; here it would mean 150% of full authority,
i.e. permanently clamped flat out.

OMNIDIRECTIONAL CONTROL

  Translation is holonomic. The old version computed a bearing to the
  target, turned to face it (proportional on that bearing), and only
  drove forward once roughly aligned (`abs(theta) < max_bearing`).
  _drive_toward_target now drives the base straight at the standoff
  point in a single step -- vx and vy together -- with no alignment gate
  at all, because there is nothing to align: the base translates in any
  direction regardless of which way it happens to be pointing.

  Yaw is now ONLY about keeping the person in frame. The camera is
  body-mounted with a limited horizontal FOV -- ~64.9 deg total, ~32.4
  deg half, on the 512x288 16:9 preview this pipeline consumes. So the
  robot still has to face roughly towards the target or it stops being
  able to see them, but that is the only remaining reason to rotate
  while following. Every angular threshold below is implicitly relative
  to that half-FOV; when the preview was briefly a 250-square crop of
  the same sensor it was 19.6 deg, and thresholds sized for 32 quietly
  became thresholds outside the frame. See the HISTORY block in
  config/person_follower.yaml. `yaw_deadband_deg` makes that explicit: while the target
  sits within that half-angle of center, va is exactly zero and every
  lateral correction is spent as strafe; past it, a proportional term
  turns just enough to bring them back inside. Small corrections are
  pure sideways motion; the robot only rotates when framing is genuinely
  at risk.

  Those two mechanisms spend the SAME error, which makes the deadband a
  balance knob rather than just a threshold: yaw drives the bearing down
  to the edge of it, and the strafe term is proportional to the sine of
  that same bearing, so wherever the deadband parks the target decides
  how much sideways motion is left over. A deadband of 6 deg frames
  beautifully and strafes almost not at all; 18 deg strafes hard and
  loses the person out of frame. config/person_follower.yaml ships three
  measured profiles along that axis (follow_style: strafe / balanced /
  yaw) -- see FOLLOW STYLE there.

  Obstacle avoidance strafes rather than steers. The avoidance signal is
  the same continuous, smoothed [-1, 1] bias described under INHERITED
  BEHAVIOR, but it is added to vy, not va -- the rover sidesteps what is
  in the way while continuing to face the person.

  That last change is worth spelling out, because it retires a bug the
  differential-drive version fought at length. There, avoidance and
  framing both wrote to the SAME axis (va): turning away from an obstacle
  swung the target's apparent bearing, the centering term immediately
  turned back to cancel that swing, and the two could null each other out
  entirely (reported directly: "avoiding left, but the robot turns right
  instead to re-center the person"). `obstacle_frame_deadband_deg` -- a
  severity-ramped loosening of centering -- existed purely to stop those
  two fighting. Here they sit on orthogonal axes: avoidance owns vy,
  framing owns va, and neither can cancel the other. That parameter is
  kept, but it is now doing a much smaller job (see its param comment)
  rather than holding the design together.

  Strafing does still move the target's bearing a little -- move left and
  they appear further right -- so the coupling is not exactly zero. It is
  just weak, and unlike the rotation case it is not self-reinforcing.

  Search-by-rotating (REACQUIRE) deliberately still ROTATES rather than
  strafing. Its job is to sweep the camera over ground it cannot
  currently see; rotating sweeps the FOV across the scene, strafing
  mostly just slides the same view sideways.

INHERITED BEHAVIOR (unchanged from the differential-drive version apart
from which axis the output lands on)

  Obstacle avoidance (while TRACKING): two independently-sized, allowed-
  to-overlap LIDAR zones are checked before every drive command (see
  _read_obstacle) -- a center trigger zone (`obstacle_trigger_cone_deg`)
  decides IF this activates at all, two side zones
  (`obstacle_side_cone_deg`, reaching in from the edges of
  `obstacle_forward_cone_deg`) decide WHICH WAY to go. Inside
  `obstacle_distance_threshold`, a strafe-away nudge blends into the
  normal person-following translation (see _apply_obstacle_avoidance) --
  weighted (`obstacle_avoid_frame_bias`) between "towards more open
  space" and "towards wherever the target actually is".

  That nudge is a CONTINUOUS value in [-1, 1] (how far left/right, not a
  binary left-or-right choice), smoothed frame to frame with a low-pass
  filter (`obstacle_steer_smoothing`) -- not, as a first attempt had it,
  a discrete +1/-1 decision defended with hysteresis/hold-time/debounce
  logic. That discrete version kept oscillating no matter how much
  hysteresis got piled on, because hysteresis only slows down how fast a
  bang-bang controller can flip -- it does not stop the flipping when the
  underlying signal genuinely sits near the decision boundary for an
  extended time (a doorway, a target that is roughly centered). Confirmed
  directly on the differential-drive robot: with
  `obstacle_avoid_frame_bias` tuned high, free space consistently favored
  one side by a wide margin (e.g. L=1.8m vs R=0.57m) yet the discrete
  decision still flipped every ~1s in lockstep with its hold time,
  because the target's bearing (which the robot's own avoidance turning
  was itself swinging back and forth) kept winning the winner-take-all
  vote against a strong, stable free-space preference. A continuous,
  smoothed signal has no winner-take-all vote to flip -- conflicting
  inputs blend and damp out instead of alternating.

  Trade-off that came with fixing that: a blended, smoothed signal can
  legitimately settle near zero -- a near-symmetric obstacle, or
  `obstacle_avoid_frame_bias` weighted towards a target bearing that is
  itself near zero because the person is roughly straight ahead -- and a
  near-zero nudge combined with forward speed simultaneously being cut by
  severity left the robot just braking in place instead of routing around
  what was ahead (reported directly: "not avoiding the obstacle and stuck
  there"). Fixed with a magnitude floor (`obstacle_min_steer_bias`, only
  once `obstacle_min_steer_severity` is reached) applied in
  _apply_obstacle_avoidance on top of the sign the continuous blend
  already computed -- it raises how hard the robot commits to a direction
  once it is genuinely close to something, without touching the sign
  itself, so it cannot reintroduce oscillation.

  Search-by-rotating (while REACQUIRE only -- not the initial SEARCH
  state, which just waits still for someone to imprint): once a
  previously-locked target is lost and person_follower_node moves to
  REACQUIRE, this rotates in short bursts towards wherever the target was
  last seen (their last computed bearing while still TRACKING), pausing
  between each burst. Continuous rotation was tried first and does not
  work: reacquire_positive_count needs the person to stay still in-frame
  for several consecutive frames, which never happens if the camera
  itself keeps sweeping past them. The pause gives that window.

A watchdog timer publishes a zero Twist if no fresh drive/search command
has gone out for `timeout` seconds. Note the base has its own independent
2s failsafe (BaseDriver's vel_timeout) that halts the motors if it stops
hearing commands entirely -- this watchdog is the faster, finer-grained
one, and the one that matters for "target lost, stop now".

Every velocity command this node publishes is accompanied by a
RobotDecision message on ~/decision -- the full "why" (base person-
following command before any nudge, obstacle-avoidance blend inputs/
outputs, search phase/direction) as structured, bag-able data instead of
only ever existing as logger.warn() text. See RobotDecision.msg.

LIDAR mounting offset: a scan's own angle=0 means "straight ahead of the
LIDAR", not "straight ahead of the robot" -- those are only the same
thing if the sensor happens to be mounted with zero yaw relative to the
base frame, which is not a safe assumption (it was not true on the
differential-drive robot, whose lidar sat yawed +90 deg). Getting this
wrong makes _read_obstacle check the wrong direction entirely, and on an
omni base that now means strafing the wrong way as well. Rather than
hardcode a robot-specific offset, this looks up the real yaw between the
scan's frame and base_frame via tf2 once (it is a static mount) and
caches it. On this rover that lookup is base_footprint -> laser, which
robot_state_publisher resolves from description/open_rover.urdf.xacro
(started by localization.py, mapping.py or perception_oakd.py).
"""
import math

import rclpy
import tf2_ros
from geometry_msgs.msg import Twist
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from rclpy.time import Time
from sensor_msgs.msg import LaserScan

from person_follower_msgs.msg import FollowTarget, RobotDecision, TrackedPersonArray


def _yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def _clamp(value, low, high):
    return max(low, min(high, value))


class RobotControllerNode(Node):

    def __init__(self):
        super().__init__('robot_controller_node')

        # NOTE: every velocity and gain below is NORMALIZED THROTTLE in
        # [-1, 1], not m/s or rad/s -- see the module docstring's UNITS
        # section. `distance` is the one exception: it is a real distance
        # in meters, because it is compared against the tracker's 3D
        # position rather than sent to the base.

        # Back off when the target is closer than `distance`. Natural on
        # an omni base -- it just translates backwards, no three-point
        # turn -- so this defaults ON here, where the differential-drive
        # version left it off.
        self.declare_parameter('enable_back', True)
        # Forward/back and left/right translation ceilings. Kept as two
        # separate limits rather than one speed cap because the base is
        # not isotropic in practice: sideways is both the axis with less
        # usable thrust out of BaseDriver's wheel mixing and the axis
        # with the least sensor coverage (the lidar trigger cone is
        # centered forward and the camera only looks forward), so it is
        # worth being able to hold strafe below forward speed.
        self.declare_parameter('max_vx', 0.35)
        self.declare_parameter('max_vy', 0.30)
        # Yaw ceiling. On this base yaw exists only to keep the target in
        # frame while TRACKING, plus the REACQUIRE sweep -- it is NOT how
        # lateral error gets closed any more, strafe is. Much lower than
        # the differential-drive value for that reason (and because that
        # value was rad/s, not throttle -- see UNITS).
        self.declare_parameter('max_va', 0.50)
        self.declare_parameter('gain_vx', 0.5)
        self.declare_parameter('gain_vy', 0.5)
        self.declare_parameter('gain_va', 1.1)
        # Standoff distance to hold from the target, in METERS.
        self.declare_parameter('distance', 1.0)
        # Yaw deadband -- the core of the omnidirectional change. While
        # the target's bearing sits inside this half-angle the robot does
        # not rotate AT ALL and lateral error is corrected purely by
        # strafing; past it, a proportional term turns just enough to
        # bring them back inside. Keep it comfortably inside the camera's
        # real half-FOV -- ~32.4 deg on the 512x288 preview this pipeline
        # consumes (see the module docstring) -- so re-framing starts
        # well before the target reaches the edge of the image, where
        # keypoint detection gets unreliable anyway.
        # Set this to 0.0 to get continuous centering (always turning a
        # little); set it above max_bearing_deg to never turn while
        # TRACKING at all.
        self.declare_parameter('yaw_deadband_deg', 12.0)
        # Purely a LABEL for the profile that yaw_deadband_deg, gain_va,
        # max_va and obstacle_frame_deadband_deg came from -- nothing
        # reads it as a control input. It exists so the overlay in
        # config/follow_style_<name>.yaml can name itself, so the startup
        # log says which balance is running, and so a bag recorded for a
        # training set traces back to a behavior rather than to four
        # loose numbers. See FOLLOW STYLE in config/person_follower.yaml.
        self.declare_parameter('follow_style', 'balanced')
        # Beyond this bearing the target is close enough to leaving frame
        # that re-framing takes priority: translation is suppressed and
        # the robot turns in place until they are back inside. On the
        # differential-drive version this gated forward drive on being
        # roughly ALIGNED (you cannot usefully drive at something you are
        # not pointing at); here alignment is irrelevant to driving, so
        # it is purely a "stop and re-frame" guard.
        self.declare_parameter('max_bearing_deg', 30.0)
        self.declare_parameter('timeout', 0.5)
        self.declare_parameter('watchdog_rate', 10.0)

        # -- obstacle avoidance --
        # Everything on this rover speaks base_footprint -- there is no
        # base_link in its tf tree at all (see launch/localization.py,
        # which notes the same thing about rf2o's upstream default).
        self.declare_parameter('base_frame', 'base_footprint')
        # Overall forward FOV considered (total width, centered on
        # straight ahead) -- bounds where the two side free-space zones
        # (obstacle_side_cone_deg) can reach out to. See _read_obstacle's
        # docstring for how this and the next two params fit together.
        self.declare_parameter('obstacle_forward_cone_deg', 70.0)
        # Center zone (total width) that decides IF avoidance triggers at
        # all -- "is anything actually blocking the path ahead". Wider
        # than a plain 1/3-split gives earlier, more gradual reactions
        # instead of only noticing an obstacle once it's nearly dead
        # ahead and close.
        self.declare_parameter('obstacle_trigger_cone_deg', 110.0)
        # Width of EACH side zone (left and right), measured in from the
        # outer edge of obstacle_forward_cone_deg -- decides WHICH WAY to
        # steer once triggered. Deliberately allowed to overlap with the
        # trigger zone above (a wider trigger zone or narrower total FOV
        # makes that overlap bigger) -- this is intentional, not a bug:
        # the two zones answer different questions ("should I react" vs
        # "which way"), and there's no reason a single LIDAR point can't
        # inform both.
        self.declare_parameter('obstacle_side_cone_deg', 60.0)
        # Inside this range, steering starts blending in a turn away
        # from the obstacle and forward speed starts scaling down --
        # both proportional to how deep into this range the obstacle is
        # (0% right at the threshold, 100% right at
        # obstacle_critical_distance). This is an added nudge on top of
        # the normal person-following command, not a replacement for it,
        # so the robot still turns/drives towards the person while
        # steering around what's in the way.
        self.declare_parameter('obstacle_distance_threshold', 0.6)
        # Inside this range, forward motion is cut completely (hard
        # safety floor) regardless of blending -- only steering away
        # continues.
        self.declare_parameter('obstacle_critical_distance', 0.3)
        # Angular speed (rad/s) contribution from the avoidance nudge at
        # full severity (i.e. at/inside obstacle_critical_distance).
        self.declare_parameter('obstacle_avoid_gain', 0.6)
        # 0.0 = steer nudge always towards whichever side has more open
        # space, ignoring the target; 1.0 = always towards whichever
        # side the target is on, ignoring free space. Default leans
        # towards free space (safety) with some pull towards the target.
        # This is the weight used when the target is comfortably in
        # frame -- see obstacle_frame_bias_max for what it rises to as
        # that stops being true.
        self.declare_parameter('obstacle_avoid_frame_bias', 0.3)
        # Ceiling the blend weight above rises towards as the target's
        # actual bearing approaches obstacle_frame_deadband_deg (i.e. as
        # they get closer to actually being at risk of leaving frame).
        # 1.0 = once genuinely at risk, ignore free space entirely and
        # follow the target -- losing them is worse than a rougher path
        # around the obstacle. See _read_obstacle's edge_urgency comment.
        self.declare_parameter('obstacle_frame_bias_max', 1.0)
        # Free-space and target-bearing inputs are each squashed through
        # tanh(value/scale) into a continuous [-1,1] contribution before
        # blending (see _read_obstacle) -- these are the "how much
        # difference/angle counts as fully committing to one side" scales.
        # A free_left-free_right difference of one full scale is ~0.76 of
        # the way to fully committed; well past 2x scale is ~fully
        # committed. Smaller scale = more sensitive to small
        # differences/angles.
        self.declare_parameter('obstacle_free_space_scale_m', 1.0)
        self.declare_parameter('obstacle_frame_angle_scale_deg', 30.0)
        # Low-pass filter on the final blended steer bias: each new
        # reading only pulls the output this fraction of the way towards
        # itself. Lower = smoother/slower to react, higher = snappier but
        # noisier. This -- not a discrete decision defended with
        # hysteresis -- is what actually stops oscillation near an
        # obstacle: see the module docstring for why the discrete-plus-
        # hysteresis version kept flipping no matter how much hysteresis
        # got piled onto it.
        self.declare_parameter('obstacle_steer_smoothing', 0.3)
        # Floor on how small a turn the avoidance nudge is allowed to
        # apply once it's actually committed to reacting (see
        # obstacle_min_steer_severity below) -- without this, a blended
        # steer_bias that's genuinely near zero (a near-symmetric
        # obstacle with free_left~=free_right, or obstacle_avoid_frame_bias
        # weighted heavily towards a target bearing that's itself near
        # zero because the person is roughly straight ahead) asymptotes
        # towards a turn too weak to actually change course, while
        # forward speed is simultaneously being scaled down by severity --
        # net result is the robot just brakes in place in front of the
        # obstacle instead of routing around it (reported directly:
        # "not avoiding the obstacle and stuck there"). This only sets a
        # MAGNITUDE floor, using the sign of whatever real (possibly
        # tiny) signal already exists -- or a fixed default direction if
        # that signal is genuinely exactly zero -- so it can't reintroduce
        # oscillation: the sign only changes when the real, filtered
        # signal's sign changes, same as without the floor.
        self.declare_parameter('obstacle_min_steer_bias', 0.35)
        # Severity (see _apply_obstacle_avoidance) has to reach at least
        # this before the floor above kicks in -- a faint, distant brush
        # against obstacle_distance_threshold shouldn't force a
        # committed turn, only a genuinely close obstacle should.
        self.declare_parameter('obstacle_min_steer_severity', 0.3)
        # The floor above needs a DIRECTION to commit to, not just a
        # magnitude -- and a near-symmetric reading's true (tiny) sign is
        # mostly scan noise, so using it directly would make the floor
        # flip sides on pure noise every frame (the exact oscillation
        # this whole redesign was meant to remove). Instead the
        # remembered "committed sign" only updates when the blended
        # signal is at least this confidently non-zero; while it's below
        # this, the floor reuses whatever direction was last confidently
        # picked instead of trusting the noisy near-zero sign.
        self.declare_parameter('obstacle_sign_confidence', 0.1)
        # While avoiding an obstacle, how far off-center (degrees) the
        # target is still allowed to sit before the person-following turn
        # starts correcting for it -- "keep them in frame", not "keep
        # them dead center". Without this, the plain proportional
        # centering term (gain_va * theta) fights the avoidance steer
        # nudge on every cycle: turning away from an obstacle shifts the
        # target's apparent bearing, and centering immediately tries to
        # turn back to cancel that shift, which can cancel the avoidance
        # turn altogether (reported directly: avoiding left, but the
        # robot turns right instead to re-center the person). Must stay
        # comfortably inside both max_bearing_deg and the camera's actual
        # half-FOV (~32.4 deg on the preview) -- this loosens centering,
        # it doesn't grow how far off bearing the target can get before
        # being lost from view. Set larger than the half-FOV it silently
        # disables framing altogether while avoiding, which loses the
        # target exactly when maneuvering needs them most. Ramps
        # in with obstacle severity (see _obstacle_severity) rather than
        # switching on abruptly at the trigger threshold, and has zero
        # effect at all when no obstacle is being avoided -- normal
        # tracking still centers tightly.
        self.declare_parameter('obstacle_frame_deadband_deg', 15.0)
        # How close something may be on a side before strafing INTO that
        # side is refused outright (see _limit_strafe). This has no
        # differential-drive equivalent and is not a tuning knob so much
        # as a hard guard: that version could only ever move along its
        # own x axis, so "the side of the robot" was never a surface it
        # could drive into, and the forward trigger cone was a complete
        # safety story. It is not one here. Compared against the CLOSEST
        # return in each side zone, not the average.
        self.declare_parameter('obstacle_side_clearance', 0.35)
        # Ignore scan returns closer than this. NOT a tuning knob -- this
        # rover's lidar sees its own ARM, and without a floor here the
        # robot believes it is permanently boxed in.
        #
        # Measured on this robot and documented in two other places
        # already: config/scan_filter.yaml found 1357 of 34914 valid
        # beams (3.9%) returning under 0.35m, clustered at 330-360deg,
        # the farthest of them at 0.177m; the nearest REAL geometry ever
        # seen was 0.35m. config/amcl_params.yaml disposes of the same
        # population with laser_min_range: 0.25, and scan_filter.yaml
        # with lower_threshold: 0.25. This is the same 0.25, for the same
        # reason, because this node subscribes to the RAW /scan -- the
        # filtered /scan_rf2o exists only to feed rf2o.
        #
        # Getting this wrong is not subtle: those returns sit well inside
        # both obstacle_critical_distance and obstacle_side_clearance and
        # never change with motion (they are bolted to the robot), so
        # they would pin min_forward and one side's min_* permanently
        # below threshold -- forward speed cut to zero, strafe to that
        # side refused, forever, with nothing actually in the way.
        self.declare_parameter('scan_min_range', 0.25)

        # -- search-by-rotating while REACQUIRE (not initial SEARCH) --
        self.declare_parameter('search_angular_speed', 0.3)
        # How long each rotate burst lasts, and how long to hold still
        # between bursts. The pause needs to comfortably cover
        # reacquire_positive_count frames at the camera's frame rate
        # (see config/person_follower.yaml) so re-id actually has a
        # stable window to lock back on.
        self.declare_parameter('search_rotate_burst_duration', 0.6)
        self.declare_parameter('search_pause_duration', 2.0)
        self.declare_parameter('search_rate', 10.0)

        self.enable_back = bool(self.get_parameter('enable_back').value)
        self.max_vx = float(self.get_parameter('max_vx').value)
        self.max_vy = float(self.get_parameter('max_vy').value)
        self.max_va = float(self.get_parameter('max_va').value)
        self.gain_vx = float(self.get_parameter('gain_vx').value)
        self.gain_vy = float(self.get_parameter('gain_vy').value)
        self.gain_va = float(self.get_parameter('gain_va').value)
        self.distance = float(self.get_parameter('distance').value)
        self.follow_style = str(self.get_parameter('follow_style').value)
        self.yaw_deadband = math.radians(float(self.get_parameter('yaw_deadband_deg').value))
        self.max_bearing = math.radians(float(self.get_parameter('max_bearing_deg').value))
        self.timeout = float(self.get_parameter('timeout').value)

        self.base_frame = self.get_parameter('base_frame').value
        self.obstacle_forward_cone = math.radians(
            float(self.get_parameter('obstacle_forward_cone_deg').value))
        self.obstacle_trigger_cone = math.radians(
            float(self.get_parameter('obstacle_trigger_cone_deg').value))
        self.obstacle_side_cone = math.radians(
            float(self.get_parameter('obstacle_side_cone_deg').value))
        self.obstacle_distance_threshold = float(
            self.get_parameter('obstacle_distance_threshold').value)
        self.obstacle_critical_distance = float(
            self.get_parameter('obstacle_critical_distance').value)
        self.obstacle_avoid_gain = float(self.get_parameter('obstacle_avoid_gain').value)
        self.obstacle_avoid_frame_bias = float(
            self.get_parameter('obstacle_avoid_frame_bias').value)
        self.obstacle_frame_bias_max = float(
            self.get_parameter('obstacle_frame_bias_max').value)
        self.obstacle_free_space_scale = float(
            self.get_parameter('obstacle_free_space_scale_m').value)
        self.obstacle_frame_angle_scale = math.radians(
            float(self.get_parameter('obstacle_frame_angle_scale_deg').value))
        self.obstacle_steer_smoothing = float(
            self.get_parameter('obstacle_steer_smoothing').value)
        self.obstacle_min_steer_bias = float(
            self.get_parameter('obstacle_min_steer_bias').value)
        self.obstacle_min_steer_severity = float(
            self.get_parameter('obstacle_min_steer_severity').value)
        self.obstacle_sign_confidence = float(
            self.get_parameter('obstacle_sign_confidence').value)
        self.obstacle_frame_deadband = math.radians(
            float(self.get_parameter('obstacle_frame_deadband_deg').value))
        self.obstacle_side_clearance = float(
            self.get_parameter('obstacle_side_clearance').value)
        self.scan_min_range = float(self.get_parameter('scan_min_range').value)
        self.search_angular_speed = float(self.get_parameter('search_angular_speed').value)
        self.search_rotate_burst_duration = float(
            self.get_parameter('search_rotate_burst_duration').value)
        self.search_pause_duration = float(self.get_parameter('search_pause_duration').value)

        self.target_id = -1
        self.target_state = 'SEARCH'
        self.last_drive_time = self.get_clock().now()
        self.last_scan = None
        # Last bearing (radians, atan2 convention: +ve = target to the
        # left) computed while actually TRACKING -- used both to bias
        # obstacle-avoidance turn direction and to pick which way to
        # rotate first when re-searching during REACQUIRE.
        self.last_known_theta = 0.0
        # Low-pass-filtered obstacle-avoidance steer bias, continuous in
        # [-1, 1] -- see _read_obstacle. Reset to 0.0 whenever the cone is
        # clear so a later, unrelated encounter starts fresh rather than
        # inheriting a stale value.
        self.obstacle_steer_bias = 0.0
        # Sign memory for the obstacle_min_steer_bias floor (see
        # _apply_obstacle_avoidance) -- only updated when obstacle_steer_bias
        # is confidently non-zero (obstacle_sign_confidence), so tiny
        # frame-to-frame noise in a near-symmetric reading can't flip
        # which way the floor commits to. Deliberately NOT reset when the
        # cone clears (unlike obstacle_steer_bias) -- persisting it means
        # a later near-symmetric encounter reuses the last direction that
        # actually worked, rather than picking a fresh arbitrary default
        # every time.
        self.obstacle_last_committed_sign = 1.0
        # Rotate-burst/pause phase for the search behavior, see
        # _search_timer_cb.
        self.search_phase = 'rotate'
        self.search_phase_start = self.get_clock().now()
        # Yaw (radians) between the LIDAR's own frame and base_frame --
        # None until the first successful tf lookup, see
        # _get_lidar_yaw_offset. Cached after that since it's a static
        # mount; a bad/late tf lookup on startup just means a couple of
        # early scans get skipped, not a wrong offset baked in forever.
        self.lidar_yaw_offset = None

        # The yaw/strafe balance is the thing most often wrong in a
        # recorded run and the thing least visible from the outside, so
        # state it up front rather than making someone diff two configs.
        self.get_logger().info(
            f"follow_style: '{self.follow_style}' -- "
            f'yaw_deadband {math.degrees(self.yaw_deadband):.1f} deg, '
            f'gain_va {self.gain_va}, max_va {self.max_va}, '
            f'obstacle_frame_deadband '
            f'{math.degrees(self.obstacle_frame_deadband):.1f} deg')
        if self.obstacle_frame_deadband <= self.yaw_deadband:
            self.get_logger().warn(
                'obstacle_frame_deadband_deg <= yaw_deadband_deg: the '
                'controller takes max() of the two, so loosening framing '
                'while avoiding obstacles will have NO effect.')

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # The rover base's own command topic, not the generic /cmd_vel
        # the differential-drive version used -- rover/BaseDriver.cpp
        # subscribes here (namespaced 'rover' by launch/rover.py), and
        # camera_detection/perception already publishes to this same name.
        # Still remappable; person_follower.launch.py exposes it as the
        # cmd_vel_topic argument.
        self.cmd_vel_pub = self.create_publisher(
            Twist, '/rover/base/velocity/set', 1)
        self.decision_pub = self.create_publisher(RobotDecision, '~/decision', 10)
        self.target_sub = self.create_subscription(
            FollowTarget, '~/target', self._target_callback, 10)
        self.tracks_sub = self.create_subscription(
            TrackedPersonArray, '~/tracks', self._tracks_callback, 10)
        self.scan_sub = self.create_subscription(
            LaserScan, '~/scan', self._scan_callback, qos_profile_sensor_data)

        watchdog_rate = float(self.get_parameter('watchdog_rate').value)
        self.watchdog_timer = self.create_timer(1.0 / watchdog_rate, self._watchdog)

        search_rate = float(self.get_parameter('search_rate').value)
        self.search_timer = self.create_timer(1.0 / search_rate, self._search_timer_cb)

    def _target_callback(self, msg):
        # Fresh entry into REACQUIRE -- start the rotate/pause cycle from
        # the beginning rather than picking up mid-phase from whatever
        # the last search left behind.
        if msg.state == 'REACQUIRE' and self.target_state != 'REACQUIRE':
            self.search_phase = 'rotate'
            self.search_phase_start = self.get_clock().now()

        self.target_id = msg.target_id
        self.target_state = msg.state

    def _scan_callback(self, msg):
        self.last_scan = msg

    def _tracks_callback(self, msg):
        if self.target_state == 'TRACKING' and self.target_id >= 0:
            track = next((t for t in msg.tracks if t.id == self.target_id), None)
            if track is not None and track.has_position_base_link:
                self._drive_toward_target(track)
                return

        if self.target_state == 'REACQUIRE':
            return  # driven by _search_timer_cb instead

        # SEARCH (waiting still for someone to imprint), or TRACKING with
        # no valid track this frame, or LOST.
        self._stop(self.target_state)

    def _drive_toward_target(self, track):
        x = track.position_base_link.x
        y = track.position_base_link.y
        theta = math.atan2(y, x)
        self.last_known_theta = theta

        # Read the scan once, up front -- the yaw deadband below,
        # _apply_obstacle_avoidance and the sideways guard all need it,
        # and they must agree on one reading rather than each re-deriving
        # from a scan that could have changed in between.
        zones = self._scan_zones()
        obstacle = self._read_obstacle(zones)
        severity = self._obstacle_severity(obstacle[0]) if obstacle is not None else 0.0

        # -- yaw: keep them in frame, and nothing else --
        #
        # Deadbanded by yaw_deadband: inside it the robot does not rotate
        # at all, because strafe below is what corrects lateral error now.
        # obstacle_frame_deadband widens that further while avoiding
        # something, exactly as it did on the differential-drive version
        # -- it just matters far less now that avoidance writes to vy
        # instead of competing for this same axis (module docstring).
        # Taking max() of the two rather than adding them keeps the
        # normal (no obstacle) deadband as the floor.
        deadband = max(self.yaw_deadband, self.obstacle_frame_deadband * severity)
        centering_theta = 0.0 if abs(theta) <= deadband \
            else theta - math.copysign(deadband, theta)
        base_va = _clamp(self.gain_va * centering_theta, -self.max_va, self.max_va)

        # -- translation: straight at the standoff point --
        #
        # The target is at (x, y); the point to actually occupy is
        # `distance` short of them along that same bearing. Scaling the
        # whole target vector by (1 - distance/range) is the error to
        # that point, on both axes at once -- so a target off to one side
        # produces a single diagonal move (forward AND sideways) instead
        # of the old turn-until-aligned-then-drive sequence.
        #
        # Deliberately a RADIAL approach: it closes range without trying
        # to circle around to sit dead ahead of the target. Someone
        # walking laterally gets followed by the robot cutting the corner
        # towards where they now are, which is what you want, rather than
        # orbiting to hold a bearing.
        base_vx = 0.0
        base_vy = 0.0
        if abs(theta) < self.max_bearing:
            rng = math.hypot(x, y)
            # Guard the division only. A range this small means the
            # tracker is reporting someone essentially on top of the
            # robot's own origin, which is not a real measurement.
            if rng > 1e-3:
                scale = 1.0 - self.distance / rng
                # enable_back gates CLOSING versus BACKING OFF, not the
                # x axis specifically: a negative scale means "too close,
                # retreat", along whichever direction the target actually
                # is, which on this base may be mostly sideways.
                if scale < 0.0 and not self.enable_back:
                    scale = 0.0
                base_vx, base_vy = self._limit_translation(
                    self.gain_vx * x * scale, self.gain_vy * y * scale)

        vx, vy, va, obstacle_info = self._apply_obstacle_avoidance(
            base_vx, base_vy, base_va, obstacle, severity, zones)

        self._publish('TRACKING', vx, vy, va,
                      base_vx=base_vx, base_vy=base_vy, base_va=base_va,
                      obstacle=obstacle_info)

    def _limit_translation(self, vx, vy):
        """Scale a translation vector down to fit inside max_vx/max_vy
        while PRESERVING ITS DIRECTION, instead of clamping each axis
        independently.

        Clamping the axes independently would quietly rotate the
        commanded heading whenever one of them saturated first -- which
        it routinely would, since max_vx and max_vy are deliberately
        different -- turning "drive straight at them" into "drive
        somewhat to one side of them", worst of all exactly when the
        robot is moving fastest. On a differential-drive base this
        failure mode does not exist (there is only one translation axis
        to clamp), which is why it needs saying here.
        """
        if self.max_vx <= 0.0:
            vx = 0.0
        if self.max_vy <= 0.0:
            vy = 0.0
        over = 1.0
        if vx and self.max_vx > 0.0:
            over = max(over, abs(vx) / self.max_vx)
        if vy and self.max_vy > 0.0:
            over = max(over, abs(vy) / self.max_vy)
        return vx / over, vy / over

    def _limit_strafe(self, vy, zones):
        """Refuse to strafe into a side that is already too close to
        something (obstacle_side_clearance).

        The forward trigger cone that drives all the avoidance logic
        below says nothing whatsoever about the robot's flanks. On the
        differential-drive version that was fine -- it could only
        translate along its own x axis, so its sides were never a
        surface it could drive into. This base strafes as a matter of
        course, both to follow the person and to route around obstacles,
        so its flanks are now a real collision surface with no coverage
        from the rest of this file.

        Checked against the CLOSEST return in the side zone rather than
        the average free_left/free_right used for direction preference:
        an average stays perfectly comfortable while one table leg sits
        centimeters from the wheels, and the average is answering a
        different question ("which way has more room to maneuver") than
        this one ("is it safe to move that way at all").
        """
        if zones is None or vy == 0.0:
            return vy
        _, _, _, min_left, min_right = zones
        blocked = (vy > 0.0 and min_left < self.obstacle_side_clearance) or \
                  (vy < 0.0 and min_right < self.obstacle_side_clearance)
        if not blocked:
            return vy
        self.get_logger().warn(
            f'strafe {"left" if vy > 0.0 else "right"} blocked -- '
            f'{min_left if vy > 0.0 else min_right:.2f}m clearance that side',
            throttle_duration_sec=1.0)
        return 0.0

    def _obstacle_severity(self, min_forward):
        """0.0 right at obstacle_distance_threshold, 1.0 at (or inside)
        obstacle_critical_distance. Shared between _apply_obstacle_avoidance
        and _drive_toward_target's frame-centering deadband so both agree
        on how close counts as "close"."""
        span = self.obstacle_distance_threshold - self.obstacle_critical_distance
        if span <= 0.0:
            return 1.0
        return _clamp((self.obstacle_distance_threshold - min_forward) / span, 0.0, 1.0)

    def _apply_obstacle_avoidance(self, vx, vy, va, obstacle, severity, zones):
        """Nudge the translation away from anything too close ahead
        without throwing away the person-following command that produced
        it -- turning the target's real bearing into a complete
        non-factor (route by free space only, approach not at all) made
        the robot look like it was actively moving away from the person
        while "avoiding". Instead: add a strafe-away term on top of the
        existing translation, and scale FORWARD speed down by how close
        the obstacle is (a hard, non-negotiable stop only once genuinely
        close -- obstacle_critical_distance).

        The nudge lands on vy (strafe), not va (turn). That is the whole
        omnidirectional point: the rover sidesteps the obstacle while
        still facing, and therefore still seeing, the person. On the
        differential-drive version this same term had to be spent as
        rotation, which swung the camera off the target and then fought
        the centering term for control of the single axis they shared.
        See the module docstring.

        Forward speed is still what gets cut, not strafe: the trigger
        cone that detected this looks FORWARD, so forward is the
        direction actually known to be blocked. Cutting vx while letting
        vy run is precisely the "sidestep, then carry on" behavior wanted
        here -- and _limit_strafe independently vetoes that sidestep if
        the flank it would move into is itself occupied.

        obstacle/severity/zones are what the caller already obtained from
        _scan_zones()/_read_obstacle()/_obstacle_severity() -- passed in
        rather than re-derived here, since the caller
        (_drive_toward_target) also needs them for the yaw deadband and
        the two must agree on one reading.

        Returns (vx, vy, va, obstacle_info) where obstacle_info is None
        (cone clear) or (min_forward, strafe_bias, severity, free_left,
        free_right) -- exactly what gets published on ~/decision, so the
        caller does not need to re-derive anything for that. strafe_bias
        is continuous in [-1, 1] (positive = left), not a discrete
        left-or-right choice -- see _read_obstacle. Note this is the
        value actually used to compute vy, i.e. after the
        obstacle_min_steer_bias magnitude floor below, so it can differ
        from the raw filtered value _read_obstacle computed.
        """
        if obstacle is None:
            # Nothing ahead does not mean nothing beside: the ordinary
            # person-following strafe still has to clear the flank it is
            # about to move into.
            return vx, self._limit_strafe(vy, zones), va, None

        min_forward, steer_bias, free_left, free_right = obstacle

        # Magnitude floor once genuinely close (see obstacle_min_steer_bias
        # param comment) -- a blended steer_bias can legitimately sit near
        # zero (near-symmetric obstacle, or a near-centered target washing
        # out the free-space term) while forward speed is simultaneously
        # being cut by severity below, which otherwise leaves the robot
        # just braking in place instead of routing around what's ahead.
        # Uses obstacle_last_committed_sign, NOT this frame's own
        # (possibly noise-dominated, near-zero) sign -- using the
        # instantaneous sign here would make the floor itself flip sides
        # on pure scan noise every frame once the signal is small enough,
        # i.e. reintroduce exactly the oscillation this whole redesign was
        # meant to remove. See obstacle_sign_confidence param comment.
        if severity >= self.obstacle_min_steer_severity \
                and abs(steer_bias) < self.obstacle_min_steer_bias:
            steer_bias = math.copysign(
                self.obstacle_min_steer_bias, self.obstacle_last_committed_sign)

        vy = _clamp(vy + steer_bias * self.obstacle_avoid_gain * severity,
                    -self.max_vy, self.max_vy)
        vy = self._limit_strafe(vy, zones)

        if vx > 0.0:
            if min_forward <= self.obstacle_critical_distance:
                vx = 0.0
            else:
                vx = vx * (1.0 - severity)

        return vx, vy, va, (min_forward, steer_bias, severity, free_left, free_right)

    def _get_lidar_yaw_offset(self, scan):
        """Yaw (radians) to add to a raw scan angle to get the equivalent
        angle relative to base_frame -- None if not known yet (tf not
        available/settled; caller should just skip that scan rather than
        guess). See module docstring: a scan's own angle=0 is "forward
        for the sensor", not "forward for the robot", unless the two
        happen to be mounted with zero relative yaw."""
        if self.lidar_yaw_offset is not None:
            return self.lidar_yaw_offset
        try:
            transform = self.tf_buffer.lookup_transform(
                self.base_frame, scan.header.frame_id, Time())
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException,
                tf2_ros.ExtrapolationException) as exc:
            self.get_logger().warn(
                f"lidar tf lookup ({scan.header.frame_id} -> {self.base_frame}) "
                f'failed, skipping obstacle check this frame: {exc}',
                throttle_duration_sec=5.0)
            return None

        self.lidar_yaw_offset = _yaw_from_quaternion(transform.transform.rotation)
        self.get_logger().info(
            f"lidar yaw offset ({scan.header.frame_id} -> {self.base_frame}): "
            f'{math.degrees(self.lidar_yaw_offset):.1f} deg')
        return self.lidar_yaw_offset

    def _scan_zones(self):
        """Bucket the current scan into the zones this node reasons
        about, every angle converted into base_frame. None if there is no
        usable scan or tf yet (the caller should skip the frame rather
        than guess), otherwise:

            (min_forward, free_left, free_right, min_left, min_right)

        min_forward is the CLOSEST return inside the center trigger zone
        (obstacle_trigger_cone_deg) -- "is anything actually blocking the
        path ahead". free_left/free_right are the MEAN range in each side
        zone -- "how much room is there to maneuver that way".
        min_left/min_right are the CLOSEST return in those same side
        zones -- "is it safe to move that way at all", which a mean
        cannot answer (see _limit_strafe).

        Split out from _read_obstacle so the side readings stay available
        even when the forward trigger cone is perfectly clear. On the
        differential-drive version that separation would have been
        pointless: if nothing was ahead, nothing could be hit. This base
        can strafe into something sideways with entirely clear space in
        front of it.

        Two independently-sized zones, deliberately allowed to overlap
        (see obstacle_trigger_cone_deg/obstacle_side_cone_deg): a center
        trigger zone, and two side zones each reaching in from the outer
        edge of obstacle_forward_cone_deg. Keeping those two questions on
        separate (if overlapping) data matters -- naively splitting one
        cone in half at 0deg means an obstacle sitting dead ahead lands
        entirely inside whichever half happens to include angle=0,
        dragging that side's average down for a reason that has nothing
        to do with which side actually has more room.
        """
        scan = self.last_scan
        if scan is None or not scan.ranges:
            return None

        yaw_offset = self._get_lidar_yaw_offset(scan)
        if yaw_offset is None:
            return None

        half_fov = self.obstacle_forward_cone / 2.0
        half_trigger = self.obstacle_trigger_cone / 2.0
        # Side zones reach in from the outer edge of the overall FOV by
        # obstacle_side_cone_deg -- e.g. half_fov=90deg, side=60deg gives
        # left=[+30,+90], right=[-90,-30]. Clamped so a side wider than
        # the FOV itself doesn't invert into a negative-width zone.
        side_inner_bound = max(0.0, half_fov - self.obstacle_side_cone)

        min_forward = float('inf')
        min_left = float('inf')
        min_right = float('inf')
        left_ranges = []
        right_ranges = []

        raw_angle = scan.angle_min
        for r in scan.ranges:
            # Wrap into (-pi, pi] so the zone checks below are meaningful
            # regardless of yaw_offset's sign/size.
            angle = math.atan2(math.sin(raw_angle + yaw_offset), math.cos(raw_angle + yaw_offset))
            # scan_min_range, not just the driver's own range_min: the
            # A1 reports 0.15, which lets every one of this robot's arm
            # returns straight through. See the scan_min_range param.
            r_min = max(scan.range_min, self.scan_min_range)
            if math.isfinite(r) and r_min <= r <= scan.range_max:
                # Not elif -- a point can count towards the trigger AND a
                # side zone at once where they overlap, by design.
                if -half_trigger <= angle <= half_trigger:
                    min_forward = min(min_forward, r)
                if side_inner_bound <= angle <= half_fov:
                    left_ranges.append(r)
                    min_left = min(min_left, r)
                if -half_fov <= angle <= -side_inner_bound:
                    right_ranges.append(r)
                    min_right = min(min_right, r)
            raw_angle += scan.angle_increment

        free_left = sum(left_ranges) / len(left_ranges) if left_ranges else 0.0
        free_right = sum(right_ranges) / len(right_ranges) if right_ranges else 0.0
        return min_forward, free_left, free_right, min_left, min_right

    def _read_obstacle(self, zones):
        """None if the trigger zone is clear. Otherwise (min_forward,
        strafe_bias, free_left, free_right), strafe_bias a CONTINUOUS
        value in [-1, 1] (positive = strafe left) -- a smoothed blend of
        "more open space that way" and "the target is over there", not a
        discrete left-or-right choice. See the module docstring for why
        continuous + smoothed replaced an earlier discrete-plus-
        hysteresis version that kept oscillating regardless of how much
        hysteresis got piled onto it.

        `zones` is whatever _scan_zones() returned this cycle (None if
        there was no usable scan/tf).

        Trade-off worth knowing: something very close but entirely
        outside the trigger zone will not trigger avoidance at all. Keep
        the trigger zone wide enough that a real hazard shows up there
        before it actually becomes one; this is not a full side-collision
        check while moving. On this base, _limit_strafe covers the
        specific sideways case that gap would otherwise leave wide open
        now that the robot deliberately moves that way.
        """
        if zones is None:
            return None
        min_forward, free_left, free_right, _, _ = zones

        if min_forward >= self.obstacle_distance_threshold:
            # Cone's clear -- reset the smoothed bias so the next
            # encounter (which could be a completely different
            # situation) starts fresh instead of inheriting a stale one.
            self.obstacle_steer_bias = 0.0
            return None

        # Each input squashed to a continuous [-1,1] contribution (see
        # obstacle_free_space_scale_m/obstacle_frame_angle_scale_deg)
        # instead of collapsing to a hard sign -- a small difference/
        # angle gives a small nudge, not the same full-strength vote as
        # a large one.
        free_bias = math.tanh((free_left - free_right) / self.obstacle_free_space_scale) \
            if self.obstacle_free_space_scale > 0.0 else 0.0
        frame_bias = math.tanh(self.last_known_theta / self.obstacle_frame_angle_scale) \
            if self.obstacle_frame_angle_scale > 0.0 else 0.0

        # The free/frame blend weight is NOT constant -- it rises towards
        # obstacle_frame_bias_max as the target's actual (raw,
        # un-deadbanded) bearing approaches obstacle_frame_deadband, i.e.
        # "how close are they to actually being at risk of leaving
        # frame". A low, fixed weight is what makes the direction choice
        # stable when nothing is at stake (see module docstring -- a
        # target that's roughly centered but noisily swinging must not
        # get to flip the vote), but a bagged field test (run
        # 20260817_200939, ~20-22s and ~27.5-28.8s) showed that same low,
        # fixed weight lets avoidance commit hard AWAY from a target that
        # was already near/past the frame edge -- free space won the
        # vote and forced motion opposite to where the person actually
        # was, losing them within about a second both times. (That test
        # was on the differential-drive base, where the commitment was a
        # turn; here it is a strafe, which moves the camera off target
        # far less violently -- so this is less acute than it was, but
        # the vote itself is unchanged and so is the fix.) Scaling the
        # weight up specifically when there's real risk keeps the stable
        # low-weight behavior for the common case while still yielding to
        # "don't lose the person" exactly when it matters.
        edge_urgency = _clamp(
            abs(self.last_known_theta) / self.obstacle_frame_deadband, 0.0, 1.0) \
            if self.obstacle_frame_deadband > 0.0 else 0.0
        bias = self.obstacle_avoid_frame_bias + \
            (self.obstacle_frame_bias_max - self.obstacle_avoid_frame_bias) * edge_urgency
        raw_bias = (1.0 - bias) * free_bias + bias * frame_bias

        # Low-pass filter: this, not a discrete decision, is what
        # actually prevents oscillation -- see module docstring.
        alpha = self.obstacle_steer_smoothing
        self.obstacle_steer_bias = alpha * raw_bias + (1.0 - alpha) * self.obstacle_steer_bias

        # Only trust this frame's sign enough to remember it once it's
        # confidently non-zero -- see obstacle_sign_confidence param
        # comment. Below that, obstacle_last_committed_sign is left as
        # whatever it already was. Checked against raw_bias (this frame's
        # actual signal), NOT the smoothed self.obstacle_steer_bias --
        # the EMA lags by design (that's what damps oscillation), but
        # that same lag means the committed sign could still be stuck on
        # a stale/default direction for several frames after a real,
        # decisive signal has already arrived. With a narrow gap between
        # obstacle_distance_threshold and obstacle_critical_distance,
        # there may only BE a few frames before the hard stop -- a floor
        # that's still pointing the wrong way for that entire window
        # looks exactly like "stuck" (fighting itself: floor pushing one
        # way, the real, still-smoothing signal pulling the other).
        if abs(raw_bias) >= self.obstacle_sign_confidence:
            self.obstacle_last_committed_sign = math.copysign(1.0, raw_bias)

        self.get_logger().warn(
            f'obstacle {min_forward:.2f}m ahead -- strafe_bias={self.obstacle_steer_bias:+.2f} '
            f'({"left" if self.obstacle_steer_bias >= 0.0 else "right"}; '
            f'free: L={free_left:.2f}m R={free_right:.2f}m, raw={raw_bias:+.2f})',
            throttle_duration_sec=1.0)
        return min_forward, self.obstacle_steer_bias, free_left, free_right

    def _search_timer_cb(self):
        if self.target_state != 'REACQUIRE':
            return  # nothing to do -- _tracks_callback drives TRACKING/SEARCH/LOST

        now = self.get_clock().now()
        elapsed = (now - self.search_phase_start).nanoseconds * 1e-9

        if self.search_phase == 'rotate':
            if elapsed >= self.search_rotate_burst_duration:
                self.search_phase = 'pause'
                self.search_phase_start = now
                self._publish('REACQUIRE', 0.0, 0.0, 0.0, search_phase='pause')
            else:
                # Rotate back towards wherever the target was last seen
                # (sign of the last bearing computed while TRACKING).
                # Deliberately still a ROTATION on this omni base, not a
                # strafe: the point is to sweep the camera over ground it
                # cannot currently see, and strafing mostly just slides
                # the same view sideways. See the module docstring.
                direction = 1.0 if self.last_known_theta >= 0.0 else -1.0
                self._publish('REACQUIRE', 0.0, 0.0,
                              direction * self.search_angular_speed,
                              search_phase='rotate', search_direction=direction)
        else:
            self._publish('REACQUIRE', 0.0, 0.0, 0.0, search_phase='pause')
            if elapsed >= self.search_pause_duration:
                self.search_phase = 'rotate'
                self.search_phase_start = now

    def _publish(self, mode, vx, vy, va, base_vx=0.0, base_vy=0.0, base_va=0.0,
                 obstacle=None, search_phase='', search_direction=0.0):
        twist = Twist()
        # All three are normalized throttle in [-1, 1], NOT m/s and rad/s
        # -- see the module docstring's UNITS section. BaseDriver mixes
        # linear.y in as the strafe axis of its four-wheel solution; on
        # the differential-drive base it was simply ignored.
        twist.linear.x = vx
        twist.linear.y = vy
        twist.angular.z = va
        self.cmd_vel_pub.publish(twist)
        self.last_drive_time = self.get_clock().now()

        decision = RobotDecision()
        decision.header.stamp = self.last_drive_time.to_msg()
        decision.mode = mode
        decision.base_vx = base_vx
        decision.base_vy = base_vy
        decision.base_va = base_va
        if obstacle is not None:
            min_forward, direction, severity, free_left, free_right = obstacle
            decision.obstacle_detected = True
            decision.obstacle_min_forward_m = float(min_forward)
            decision.obstacle_direction = float(direction)
            decision.obstacle_severity = float(severity)
            decision.free_space_left_m = float(free_left)
            decision.free_space_right_m = float(free_right)
        else:
            decision.obstacle_detected = False
            decision.obstacle_min_forward_m = float('nan')
            decision.obstacle_direction = 0.0
            decision.obstacle_severity = 0.0
            decision.free_space_left_m = float('nan')
            decision.free_space_right_m = float('nan')
        decision.search_phase = search_phase
        decision.search_direction = search_direction
        decision.final_vx = vx
        decision.final_vy = vy
        decision.final_va = va
        self.decision_pub.publish(decision)

    def _watchdog(self):
        elapsed = (self.get_clock().now() - self.last_drive_time).nanoseconds * 1e-9
        if elapsed > self.timeout:
            self._stop('TIMEOUT')

    def _stop(self, mode):
        self._publish(mode, 0.0, 0.0, 0.0)


def main(args=None):
    rclpy.init(args=args)
    node = RobotControllerNode()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
