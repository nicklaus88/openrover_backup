"""Control-law tests for the omnidirectional port of robot_controller_node.

These exercise the parts that changed when this package moved from the
differential-drive TurtleBot4 to the RoverMaster omni base -- holonomic
translation, the yaw deadband, strafe-based obstacle avoidance and the
sideways clearance guard. They drive the node object directly (no
executor, no DDS discovery) and read back what it published, so they run
anywhere rclpy imports.
"""
import math
import os

import pytest
import rclpy
from ament_index_python.packages import get_package_share_directory

from person_follower.robot_controller_node import RobotControllerNode


class _FakePoint:
    def __init__(self, x, y, z=0.0):
        self.x, self.y, self.z = x, y, z


class _FakeTrack:
    """Just the two attributes _drive_toward_target actually reads."""

    def __init__(self, x, y):
        self.id = 1
        self.position_base_link = _FakePoint(x, y)


class _FakeScan:
    """A LaserScan-shaped object with a uniform range everywhere, so the
    obstacle logic sees a clear world unless a test says otherwise."""

    def __init__(self, value=10.0, count=360):
        self.ranges = [value] * count
        self.angle_min = -math.pi
        self.angle_increment = 2.0 * math.pi / count
        self.range_min = 0.12
        self.range_max = 12.0

        class _H:
            frame_id = 'laser'
        self.header = _H()

    def set_arc(self, center_deg, half_width_deg, value):
        """Put an obstacle at a bearing, in the scan's own frame."""
        for i in range(len(self.ranges)):
            ang = math.degrees(self.angle_min + i * self.angle_increment)
            delta = (ang - center_deg + 180.0) % 360.0 - 180.0
            if abs(delta) <= half_width_deg:
                self.ranges[i] = value


# Deliberately the REAL deployed config, not the node's code defaults.
# Several of them differ substantially -- obstacle_forward_cone_deg is 70
# in code and 180 in the config, and at 70 the two side zones collapse
# into a [-35,0]/[0,35] split that has no outer reach at all, so the
# sideways guard would be testing something the robot never runs. Loading
# the config also means a bad edit to it fails these tests.
_PARAMS = os.path.join(
    get_package_share_directory('person_follower'),
    'config', 'person_follower.yaml')


@pytest.fixture(scope='module', autouse=True)
def _ros():
    rclpy.init(args=['--ros-args', '--params-file', _PARAMS])
    yield
    rclpy.shutdown()


@pytest.fixture
def node():
    n = RobotControllerNode()
    # The lidar mount lookup needs tf2, which needs a running graph.
    # Every test here places obstacles in base_frame angles directly, so
    # pin the offset and skip the lookup.
    n.lidar_yaw_offset = 0.0
    n.last_scan = _FakeScan()
    sent = []
    n._publish = lambda *a, **k: sent.append((a, k))
    n.sent = sent
    yield n
    n.destroy_node()


def test_config_actually_loaded(node):
    """Guard on the fixture itself: if the params file silently failed to
    load, every other test here would still pass while exercising the
    node's code defaults instead of what the robot runs."""
    assert math.degrees(node.obstacle_forward_cone) == pytest.approx(180.0)
    assert node.scan_min_range == pytest.approx(0.25)
    assert node.obstacle_side_clearance == pytest.approx(0.35)
    assert math.degrees(node.yaw_deadband) == pytest.approx(12.0)


# The 512x288 preview matches the ISP's 16:9, so nothing is cropped and
# the horizontal FOV is the sensor's full 64.9 deg -- atan(cx/fx) with
# the preview's fx=402.64, cx=256. Every bearing threshold in the config
# has to live inside that half-angle or it is unreachable, and a DEADBAND
# set outside it silently means "never rotate": that is exactly what a
# 250-square preview (half-FOV 19.6 deg) did to an 18 deg deadband on
# 2026-08-26. See config/person_follower.yaml.
#
# If the preview geometry changes again, this constant and every angular
# threshold in the config move together. Nothing in the code couples them.
PREVIEW_HALF_FOV_DEG = 32.4


def test_framing_thresholds_fit_inside_the_camera_fov(node):
    """Regression guard for the retune of 2026-08-28.

    obstacle_frame_deadband_deg was 26 against a 19.6 deg half-FOV (the
    250-square preview then in use). It is
    max()'d with yaw_deadband, so at full obstacle severity the robot did
    not begin to rotate until the target was 6 deg BEYOND the edge of the
    frame -- framing was not loosened while avoiding, it was disabled,
    and the target was reliably lost exactly while maneuvering.

    Nothing in the code enforces the relationship, so assert it here for
    the config the robot actually loads.
    """
    _assert_profile_fits(
        'person_follower.yaml',
        math.degrees(node.yaw_deadband),
        math.degrees(node.obstacle_frame_deadband),
        math.degrees(node.max_bearing))


def _assert_profile_fits(where, yaw, frame, max_bearing):
    assert yaw < frame, (
        f'{where}: obstacle_frame_deadband_deg ({frame}) must exceed '
        f'yaw_deadband_deg ({yaw}) -- the controller takes max() of the '
        f'two, so at or below the plain deadband it is inert')
    assert frame < PREVIEW_HALF_FOV_DEG, (
        f'{where}: obstacle_frame_deadband_deg={frame} is outside the camera '
        f'half-FOV ({PREVIEW_HALF_FOV_DEG} deg): framing is switched off '
        f'while avoiding')
    assert max_bearing > PREVIEW_HALF_FOV_DEG, (
        f'{where}: max_bearing_deg is a coasted-track guard -- inside the FOV '
        f'it would suppress translation for targets that are still visible')


@pytest.mark.parametrize('style', ['strafe', 'balanced', 'yaw'])
def test_every_follow_style_profile_fits_the_fov(style):
    """The same invariant, for every profile the follow_style launch
    argument can select -- not just the one the fixture happens to load.

    A profile is four numbers that move together (see FOLLOW STYLE in
    config/person_follower.yaml); it is exactly the kind of file where a
    later edit widens a deadband past the FOV again without anyone
    running that style until the robot is already out on a bag run.
    """
    import yaml

    share = get_package_share_directory('person_follower')
    path = os.path.join(share, 'config', f'follow_style_{style}.yaml')
    assert os.path.exists(path), f'{path} missing -- follow_style:={style} would fail'
    params = yaml.safe_load(open(path))['robot_controller_node']['ros__parameters']

    assert params['follow_style'] == style, \
        'the profile must name itself -- it labels bags in a training set'
    # max_bearing_deg is not part of a profile, so take it from the base
    # config the overlay is applied on top of.
    base = yaml.safe_load(
        open(os.path.join(share, 'config', 'person_follower.yaml')))
    max_bearing = base['robot_controller_node']['ros__parameters']['max_bearing_deg']
    _assert_profile_fits(
        f'follow_style_{style}.yaml',
        params['yaw_deadband_deg'],
        params['obstacle_frame_deadband_deg'],
        max_bearing)


def _cmd(node):
    """(vx, vy, va) from the most recent _publish call."""
    args, _ = node.sent[-1]
    return args[1], args[2], args[3]


# ---------------------------------------------------------------- yaw --

def test_small_bearing_strafes_without_turning(node):
    """The core of the port: a target off to one side but still well
    inside the camera frame is closed by strafing, with NO rotation."""
    # ~4.8 deg to the left, inside the 12 deg yaw deadband.
    node._drive_toward_target(_FakeTrack(3.0, 0.25))
    vx, vy, va = _cmd(node)

    assert va == 0.0, 'inside yaw_deadband the robot must not rotate at all'
    assert vy > 0.0, 'target on the left must produce a leftward strafe'
    assert vx > 0.0, 'target beyond standoff must still be approached'


def test_large_bearing_turns_to_reframe(node):
    """Past the deadband, yaw comes back to keep them in frame."""
    # ~16.7 deg left: past yaw_deadband (12), inside max_bearing (45)
    # and well inside the preview's ~32.4 deg half-FOV, i.e. a target
    # the camera can actually still see.
    node._drive_toward_target(_FakeTrack(3.0, 0.9))
    _, vy, va = _cmd(node)

    assert va > 0.0, 'past the deadband the robot must turn to re-frame'
    assert vy > 0.0, 'and should still be strafing towards them'


def test_beyond_max_bearing_reframes_without_translating(node):
    """max_bearing_deg is now a "stop and re-frame" guard, not an
    alignment gate on forward drive."""
    # ~63 deg left, past max_bearing (30).
    node._drive_toward_target(_FakeTrack(1.0, 2.0))
    vx, vy, va = _cmd(node)

    assert (vx, vy) == (0.0, 0.0), 'translation suppressed past max_bearing'
    assert va > 0.0, 'but still turning to bring them back into frame'


# -------------------------------------------------------- translation --

def test_holds_standoff_distance(node):
    """At exactly `distance`, translation is ~zero -- no creeping."""
    node._drive_toward_target(_FakeTrack(node.distance, 0.0))
    vx, vy, _ = _cmd(node)

    assert vx == pytest.approx(0.0, abs=1e-6)
    assert vy == pytest.approx(0.0, abs=1e-6)


def test_backs_off_when_too_close(node):
    """enable_back gates retreat along whatever direction they are in --
    on this base that can be mostly sideways."""
    node._drive_toward_target(_FakeTrack(0.4, 0.0))
    vx, _, _ = _cmd(node)

    assert vx < 0.0, 'closer than standoff must reverse'


def test_translation_limit_preserves_direction(node):
    """_limit_translation must scale the vector, not clamp axes
    independently -- clamping separately rotates the commanded heading
    whenever one axis saturates first, which it will since max_vx and
    max_vy differ."""
    vx, vy = node._limit_translation(10.0, 5.0)

    assert vx / vy == pytest.approx(2.0), 'heading must survive limiting'
    assert vx <= node.max_vx + 1e-9
    assert vy <= node.max_vy + 1e-9


def test_translation_limit_respects_both_axis_caps(node):
    vx, vy = node._limit_translation(0.0, 99.0)
    assert vy == pytest.approx(node.max_vy)
    assert vx == pytest.approx(0.0)


# ---------------------------------------------------------- obstacles --

def test_obstacle_ahead_strafes_and_slows(node):
    """Avoidance must land on vy (sidestep), not va (turn), and cut
    forward speed rather than lateral."""
    scan = _FakeScan()
    scan.set_arc(0.0, 8.0, 0.40)      # dead ahead, inside threshold (0.5)
    scan.set_arc(60.0, 25.0, 8.0)     # lots of room to the left
    scan.set_arc(-60.0, 25.0, 0.6)    # much less to the right
    node.last_scan = scan

    node._drive_toward_target(_FakeTrack(3.0, 0.0))
    vx, vy, _ = _cmd(node)

    assert vy > 0.0, 'must sidestep towards the open (left) side'
    assert vx < node.max_vx, 'forward speed must be scaled down'


def test_critical_distance_stops_forward_but_keeps_sidestepping(node):
    scan = _FakeScan()
    # 0.30: inside critical (0.35) but still above scan_min_range (0.25),
    # i.e. in the band where a real obstacle is actually visible.
    scan.set_arc(0.0, 8.0, 0.30)
    scan.set_arc(60.0, 25.0, 8.0)
    scan.set_arc(-60.0, 25.0, 0.6)
    node.last_scan = scan

    node._drive_toward_target(_FakeTrack(3.0, 0.0))
    vx, vy, _ = _cmd(node)

    assert vx == 0.0, 'hard stop on forward motion inside critical distance'
    assert vy != 0.0, 'but the sidestep continues'


def test_strafe_blocked_into_occupied_side(node):
    """The guard with no differential-drive equivalent: never strafe into
    a flank that is already close to something, even with nothing ahead."""
    scan = _FakeScan()
    scan.set_arc(75.0, 10.0, 0.30)    # something tight on the LEFT
    node.last_scan = scan

    # Target to the left, which would otherwise command a left strafe.
    node._drive_toward_target(_FakeTrack(3.0, 0.6))
    _, vy, _ = _cmd(node)

    assert vy == 0.0, 'strafe into an occupied flank must be refused'


def test_strafe_allowed_away_from_occupied_side(node):
    """The guard is directional -- a close left flank must not also
    prevent strafing right."""
    scan = _FakeScan()
    scan.set_arc(75.0, 10.0, 0.30)    # tight on the LEFT
    node.last_scan = scan

    node._drive_toward_target(_FakeTrack(3.0, -0.6))   # target on the RIGHT
    _, vy, _ = _cmd(node)

    assert vy < 0.0, 'moving away from the close side must still be allowed'


def test_own_arm_returns_are_ignored(node):
    """This rover's lidar sees its own arm at ~0.18m. Without
    scan_min_range those returns pin the robot in place forever."""
    scan = _FakeScan()
    # Arm-like returns all around the robot, all below scan_min_range.
    scan.set_arc(0.0, 20.0, 0.17)
    scan.set_arc(75.0, 20.0, 0.17)
    scan.set_arc(-75.0, 20.0, 0.17)
    node.last_scan = scan

    node._drive_toward_target(_FakeTrack(3.0, 0.0))
    vx, _, _ = _cmd(node)

    assert vx > 0.0, 'sub-scan_min_range returns must not stop the robot'


def test_blind_below_scan_min_range_is_deliberate(node):
    """Documents a real, accepted limitation rather than asserting good
    behaviour: because scan_min_range exists to hide the robot's own arm,
    ANY return below it is invisible -- including a genuine obstacle.

    The visible band for the hard guards is therefore narrow:
    scan_min_range (0.25) .. obstacle_critical_distance (0.35) forward,
    and .. obstacle_side_clearance (0.35) sideways. In practice things
    approach through the much wider obstacle_distance_threshold (0.5)
    band first and the robot slows/stops before reaching it, but
    something appearing suddenly inside 0.25m is not seen at all.

    A tighter fix would mask only the arm's angular sector instead of
    every close return, at the cost of a robot-specific angle to keep in
    sync. config/scan_filter.yaml and config/amcl_params.yaml both made
    the same global-range-floor tradeoff, and this matches them.
    """
    scan = _FakeScan()
    scan.set_arc(0.0, 8.0, 0.10)     # well inside scan_min_range
    node.last_scan = scan

    node._drive_toward_target(_FakeTrack(3.0, 0.0))
    vx, _, _ = _cmd(node)

    assert vx > 0.0, 'documented blind spot: sub-0.25m returns are ignored'


def test_no_scan_does_not_crash(node):
    """No tf/scan yet must skip obstacle logic, not raise."""
    node.last_scan = None
    node._drive_toward_target(_FakeTrack(3.0, 0.0))
    vx, _, _ = _cmd(node)
    assert vx > 0.0


# -------------------------------------------------------------- units --

def test_commands_stay_within_normalized_limits(node):
    """Every output is normalized throttle -- BaseDriver clamps to +-1,
    so anything outside the configured caps is a bug here."""
    for x, y in [(20.0, 0.0), (0.0, 20.0), (-20.0, 8.0), (5.0, -14.0)]:
        node._drive_toward_target(_FakeTrack(x, y))
        vx, vy, va = _cmd(node)
        assert abs(vx) <= node.max_vx + 1e-9
        assert abs(vy) <= node.max_vy + 1e-9
        assert abs(va) <= node.max_va + 1e-9
