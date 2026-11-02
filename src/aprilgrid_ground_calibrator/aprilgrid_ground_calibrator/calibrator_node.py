#!/usr/bin/env python3
"""
aprilgrid_ground_calibrator / calibrator_node.py

Calibrates camera height above the ground and camera downward tilt angle
(plus roll) using a planar AprilGrid target (4 cols x 5 rows, tag36h11 by
default) placed flat on the floor.

======================================================================
COORDINATE FRAME CONVENTIONS (read this before touching the geometry)
======================================================================

BOARD / GROUND FRAME
---------------------
Origin  : the physical top-left corner of tag (row=0, col=0) -- i.e. the
          corner nearest the "0" label in the printed ID map below.
X_board : increases to the RIGHT, along columns.
Y_board : increases UP THE PAGE -- i.e. OPPOSITE the direction of
          increasing row index. Row 0 (top row on paper) has larger Y
          than row 4 (bottom row on paper).
Z_board : the board normal, chosen so that (X_board, Y_board, Z_board)
          is right-handed AND Z_board points AWAY FROM THE FLOOR (up,
          toward the sky / toward the camera).

          This is exactly why Y_board is defined pointing "up the page"
          rather than "down the page": with X right and Y down (a
          natural image-like convention) the right-handed cross product
          X x Y points INTO the floor, which would silently flip the
          sign of every height/tilt number computed downstream. With X
          right and Y up, X x Y points up out of the floor, which is
          what we want.

Because the AprilGrid lies flat on the floor, every tag corner has
Z_board == 0 by construction.

CAMERA (OPTICAL) FRAME
------------------------
Standard OpenCV / OAK-D optical-frame convention:
    +x = right, +y = down, +z = forward (out of the lens).
This is the frame that cv2.solvePnP's rvec/tvec are expressed in.

PNP TRANSFORM AND ITS INVERSE
-------------------------------
cv2.solvePnP(...) returns (rvec, tvec) such that

    X_camera = R_camera_board @ X_board + t_camera_board          (1)

where R_camera_board = cv2.Rodrigues(rvec) and t_camera_board = tvec.

We want the CAMERA's pose expressed IN THE BOARD/GROUND FRAME, i.e. the
inverse of (1):

    R_board_camera = R_camera_board.T
    t_board_camera = -R_camera_board.T @ t_camera_board

so that

    X_board = R_board_camera @ X_camera + t_board_camera

t_board_camera is the camera's XYZ position in ground coordinates: its Z
component is exactly the (signed) camera height above the Z_board=0
floor plane.

TAG CORNER ORDERING
----------------------
We assume the widely-documented AprilTag corner convention used by the
reference AprilTag C library and by pupil_apriltags (which wraps it):
corners are returned counter-clockwise starting at the tag's own
bottom-left corner:

    index 0: bottom-left
    index 1: bottom-right
    index 2: top-right
    index 3: top-left

"bottom/top/left/right" refer to the tag as printed in its upright
orientation (all tags on a standard AprilGrid share one orientation --
none are individually rotated relative to the board). Because Y_board
points "up the page", the tag's "top" is +Y_board and its "bottom" is
-Y_board, consistent with the board-frame definition above.

If your detector's actual corner order differs (this has genuinely
varied across AprilTag library versions), set the ROS parameter
`flip_tag_corner_order: true`, or edit
AprilGridGeometry.TAG_LOCAL_CORNER_OFFSETS directly. The debug image
draws each corner's index (0-3) so the assumption can be checked
visually against the physical board.
"""

import math
import time
from dataclasses import dataclass
from typing import Dict, List, Optional, Tuple

import cv2
import numpy as np
import yaml

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data, QoSProfile, ReliabilityPolicy, HistoryPolicy

from sensor_msgs.msg import Image, CameraInfo
from std_msgs.msg import Float64
from geometry_msgs.msg import PoseStamped, TransformStamped
from cv_bridge import CvBridge

try:
    import tf2_ros
    _TF2_AVAILABLE = True
except ImportError:
    _TF2_AVAILABLE = False

try:
    from pupil_apriltags import Detector as PupilAprilTagDetector
    _PUPIL_APRILTAGS_AVAILABLE = True
except ImportError:
    _PUPIL_APRILTAGS_AVAILABLE = False

try:
    from apriltag_msgs.msg import AprilTagDetectionArray
    _APRILTAG_ROS_MSGS_AVAILABLE = True
except ImportError:
    _APRILTAG_ROS_MSGS_AVAILABLE = False


# ======================================================================
# Tag ID <-> (row, col) mapping
# ======================================================================
def tag_id_to_row_col(tag_id: int, rows: int, cols: int) -> Optional[Tuple[int, int]]:
    """
    Maps an AprilTag ID to its (row, col) position on the AprilGrid.

    CONVENTION (this is the ONLY function that encodes the physical tag
    layout -- edit only this if your board differs):

    We assume the standard Kalibr-style sequential layout: tag IDs
    increase left-to-right along a row, then wrap to the next row down,
    starting at 0 in the top-left tag. This is the convention produced
    by Kalibr's `kalibr_create_target_pdf` for a plain AprilGrid target:

        row 0:   0    1    2    3
        row 1:   4    5    6    7
        row 2:   8    9   10   11
        row 3:  12   13   14   15
        row 4:  16   17   18   19

    i.e.   row = tag_id // cols
           col = tag_id %  cols

    If your physical board was generated with a different convention
    (column-major numbering, a different starting corner, non-sequential
    IDs, etc.), reimplement this function accordingly -- nothing else in
    this file needs to change.
    """
    if tag_id < 0 or tag_id >= rows * cols:
        return None
    return tag_id // cols, tag_id % cols


class AprilGridGeometry:
    """
    Physical AprilGrid layout -> board-frame 3D corner coordinates.

    See the module docstring for the full board-frame definition. The
    short version: origin at tag(0,0)'s top-left corner, X right along
    columns, Y up the page (opposite row direction), Z up out of the
    floor, all points at Z=0.
    """

    # (dx, dy) offsets from a tag's center, in units of half-tag-size,
    # ordered to match the assumed detector corner order:
    #   [bottom-left, bottom-right, top-right, top-left]
    TAG_LOCAL_CORNER_OFFSETS = [
        (-1.0, -1.0),  # 0: bottom-left
        (+1.0, -1.0),  # 1: bottom-right
        (+1.0, +1.0),  # 2: top-right
        (-1.0, +1.0),  # 3: top-left
    ]

    def __init__(self, rows: int, cols: int, tag_size: float, tag_spacing: float,
                 flip_tag_corner_order: bool = False):
        self.rows = rows
        self.cols = cols
        self.tag_size = tag_size
        self.tag_spacing = tag_spacing
        self.tag_pitch = tag_size + tag_spacing
        self.half_size = tag_size / 2.0

        offsets = list(self.TAG_LOCAL_CORNER_OFFSETS)
        if flip_tag_corner_order:
            offsets = offsets[::-1]
        self._corner_offsets = offsets

        # Precompute the 4 board-frame corner coordinates for every tag id.
        self._tag_object_points: Dict[int, np.ndarray] = {}
        for tag_id in range(rows * cols):
            row, col = tag_id_to_row_col(tag_id, rows, cols)
            cx = col * self.tag_pitch
            cy = -row * self.tag_pitch  # Y grows "up the page" -> negative as row grows
            pts = np.array(
                [[cx + dx * self.half_size, cy + dy * self.half_size, 0.0]
                 for (dx, dy) in self._corner_offsets],
                dtype=np.float64,
            )
            self._tag_object_points[tag_id] = pts

    def num_tags(self) -> int:
        return self.rows * self.cols

    def object_points_for_tag(self, tag_id: int) -> Optional[np.ndarray]:
        return self._tag_object_points.get(tag_id)

    def board_extent_m(self) -> Tuple[float, float]:
        width = self.cols * self.tag_size + (self.cols - 1) * self.tag_spacing
        height = self.rows * self.tag_size + (self.rows - 1) * self.tag_spacing
        return width, height

    def id_map_lines(self) -> List[str]:
        lines = []
        for row in range(self.rows):
            ids = [row * self.cols + c for c in range(self.cols)]
            lines.append(' '.join(f'{i:2d}' for i in ids))
        return lines


class ArucoMarkerGeometry:
    """
    Board-frame corner geometry for a SINGLE square ArUco marker used as
    the ground-plane calibration target, as an alternative to the full
    AprilGrid (fiducial_type: 'aruco_single').

    Board frame convention matches AprilGridGeometry / the module
    docstring: origin at the marker's own center, X right, Y up, Z up out
    of the floor (toward the camera), all corners at Z=0. Since there is
    only one marker, "board frame" and "marker-local frame" are the same
    thing here.

    IMPORTANT -- different corner order than the AprilTag path:
    cv2.aruco.ArucoDetector.detectMarkers() returns each marker's 4
    corners in a FIXED order that is NOT the same as the AprilTag/
    pupil_apriltags convention used elsewhere in this file: clockwise
    starting at the marker's own top-left corner (see OpenCV's ArUco
    documentation), i.e.:

        index 0: top-left
        index 1: top-right
        index 2: bottom-right
        index 3: bottom-left

    This is exactly why a separate geometry class exists instead of
    reusing AprilGridGeometry.TAG_LOCAL_CORNER_OFFSETS -- mixing the two
    conventions up would silently corrupt the pose (wrong rotation, not
    a crash), so keeping them in clearly-labeled, separate lookup tables
    is deliberate.

    `marker_size` is the physical side length of the marker's own black
    square (the ArUco "markerLength" convention) -- NOT including any
    white quiet-zone margin around it, since that margin is not part of
    what the detector reports as the marker's corners.
    """

    MARKER_LOCAL_CORNER_OFFSETS = [
        (-1.0, +1.0),  # 0: top-left
        (+1.0, +1.0),  # 1: top-right
        (+1.0, -1.0),  # 2: bottom-right
        (-1.0, -1.0),  # 3: bottom-left
    ]

    def __init__(self, marker_id: int, marker_size: float):
        self.marker_id = marker_id
        self.marker_size = marker_size
        self.tag_pitch = marker_size  # kept for interface parity with AprilGridGeometry
        self.half_size = marker_size / 2.0
        self._object_points = np.array(
            [[dx * self.half_size, dy * self.half_size, 0.0]
             for (dx, dy) in self.MARKER_LOCAL_CORNER_OFFSETS],
            dtype=np.float64,
        )

    def num_tags(self) -> int:
        return 1

    def object_points_for_tag(self, tag_id: int) -> Optional[np.ndarray]:
        return self._object_points if tag_id == self.marker_id else None

    def board_extent_m(self) -> Tuple[float, float]:
        return self.marker_size, self.marker_size

    def id_map_lines(self) -> List[str]:
        return [f'(single marker, id={self.marker_id} -- no grid)']


# ======================================================================
# Robust statistics
# ======================================================================
def robust_stats(values: List[float]) -> Tuple[float, float, float, np.ndarray, int]:
    """
    Robust median / mean / std with outlier rejection.

    Primary method: MAD (median absolute deviation) scaled by 1.4826 to
    be comparable to a normal-distribution std-dev, with points beyond
    3.5 scaled-MAD from the median rejected. Falls back to an IQR fence
    if MAD is degenerate (e.g. many identical values, MAD == 0).

    Returns (median, mean, std, kept_values, num_rejected) computed over
    the *kept* (inlier) values -- this is what we report as the final
    calibration statistic, per the "reject statistical outliers" spec.
    """
    arr = np.asarray(values, dtype=np.float64)
    if arr.size == 0:
        return float('nan'), float('nan'), float('nan'), arr, 0

    med = np.median(arr)
    mad = np.median(np.abs(arr - med))

    if mad > 1e-12:
        sigma_est = 1.4826 * mad
        keep_mask = np.abs(arr - med) <= 3.5 * sigma_est
    else:
        q1, q3 = np.percentile(arr, [25, 75])
        iqr = q3 - q1
        if iqr > 1e-12:
            lo, hi = q1 - 1.5 * iqr, q3 + 1.5 * iqr
            keep_mask = (arr >= lo) & (arr <= hi)
        else:
            keep_mask = np.ones_like(arr, dtype=bool)

    kept = arr[keep_mask]
    if kept.size == 0:
        kept = arr  # never end up with zero samples

    return (float(np.median(kept)), float(np.mean(kept)), float(np.std(kept)),
            kept, int(arr.size - kept.size))


def bgr8_to_imgmsg(img: np.ndarray, header) -> Image:
    """
    Build a sensor_msgs/Image (encoding='bgr8') directly from a BGR uint8
    numpy array, without going through cv_bridge.cv2_to_imgmsg().

    This sidesteps a real, observed environment issue: cv_bridge's
    cv2_to_imgmsg() looks up the OpenCV "CV_8UC3"-style type code in an
    internal dict that it builds by introspecting the `cv2` module at
    import time. If a pip-installed `opencv-python` (here: 5.0.0) shadows
    the apt `python3-opencv` that the prebuilt ros-humble cv_bridge
    .so was actually compiled against, that dict can end up missing
    entries for multi-channel types and cv2_to_imgmsg raises
    `KeyError(16)` for every 3-channel image. imgmsg_to_cv2() (the
    incoming-image direction, which we still use) does not hit this code
    path, so only outgoing conversions need this workaround. The wire
    format for 'bgr8' is simple enough (row-major, 1 byte/channel, no
    padding) that hand-building it here is both correct and immune to
    whichever cv2 happens to be first on sys.path.
    """
    msg = Image()
    msg.header = header
    msg.height, msg.width = img.shape[0], img.shape[1]
    msg.encoding = 'bgr8'
    msg.is_bigendian = 0
    contiguous = np.ascontiguousarray(img, dtype=np.uint8)
    msg.step = msg.width * 3
    msg.data = contiguous.tobytes()
    return msg


def rotation_matrix_to_quaternion(R: np.ndarray) -> Tuple[float, float, float, float]:
    """Standard rotation-matrix -> quaternion (x, y, z, w) conversion."""
    m00, m01, m02 = R[0]
    m10, m11, m12 = R[1]
    m20, m21, m22 = R[2]
    trace = m00 + m11 + m22

    if trace > 0.0:
        s = 0.5 / math.sqrt(trace + 1.0)
        w = 0.25 / s
        x = (m21 - m12) * s
        y = (m02 - m20) * s
        z = (m10 - m01) * s
    elif m00 > m11 and m00 > m22:
        s = 2.0 * math.sqrt(1.0 + m00 - m11 - m22)
        w = (m21 - m12) / s
        x = 0.25 * s
        y = (m01 + m10) / s
        z = (m02 + m20) / s
    elif m11 > m22:
        s = 2.0 * math.sqrt(1.0 + m11 - m00 - m22)
        w = (m02 - m20) / s
        x = (m01 + m10) / s
        y = 0.25 * s
        z = (m12 + m21) / s
    else:
        s = 2.0 * math.sqrt(1.0 + m22 - m00 - m11)
        w = (m10 - m01) / s
        x = (m02 + m20) / s
        y = (m12 + m21) / s
        z = 0.25 * s

    return x, y, z, w


# ======================================================================
# Pose estimation helpers
# ======================================================================
def estimate_board_pose(object_points: np.ndarray, image_points: np.ndarray,
                         K: np.ndarray, D: np.ndarray, ransac_reproj_px: float):
    """
    Estimate ONE planar-board pose from many tag-corner correspondences.

    Solver choice
    -------------
    The AprilGrid is perfectly planar (every object point has Z=0). Some
    classical PnP solvers (e.g. EPNP) are formulated primarily for
    general, non-planar 3D point sets and can be poorly conditioned on
    planar configurations. OpenCV's SQPNP solver
    (cv2.SOLVEPNP_SQPNP, OpenCV >= 4.5.4) is a globally-optimal,
    non-iterative minimal solver that is explicitly correct for planar
    (and non-planar) point sets and integrates cleanly with
    solvePnPRansac. We use SQPNP when available and fall back to the
    classical iterative Levenberg-Marquardt solver
    (cv2.SOLVEPNP_ITERATIVE) on older OpenCV builds.

    We do not just trust the first solution: RANSAC first rejects any
    grossly wrong correspondences (false detections, corner mismatches),
    and the inlier-only pose is then refined with cv2.solvePnPRefineLM,
    which directly minimizes reprojection error rather than an algebraic
    proxy.
    """
    object_points = object_points.reshape(-1, 1, 3).astype(np.float64)
    image_points = image_points.reshape(-1, 1, 2).astype(np.float64)

    pnp_flag = getattr(cv2, 'SOLVEPNP_SQPNP', cv2.SOLVEPNP_ITERATIVE)
    try:
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points, image_points, K, D,
            flags=pnp_flag,
            reprojectionError=ransac_reproj_px,
            confidence=0.999,
            iterationsCount=300,
        )
    except cv2.error:
        pnp_flag = cv2.SOLVEPNP_ITERATIVE
        ok, rvec, tvec, inliers = cv2.solvePnPRansac(
            object_points, image_points, K, D,
            flags=pnp_flag,
            reprojectionError=ransac_reproj_px,
            confidence=0.999,
            iterationsCount=300,
        )

    if not ok or inliers is None or len(inliers) < 4:
        return None

    inlier_idx = inliers.reshape(-1)
    obj_in = object_points[inlier_idx]
    img_in = image_points[inlier_idx]

    rvec, tvec = cv2.solvePnPRefineLM(obj_in, img_in, K, D, rvec, tvec)

    return rvec, tvec, inlier_idx


def compute_reprojection_errors(object_points: np.ndarray, image_points: np.ndarray,
                                 rvec: np.ndarray, tvec: np.ndarray,
                                 K: np.ndarray, D: np.ndarray) -> np.ndarray:
    """Per-point reprojection error (px) for ALL supplied correspondences."""
    projected, _ = cv2.projectPoints(object_points.reshape(-1, 1, 3), rvec, tvec, K, D)
    projected = projected.reshape(-1, 2)
    return np.linalg.norm(projected - image_points.reshape(-1, 2), axis=1)


def invert_pnp_pose(rvec: np.ndarray, tvec: np.ndarray) -> Tuple[np.ndarray, np.ndarray]:
    """T_camera_board -> T_board_camera. See module docstring for the derivation."""
    R_camera_board, _ = cv2.Rodrigues(rvec)
    R_board_camera = R_camera_board.T
    t_board_camera = -R_camera_board.T @ tvec.reshape(3, 1)
    return R_board_camera, t_board_camera


def compute_height_tilt_roll(R_board_camera: np.ndarray,
                              t_board_camera: np.ndarray) -> Tuple[float, float, float, float]:
    """
    Returns (height_m, tilt_magnitude_deg, downward_tilt_deg, roll_deg).

    height: perpendicular distance from the camera to the Z_board=0
        ground plane.
    tilt_magnitude: angle of the optical axis relative to the floor
        PLANE. 0 deg = looking parallel to the floor, 90 deg = looking
        straight down. Always non-negative (no sign information).
    downward_tilt: signed version of the above. Positive means the
        camera is pitched to look DOWN toward the floor (the normal
        mounting case); negative would mean pitched up above horizontal.
        Valid because +Z_board points up, away from the floor.
    roll: rotation of the camera about its own optical (forward) axis
        relative to level. Positive = the camera's RIGHT side is tilted
        UP relative to the floor (i.e. rotating counter-clockwise about
        the forward axis as seen from behind the camera, looking along
        the viewing direction). Negative = right side tilted down.
    """
    camera_height = abs(float(t_board_camera[2, 0]))

    camera_forward = np.array([0.0, 0.0, 1.0])
    forward_ground = R_board_camera @ camera_forward
    fx, fy, fz = forward_ground
    horizontal = math.hypot(fx, fy)

    tilt_rad = math.atan2(abs(fz), horizontal)
    tilt_deg = math.degrees(tilt_rad)

    downward_tilt_deg = math.degrees(math.atan2(-fz, horizontal))

    camera_right = np.array([1.0, 0.0, 0.0])
    right_ground = R_board_camera @ camera_right
    rx, ry, rz = right_ground
    roll_deg = math.degrees(math.atan2(rz, math.hypot(rx, ry)))

    return camera_height, tilt_deg, downward_tilt_deg, roll_deg


def classify_calibration(accepted: int, rejected: int, height_std: float, tilt_std: float,
                          roll_std: float, reproj_rmse_median: float,
                          max_height_std_m: float, max_tilt_std_deg: float,
                          max_roll_std_deg: float, max_reprojection_error_px: float
                          ) -> Tuple[str, List[str]]:
    if accepted == 0:
        return 'FAILED', ['No accepted frames -- calibration could not be computed.']

    status = 'GOOD'
    reasons: List[str] = []
    total = accepted + rejected
    reject_ratio = rejected / total if total > 0 else 0.0

    if height_std > max_height_std_m:
        status = 'WARNING'
        reasons.append(f'height std {height_std:.4f} m exceeds limit {max_height_std_m:.4f} m')
    if tilt_std > max_tilt_std_deg:
        status = 'WARNING'
        reasons.append(f'tilt std {tilt_std:.3f} deg exceeds limit {max_tilt_std_deg:.3f} deg')
    if roll_std > max_roll_std_deg:
        status = 'WARNING'
        reasons.append(f'roll std {roll_std:.3f} deg exceeds limit {max_roll_std_deg:.3f} deg')
    if reproj_rmse_median > max_reprojection_error_px:
        status = 'WARNING'
        reasons.append(f'median reprojection RMSE {reproj_rmse_median:.3f} px exceeds limit '
                        f'{max_reprojection_error_px:.3f} px')
    if reject_ratio > 0.5:
        status = 'FAILED'
        reasons.append(f'more than half of processed frames were rejected '
                        f'({rejected} rejected vs {accepted} accepted)')

    if not reasons:
        reasons.append('all stability and reprojection checks passed')

    return status, reasons


@dataclass
class FrameSample:
    height_m: float
    tilt_deg: float
    downward_tilt_deg: float
    roll_deg: float
    reprojection_rmse_px: float
    num_tags: int
    num_corners: int
    R_board_camera: np.ndarray
    t_board_camera: np.ndarray


def select_representative_sample(samples: List[FrameSample],
                                  h_med: float, h_std: float,
                                  t_med: float, t_std: float,
                                  r_med: float, r_std: float) -> FrameSample:
    """
    Pick the single accepted FrameSample whose (height, downward_tilt, roll)
    is closest to the robust medians, using a per-axis normalized distance
    (each axis scaled by its own std, floored so a near-zero std doesn't
    blow up the distance). Used to obtain a real, internally-consistent
    R_board_camera/t_board_camera pair for the final static TF -- the
    scalar medians alone under-determine a full 3-DoF rotation (yaw is
    intentionally never tracked), so we deliberately do NOT synthesize one.
    """
    eps_h = max(h_std, 1e-4)   # meters
    eps_t = max(t_std, 1e-3)   # degrees
    eps_r = max(r_std, 1e-3)   # degrees

    def distance(s: FrameSample) -> float:
        dh = (s.height_m - h_med) / eps_h
        dt = (s.downward_tilt_deg - t_med) / eps_t
        dr = (s.roll_deg - r_med) / eps_r
        return dh * dh + dt * dt + dr * dr

    return min(samples, key=distance)


@dataclass
class CameraModel:
    K: np.ndarray
    D: np.ndarray
    width: int
    height: int
    distortion_model: str


# ======================================================================
# Node
# ======================================================================
class AprilGridGroundCalibratorNode(Node):

    def __init__(self):
        super().__init__('aprilgrid_ground_calibrator')

        self._declare_parameters()
        p = self._read_parameters()
        self.p = p

        if p['fiducial_type'] == 'aruco_single':
            self.geometry = ArucoMarkerGeometry(
                marker_id=p['aruco_marker_id'], marker_size=p['tag_size'])
            # There is exactly one marker to find -- "min_tags" (a concept
            # that matters for the multi-tag AprilGrid) can only sensibly
            # be 1 here, regardless of what was configured.
            self._effective_min_tags = 1
            if p['min_tags'] != 1:
                self.get_logger().info(
                    f"fiducial_type='aruco_single': treating min_tags as 1 "
                    f"(configured value {p['min_tags']} does not apply to a "
                    "single-marker target).")
        elif p['fiducial_type'] == 'apriltag_grid':
            self.geometry = AprilGridGeometry(
                rows=p['tag_rows'], cols=p['tag_cols'],
                tag_size=p['tag_size'], tag_spacing=p['tag_spacing'],
                flip_tag_corner_order=p['flip_tag_corner_order'],
            )
            self._effective_min_tags = p['min_tags']
        else:
            self.get_logger().fatal(
                f"Unknown fiducial_type '{p['fiducial_type']}' "
                "(expected 'apriltag_grid' or 'aruco_single').")
            self.geometry = AprilGridGeometry(
                rows=p['tag_rows'], cols=p['tag_cols'],
                tag_size=p['tag_size'], tag_spacing=p['tag_spacing'],
                flip_tag_corner_order=p['flip_tag_corner_order'],
            )
            self._effective_min_tags = p['min_tags']

        self._print_startup_banner()

        self.bridge = CvBridge()
        self.camera_model: Optional[CameraModel] = None
        self._printed_intrinsics = False
        self._resolution_error_logged = False

        self.samples: List[FrameSample] = []
        self.rejected_count = 0
        self.finalized = False
        self._prev_accepted: Optional[FrameSample] = None

        # Wall-clock deadline, independent of whether any image/detection
        # callback ever actually fires (e.g. wrong topic name, AprilGrid
        # never in view, board not flat/large enough to be detected --
        # none of those trigger _on_image's own reject-path logging on
        # their own). Without this a bad setup just waits forever with no
        # report and no output YAML. A periodic timer -- not a check
        # inside _on_image -- is what makes this catch "zero images ever
        # received" too.
        self._start_time = time.monotonic()
        self.timeout_timer = None
        if p['calibration_timeout_sec'] > 0:
            self.timeout_timer = self.create_timer(2.0, self._check_timeout)

        # ---- Detector setup -------------------------------------------------
        self.internal_detector = None
        self.aruco_detector = None
        self.latest_apriltag_ros_detections: Dict[int, np.ndarray] = {}

        if p['fiducial_type'] == 'aruco_single':
            # A single ArUco marker is always detected internally via
            # cv2.aruco -- detection_source (which only distinguishes
            # AprilTag detection engines) does not apply here.
            if p['detection_source'] != 'internal':
                self.get_logger().warn(
                    "fiducial_type='aruco_single' always uses internal cv2.aruco "
                    f"detection; ignoring detection_source='{p['detection_source']}'.")
            if not (hasattr(cv2, 'aruco') and hasattr(cv2.aruco, 'ArucoDetector')):
                self.get_logger().fatal(
                    "fiducial_type='aruco_single' requires cv2.aruco.ArucoDetector "
                    "(OpenCV >= 4.7 built with the aruco module), which is not "
                    "available in this cv2 install."
                )
            else:
                dict_name = p['aruco_dictionary']
                dict_id = getattr(cv2.aruco, dict_name, None)
                if dict_id is None:
                    self.get_logger().fatal(
                        f"Unknown aruco_dictionary '{dict_name}' -- expected an "
                        "attribute name from cv2.aruco, e.g. 'DICT_4X4_50', "
                        "'DICT_6X6_250', 'DICT_ARUCO_ORIGINAL'."
                    )
                else:
                    aruco_dict = cv2.aruco.getPredefinedDictionary(dict_id)
                    aruco_params = cv2.aruco.DetectorParameters()
                    self.aruco_detector = cv2.aruco.ArucoDetector(aruco_dict, aruco_params)
        elif p['detection_source'] == 'internal':
            if not _PUPIL_APRILTAGS_AVAILABLE:
                self.get_logger().fatal(
                    "detection_source='internal' but the 'pupil_apriltags' python "
                    "package is not installed. Install it with:\n"
                    "    pip3 install pupil-apriltags\n"
                    "or switch to detection_source:='apriltag_ros'."
                )
            else:
                self.internal_detector = PupilAprilTagDetector(
                    families=p['tag_family'],
                    nthreads=p['apriltag_nthreads'],
                    quad_decimate=p['apriltag_quad_decimate'],
                    quad_sigma=p['apriltag_quad_sigma'],
                    refine_edges=1,
                    decode_sharpening=0.25,
                )
        elif p['detection_source'] == 'apriltag_ros':
            if not _APRILTAG_ROS_MSGS_AVAILABLE:
                self.get_logger().fatal(
                    "detection_source='apriltag_ros' but the 'apriltag_msgs' "
                    "package is not importable. Install/build apriltag_msgs, or "
                    "switch to detection_source:='internal'."
                )
            else:
                self.create_subscription(
                    AprilTagDetectionArray, p['apriltag_ros_topic'],
                    self._on_apriltag_ros_detections, qos_profile_sensor_data,
                )
        else:
            self.get_logger().fatal(
                f"Unknown detection_source '{p['detection_source']}' "
                "(expected 'internal' or 'apriltag_ros')."
            )

        # ---- TF -----------------------------------------------------------
        # Two distinct broadcasters, both optional (publish_tf) and both
        # publishing the SAME board/ground -> camera_optical transform:
        #   - tf_broadcaster (regular /tf): one message per ACCEPTED frame,
        #     while calibration is still collecting samples. Useful to watch
        #     the pose converge live in rviz. Stops naturally once
        #     self.finalized is set (see _on_image), since no more frames
        #     are processed.
        #   - static_tf_broadcaster (/tf_static): published exactly ONCE,
        #     at the end of _finalize_calibration(), using the robust
        #     median-based result. /tf_static is latched (transient local),
        #     so any node that starts later (e.g. the ground-plane
        #     estimator) still receives it. This is the transform that
        #     should actually be trusted/consumed -- it is far less noisy
        #     than any single accepted frame.
        self.tf_broadcaster = None
        self.static_tf_broadcaster = None
        if p['publish_tf']:
            if _TF2_AVAILABLE:
                self.tf_broadcaster = tf2_ros.TransformBroadcaster(self)
                self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster(self)
            else:
                self.get_logger().error('publish_tf=true but tf2_ros is not available.')

        # ---- Publishers ---------------------------------------------------
        self.height_pub = self.create_publisher(Float64, '/aprilgrid_calibration/height', 10)
        self.tilt_pub = self.create_publisher(Float64, '/aprilgrid_calibration/tilt', 10)
        self.roll_pub = self.create_publisher(Float64, '/aprilgrid_calibration/roll', 10)
        self.reproj_pub = self.create_publisher(
            Float64, '/aprilgrid_calibration/reprojection_error', 10)
        self.debug_image_pub = self.create_publisher(
            Image, '/aprilgrid_calibration/debug_image', 10)
        self.pose_pub = self.create_publisher(
            PoseStamped, '/aprilgrid_calibration/camera_pose', 10)

        # ---- Subscriptions --------------------------------------------------
        camera_info_qos = QoSProfile(
            depth=10, reliability=ReliabilityPolicy.RELIABLE, history=HistoryPolicy.KEEP_LAST)
        self.create_subscription(
            CameraInfo, p['camera_info_topic'], self._on_camera_info, camera_info_qos)
        self.create_subscription(
            Image, p['image_topic'], self._on_image, qos_profile_sensor_data)

        self.get_logger().info(
            f"Subscribed to image topic '{p['image_topic']}' and "
            f"camera_info topic '{p['camera_info_topic']}'. Waiting for data..."
        )

    # ------------------------------------------------------------------
    # Parameter handling
    # ------------------------------------------------------------------
    def _declare_parameters(self):
        defaults = [
            # This rover's OAK-D Lite (depthai_ros_driver name:=oak).
            # The FULL rgb stream, not a preview: this solve uses
            # camera_info intrinsics, and a preview stream's intrinsics
            # differ completely from the stream it is cropped from.
            ('image_topic', '/oak/rgb/image_raw'),
            ('camera_info_topic', '/oak/rgb/camera_info'),
            ('detection_source', 'internal'),
            ('apriltag_ros_topic', '/apriltag/detections'),

            # 'apriltag_grid' (default): the 4x5 AprilGrid, joint PnP over
            #   all detected tag corners.
            # 'aruco_single': a single square ArUco marker instead of the
            #   grid -- easier to print/mount large and legible, at the
            #   cost of only 4 point correspondences instead of up to 80
            #   (noisier tilt/roll, no real RANSAC redundancy). Always
            #   detected internally via cv2.aruco, regardless of
            #   detection_source. tag_size doubles as the marker's side
            #   length in this mode; tag_rows/tag_cols/tag_spacing/
            #   tag_family/flip_tag_corner_order are ignored.
            ('fiducial_type', 'apriltag_grid'),
            ('aruco_dictionary', 'DICT_ARUCO_ORIGINAL'),
            ('aruco_marker_id', 0),

            ('tag_family', 'tag36h11'),
            ('tag_rows', 5),
            ('tag_cols', 4),
            ('tag_size', 0.040),
            ('tag_spacing', 0.008),
            ('flip_tag_corner_order', False),
            ('min_tags', 4),
            ('samples_required', 100),
            ('calibration_timeout_sec', 300.0),
            ('max_reprojection_error_px', 1.0),
            ('pnp_ransac_reprojection_error_px', 3.0),
            ('min_expected_height_m', 0.1),
            ('max_expected_height_m', 2.0),
            ('max_height_std_m', 0.005),
            ('max_tilt_std_deg', 0.25),
            ('max_roll_std_deg', 0.25),
            ('max_frame_height_jump_m', 0.03),
            ('max_frame_tilt_jump_deg', 2.0),
            ('edge_on_tilt_warning_deg', 5.0),
            ('apriltag_nthreads', 2),
            ('apriltag_quad_decimate', 1.0),
            ('apriltag_quad_sigma', 0.0),
            ('axis_length_m', 0.05),
            ('show_debug_image', True),
            ('publish_tf', False),
            ('board_frame_id', 'ground'),
            ('camera_optical_frame_id', 'oakd_rgb_camera_optical_frame'),
            ('output_yaml_path', 'camera_ground_calibration.yaml'),
        ]
        for name, default in defaults:
            self.declare_parameter(name, default)

    def _read_parameters(self) -> dict:
        names = [
            'image_topic', 'camera_info_topic', 'detection_source', 'apriltag_ros_topic',
            'fiducial_type', 'aruco_dictionary', 'aruco_marker_id',
            'tag_family', 'tag_rows', 'tag_cols', 'tag_size', 'tag_spacing',
            'flip_tag_corner_order', 'min_tags', 'samples_required', 'calibration_timeout_sec',
            'max_reprojection_error_px', 'pnp_ransac_reprojection_error_px',
            'min_expected_height_m', 'max_expected_height_m', 'max_height_std_m',
            'max_tilt_std_deg', 'max_roll_std_deg', 'max_frame_height_jump_m',
            'max_frame_tilt_jump_deg', 'edge_on_tilt_warning_deg',
            'apriltag_nthreads', 'apriltag_quad_decimate', 'apriltag_quad_sigma',
            'axis_length_m', 'show_debug_image', 'publish_tf', 'board_frame_id',
            'camera_optical_frame_id', 'output_yaml_path',
        ]
        return {n: self.get_parameter(n).value for n in names}

    def _print_startup_banner(self):
        p = self.p
        self.get_logger().info('=' * 60)
        self.get_logger().info('APRILGRID GROUND CALIBRATOR')
        self.get_logger().info('=' * 60)
        if p['fiducial_type'] == 'aruco_single':
            self.get_logger().info(
                f"Target: single ArUco marker, dictionary={p['aruco_dictionary']}, "
                f"id={p['aruco_marker_id']}, size={p['tag_size']:.4f} m"
            )
        else:
            self.get_logger().info(
                f"Board: {p['tag_rows']} rows x {p['tag_cols']} cols, "
                f"family={p['tag_family']}, tag_size={p['tag_size']:.4f} m, "
                f"spacing={p['tag_spacing']:.4f} m, "
                f"pitch={self.geometry.tag_pitch:.4f} m"
            )
        w, h = self.geometry.board_extent_m()
        self.get_logger().info(f'Tag-region extent: {w:.4f} m x {h:.4f} m')
        if p['fiducial_type'] != 'aruco_single':
            self.get_logger().info(
                'Tag ID mapping (row-major, standard Kalibr AprilGrid convention):')
        for line in self.geometry.id_map_lines():
            self.get_logger().info('  ' + line)
        self.get_logger().info(f"Detection source: {p['detection_source']}")
        self.get_logger().info(f"Samples required: {p['samples_required']}")
        self.get_logger().info('=' * 60)

    # ------------------------------------------------------------------
    # CameraInfo handling
    # ------------------------------------------------------------------
    def _on_camera_info(self, msg: CameraInfo):
        K = np.array(msg.k, dtype=np.float64).reshape(3, 3)
        D = np.array(msg.d, dtype=np.float64)
        if D.size == 0:
            D = np.zeros(5, dtype=np.float64)

        if np.allclose(K, 0.0):
            self.get_logger().error(
                'Received CameraInfo with all-zero K matrix -- refusing to use it.',
                throttle_duration_sec=5.0)
            return

        self.camera_model = CameraModel(
            K=K, D=D, width=msg.width, height=msg.height,
            distortion_model=msg.distortion_model or '(unspecified)')

        if not self._printed_intrinsics:
            self._printed_intrinsics = True
            self.get_logger().info(f'Camera resolution: {msg.width} x {msg.height}')
            self.get_logger().info(f'fx: {K[0, 0]:.4f}')
            self.get_logger().info(f'fy: {K[1, 1]:.4f}')
            self.get_logger().info(f'cx: {K[0, 2]:.4f}')
            self.get_logger().info(f'cy: {K[1, 2]:.4f}')
            self.get_logger().info(f'distortion model: {self.camera_model.distortion_model} '
                                    f'({D.size} coefficients)')
            if D.size == 8:
                self.get_logger().info(
                    'Using 8-coefficient (rational polynomial) distortion model.')

    # ------------------------------------------------------------------
    # apriltag_ros bridge (optional external-detector mode)
    # ------------------------------------------------------------------
    def _on_apriltag_ros_detections(self, msg):
        detections: Dict[int, np.ndarray] = {}
        for det in msg.detections:
            try:
                tag_id = int(det.id)
                corners = np.array(
                    [[c.x, c.y] for c in det.corners], dtype=np.float64)
                if corners.shape == (4, 2):
                    detections[tag_id] = corners
            except (AttributeError, ValueError) as exc:
                self.get_logger().warn(
                    f'Failed to parse apriltag_ros detection: {exc}',
                    throttle_duration_sec=5.0)
        self.latest_apriltag_ros_detections = detections

    # ------------------------------------------------------------------
    # Detection
    # ------------------------------------------------------------------
    def _detect_tags(self, gray: np.ndarray) -> Dict[int, np.ndarray]:
        """Returns {tag_id: corners(4,2) float64} in the assumed detector order."""
        if self.p['fiducial_type'] == 'aruco_single':
            return self._detect_aruco(gray)
        if self.p['detection_source'] == 'internal':
            if self.internal_detector is None:
                return {}
            results = self.internal_detector.detect(gray, estimate_tag_pose=False)
            return {int(d.tag_id): np.asarray(d.corners, dtype=np.float64) for d in results}
        else:
            return dict(self.latest_apriltag_ros_detections)

    def _detect_aruco(self, gray: np.ndarray) -> Dict[int, np.ndarray]:
        """
        Returns {marker_id: corners(4,2) float64} using cv2.aruco, in
        ArUco's own corner order (top-left, top-right, bottom-right,
        bottom-left -- see ArucoMarkerGeometry's docstring). Every
        detected marker is returned here; filtering down to the one
        configured aruco_marker_id happens in _on_image alongside the
        AprilGrid ID-range check, so both paths share one validation
        step.
        """
        if self.aruco_detector is None:
            return {}
        corners, ids, _rejected = self.aruco_detector.detectMarkers(gray)
        if ids is None:
            return {}
        return {int(marker_id): c.reshape(4, 2).astype(np.float64)
                for c, marker_id in zip(corners, ids.flatten())}

    # ------------------------------------------------------------------
    # Main per-frame processing
    # ------------------------------------------------------------------
    def _on_image(self, msg: Image):
        if self.finalized:
            return

        if self.camera_model is None:
            self.get_logger().warn(
                'No CameraInfo received yet -- skipping frame.', throttle_duration_sec=5.0)
            return

        if msg.width != self.camera_model.width or msg.height != self.camera_model.height:
            if not self._resolution_error_logged:
                self.get_logger().error(
                    f'Image/CameraInfo resolution MISMATCH: '
                    f'Image is {msg.width}x{msg.height}, '
                    f'CameraInfo is {self.camera_model.width}x{self.camera_model.height}. '
                    'Refusing to calibrate with mismatched intrinsics. Check that '
                    'image_topic and camera_info_topic correspond to the same stream.')
                self._resolution_error_logged = True
            return

        try:
            bgr = self.bridge.imgmsg_to_cv2(msg, desired_encoding='bgr8')
        except Exception as exc:  # noqa: BLE001 - report and skip malformed frames
            self.get_logger().warn(f'cv_bridge conversion failed: {exc}',
                                    throttle_duration_sec=5.0)
            return

        gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)
        detections = self._detect_tags(gray)

        p = self.p
        is_grid = p['fiducial_type'] != 'aruco_single'
        rows, cols = p['tag_rows'], p['tag_cols']
        valid: Dict[int, np.ndarray] = {}
        invalid_ids: List[int] = []
        for tag_id, corners in detections.items():
            id_ok = (tag_id_to_row_col(tag_id, rows, cols) is not None if is_grid
                     else tag_id == p['aruco_marker_id'])
            if id_ok:
                valid[tag_id] = corners
            else:
                invalid_ids.append(tag_id)

        if invalid_ids:
            if is_grid:
                self.get_logger().warn(
                    f'Ignoring {len(invalid_ids)} detection(s) with unexpected tag ID(s) '
                    f'{sorted(invalid_ids)} outside the valid range '
                    f'[0, {rows * cols - 1}].', throttle_duration_sec=5.0)
            else:
                self.get_logger().warn(
                    f'Ignoring {len(invalid_ids)} ArUco detection(s) with ID(s) '
                    f'{sorted(invalid_ids)} != configured aruco_marker_id '
                    f"{p['aruco_marker_id']}.", throttle_duration_sec=5.0)

        debug_state = dict(
            bgr=bgr, valid=valid, accepted=False, status_text='NO POSE',
            height=None, tilt=None, downward_tilt=None, roll=None,
            rmse=None, mean_err=None, median_err=None, max_err=None,
            rvec=None, tvec=None, num_tags=len(valid), num_corners=4 * len(valid),
        )

        if len(valid) < self._effective_min_tags:
            self.get_logger().warn(
                f'Only {len(valid)} tag(s) detected, need at least '
                f'{self._effective_min_tags}. Skipping frame.', throttle_duration_sec=5.0)
            self.rejected_count += 1
            debug_state['status_text'] = f'REJECTED: only {len(valid)} tags'
            self._publish_debug_image(debug_state, msg.header)
            return

        object_points = np.vstack(
            [self.geometry.object_points_for_tag(tid) for tid in valid.keys()])
        image_points = np.vstack([corners for corners in valid.values()])

        pnp_result = estimate_board_pose(
            object_points, image_points, self.camera_model.K, self.camera_model.D,
            p['pnp_ransac_reprojection_error_px'])

        if pnp_result is None:
            self.get_logger().warn('PnP pose estimation failed for this frame.',
                                    throttle_duration_sec=5.0)
            self.rejected_count += 1
            debug_state['status_text'] = 'REJECTED: PnP failed'
            self._publish_debug_image(debug_state, msg.header)
            return

        rvec, tvec, inlier_idx = pnp_result
        debug_state['rvec'] = rvec
        debug_state['tvec'] = tvec

        # Reproject and score against ALL detected correspondences (not just
        # the RANSAC inlier set) so bad correspondences are actually reflected
        # in the accept/reject decision.
        errors = compute_reprojection_errors(
            object_points, image_points, rvec, tvec, self.camera_model.K, self.camera_model.D)
        mean_err = float(np.mean(errors))
        median_err = float(np.median(errors))
        rmse = float(np.sqrt(np.mean(errors ** 2)))
        max_err = float(np.max(errors))

        debug_state.update(mean_err=mean_err, median_err=median_err,
                            rmse=rmse, max_err=max_err)

        num_inliers = len(inlier_idx)
        self.get_logger().info(
            f'Detected tags: {len(valid)} / {self.geometry.num_tags()} | '
            f'Used corners: {len(image_points)} | PnP inliers: {num_inliers} | '
            f'Reprojection RMSE: {rmse:.3f} px')

        if rmse > p['max_reprojection_error_px']:
            self.get_logger().warn(
                f'Reprojection RMSE {rmse:.3f} px exceeds threshold '
                f'{p["max_reprojection_error_px"]:.3f} px -- REJECTED.',
                throttle_duration_sec=2.0)
            self.rejected_count += 1
            debug_state['status_text'] = f'REJECTED: RMSE {rmse:.2f}px'
            self._publish_debug_image(debug_state, msg.header)
            return

        R_board_camera, t_board_camera = invert_pnp_pose(rvec, tvec)
        height, tilt_deg, downward_tilt_deg, roll_deg = compute_height_tilt_roll(
            R_board_camera, t_board_camera)

        if not (p['min_expected_height_m'] <= height <= p['max_expected_height_m']):
            self.get_logger().warn(
                f'Estimated height {height:.4f} m is outside the expected range '
                f'[{p["min_expected_height_m"]:.3f}, {p["max_expected_height_m"]:.3f}] m '
                '-- REJECTED (likely a degenerate PnP solution).',
                throttle_duration_sec=2.0)
            self.rejected_count += 1
            debug_state.update(height=height, tilt=tilt_deg, downward_tilt=downward_tilt_deg,
                                roll=roll_deg,
                                status_text=f'REJECTED: height {height:.2f}m out of range')
            self._publish_debug_image(debug_state, msg.header)
            return

        if tilt_deg < p['edge_on_tilt_warning_deg']:
            self.get_logger().warn(
                f'AprilGrid viewed nearly edge-on (tilt={tilt_deg:.1f} deg) -- pose '
                'estimate may be poorly conditioned.', throttle_duration_sec=5.0)

        if self._prev_accepted is not None:
            dh = abs(height - self._prev_accepted.height_m)
            dt = abs(downward_tilt_deg - self._prev_accepted.downward_tilt_deg)
            if dh > p['max_frame_height_jump_m']:
                self.get_logger().warn(
                    f'Height jumped {dh:.4f} m between consecutive accepted frames '
                    f'(> {p["max_frame_height_jump_m"]:.4f} m) -- camera or board may '
                    'have moved.', throttle_duration_sec=2.0)
            if dt > p['max_frame_tilt_jump_deg']:
                self.get_logger().warn(
                    f'Tilt jumped {dt:.3f} deg between consecutive accepted frames '
                    f'(> {p["max_frame_tilt_jump_deg"]:.3f} deg) -- camera or board may '
                    'have moved.', throttle_duration_sec=2.0)

        sample = FrameSample(
            height_m=height, tilt_deg=tilt_deg, downward_tilt_deg=downward_tilt_deg,
            roll_deg=roll_deg, reprojection_rmse_px=rmse,
            num_tags=len(valid), num_corners=len(image_points),
            R_board_camera=R_board_camera, t_board_camera=t_board_camera,
        )
        self.samples.append(sample)
        self._prev_accepted = sample

        self.get_logger().info(
            f'Height: {height:.4f} m | Tilt: {downward_tilt_deg:.2f} deg | '
            f'Roll: {roll_deg:.2f} deg | Reprojection RMSE: {rmse:.3f} px | ACCEPTED '
            f'({len(self.samples)}/{p["samples_required"]})')

        debug_state.update(accepted=True, height=height, tilt=tilt_deg,
                            downward_tilt=downward_tilt_deg, roll=roll_deg,
                            status_text='ACCEPTED')

        self._publish_scalars(height, downward_tilt_deg, roll_deg, rmse)
        self._publish_pose(R_board_camera, t_board_camera, msg.header)
        if self.tf_broadcaster is not None:
            self._publish_tf(R_board_camera, t_board_camera, msg.header)
        self._publish_debug_image(debug_state, msg.header)

        if len(self.samples) >= p['samples_required']:
            self._finalize_calibration()

    # ------------------------------------------------------------------
    # Publishing
    # ------------------------------------------------------------------
    def _publish_scalars(self, height, downward_tilt_deg, roll_deg, rmse):
        self.height_pub.publish(Float64(data=float(height)))
        # NOTE: the signed downward tilt (positive = pitched toward the
        # floor) is published on /tilt, matching the "downward tilt" that
        # is the primary quantity of interest for ground-plane estimation.
        # The unsigned tilt magnitude is available in the debug image and
        # the final YAML / console report.
        self.tilt_pub.publish(Float64(data=float(downward_tilt_deg)))
        self.roll_pub.publish(Float64(data=float(roll_deg)))
        self.reproj_pub.publish(Float64(data=float(rmse)))

    def _publish_pose(self, R_board_camera, t_board_camera, header):
        msg = PoseStamped()
        msg.header.stamp = header.stamp
        msg.header.frame_id = self.p['board_frame_id']
        msg.pose.position.x = float(t_board_camera[0, 0])
        msg.pose.position.y = float(t_board_camera[1, 0])
        msg.pose.position.z = float(t_board_camera[2, 0])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_board_camera)
        msg.pose.orientation.x = qx
        msg.pose.orientation.y = qy
        msg.pose.orientation.z = qz
        msg.pose.orientation.w = qw
        self.pose_pub.publish(msg)

    def _build_transform_stamped(self, R_board_camera, t_board_camera, stamp) -> TransformStamped:
        # We deliberately publish ONLY board/ground -> camera_optical. We do
        # NOT publish anything like base_link -> camera, because the
        # AprilGrid only defines the vertical (height/tilt/roll) relationship
        # to the floor -- its horizontal position and yaw on the floor are
        # arbitrary (wherever it happened to be placed), so a full 6-DoF
        # base_link transform derived from it would be misleading.
        t = TransformStamped()
        t.header.stamp = stamp
        t.header.frame_id = self.p['board_frame_id']
        t.child_frame_id = self.p['camera_optical_frame_id']
        t.transform.translation.x = float(t_board_camera[0, 0])
        t.transform.translation.y = float(t_board_camera[1, 0])
        t.transform.translation.z = float(t_board_camera[2, 0])
        qx, qy, qz, qw = rotation_matrix_to_quaternion(R_board_camera)
        t.transform.rotation.x = qx
        t.transform.rotation.y = qy
        t.transform.rotation.z = qz
        t.transform.rotation.w = qw
        return t

    def _publish_tf(self, R_board_camera, t_board_camera, header):
        """Per-accepted-frame transform on the regular (non-latched) /tf."""
        t = self._build_transform_stamped(R_board_camera, t_board_camera, header.stamp)
        self.tf_broadcaster.sendTransform(t)

    def _publish_static_tf(self, R_board_camera, t_board_camera):
        """
        Publish the FINAL, robust-median calibration result once, on the
        latched /tf_static topic, so late-joining subscribers (e.g. a
        ground-plane estimator started after calibration finished) still
        receive it. Overwrites/replaces any transform previously sent by
        this broadcaster between the same two frame ids.
        """
        t = self._build_transform_stamped(
            R_board_camera, t_board_camera, self.get_clock().now().to_msg())
        self.static_tf_broadcaster.sendTransform(t)

    def _publish_debug_image(self, state: dict, header):
        img = self._draw_debug_image(state)
        try:
            out_msg = bgr8_to_imgmsg(img, header)
            self.debug_image_pub.publish(out_msg)
        except Exception as exc:  # noqa: BLE001
            self.get_logger().warn(f'Failed to publish debug image: {exc}',
                                    throttle_duration_sec=5.0)

        if self.p['show_debug_image']:
            cv2.imshow('aprilgrid_ground_calibrator', img)
            cv2.waitKey(1)

    # ------------------------------------------------------------------
    # Debug visualization
    # ------------------------------------------------------------------
    def _draw_debug_image(self, state: dict) -> np.ndarray:
        img = state['bgr'].copy()
        is_grid = self.p['fiducial_type'] != 'aruco_single'
        rows, cols = self.p['tag_rows'], self.p['tag_cols']

        for tag_id, corners in state['valid'].items():
            pts = corners.astype(np.int32)
            cv2.polylines(img, [pts.reshape(-1, 1, 2)], True, (0, 255, 0), 2)
            for i, (x, y) in enumerate(corners):
                cv2.circle(img, (int(x), int(y)), 3, (0, 0, 255), -1)
                cv2.putText(img, str(i), (int(x) + 4, int(y) - 4),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.35, (0, 0, 255), 1, cv2.LINE_AA)
            center = corners.mean(axis=0).astype(int)
            if is_grid:
                row, col = tag_id_to_row_col(tag_id, rows, cols)
                label = f'id{tag_id} r{row}c{col}'
            else:
                label = f'ArUco id={tag_id}'
            cv2.putText(img, label, (center[0] - 25, center[1]),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 0), 1, cv2.LINE_AA)

        if state['rvec'] is not None and self.camera_model is not None:
            try:
                cv2.drawFrameAxes(img, self.camera_model.K, self.camera_model.D,
                                   state['rvec'], state['tvec'], self.p['axis_length_m'])
            except cv2.error:
                pass

        lines = [
            f"Tags: {state['num_tags']}  Corners: {state['num_corners']}",
        ]
        if state['height'] is not None:
            lines.append(f"Height: {state['height']:.4f} m")
            lines.append(f"Tilt: {state['downward_tilt']:.2f} deg "
                         f"(mag {state['tilt']:.2f} deg)")
            lines.append(f"Roll: {state['roll']:.2f} deg")
        if state['rmse'] is not None:
            lines.append(f"Reprojection RMSE: {state['rmse']:.3f} px "
                         f"(max {state['max_err']:.3f} px)")
        lines.append(f"Samples: {len(self.samples)}/{self.p['samples_required']}  "
                     f"Rejected: {self.rejected_count}")
        lines.append(state['status_text'])

        y0 = 20
        color = (0, 255, 0) if state['accepted'] else (0, 0, 255)
        for i, text in enumerate(lines):
            y = y0 + i * 20
            line_color = color if text == state['status_text'] else (255, 255, 255)
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        (0, 0, 0), 3, cv2.LINE_AA)
            cv2.putText(img, text, (10, y), cv2.FONT_HERSHEY_SIMPLEX, 0.5,
                        line_color, 1, cv2.LINE_AA)

        return img

    # ------------------------------------------------------------------
    # Timeout watchdog
    # ------------------------------------------------------------------
    def _check_timeout(self):
        """
        Runs on a plain wall-clock timer (independent of _on_image), so a
        misconfigured/empty pipeline still terminates: wrong image_topic,
        nothing publishing on it, the AprilGrid never in frame, etc. would
        otherwise never even reach _on_image's own reject-path logging,
        and the node would wait forever with no report and no output YAML.
        """
        if self.finalized:
            return
        elapsed = time.monotonic() - self._start_time
        if elapsed >= self.p['calibration_timeout_sec']:
            self._finalize_calibration(timed_out=True)

    # ------------------------------------------------------------------
    # Final aggregation, report, and YAML output
    # ------------------------------------------------------------------
    def _finalize_calibration(self, timed_out: bool = False):
        self.finalized = True
        if self.timeout_timer is not None:
            self.timeout_timer.cancel()

        p = self.p
        log = self.get_logger().info

        if timed_out:
            log(f"calibration_timeout_sec ({p['calibration_timeout_sec']:.0f}s) elapsed with "
                f"only {len(self.samples)}/{p['samples_required']} accepted samples "
                f"({self.rejected_count} rejected). Finalizing with partial results "
                "instead of waiting indefinitely.")

        if not self.samples:
            log('=' * 60)
            log('APRILGRID CAMERA-TO-GROUND CALIBRATION: FAILED')
            log('=' * 60)
            log('No frames were ever accepted -- there is nothing to calibrate.')
            log('Most likely causes:')
            log('  - the AprilGrid is not in the camera field of view')
            log('  - the board is too small/far/blurry in-frame to detect '
                '(try moving it closer, or use a higher-resolution image_topic)')
            log("  - tag_family does not match the physical board (currently "
                f"'{p['tag_family']}')")
            log('  - image_topic / camera_info_topic point at the wrong stream, '
                'or nothing is publishing on them')
            log('  - the board is not flat on the floor and fully in frame')
            log('=' * 60)
            self._write_yaml(None, None, None, None, None, None, float('nan'),
                              'FAILED', ['no accepted frames were collected'])
            log('Calibration ended without data. Node remains alive; press Ctrl+C to exit.')
            return

        heights = [s.height_m for s in self.samples]
        tilts = [s.downward_tilt_deg for s in self.samples]
        rolls = [s.roll_deg for s in self.samples]
        rmses = [s.reprojection_rmse_px for s in self.samples]
        num_tags_list = [s.num_tags for s in self.samples]
        num_corners_list = [s.num_corners for s in self.samples]

        h_med, h_mean, h_std, _, h_rej = robust_stats(heights)
        t_med, t_mean, t_std, _, t_rej = robust_stats(tilts)
        r_med, r_mean, r_std, _, r_rej = robust_stats(rolls)
        rmse_med = float(np.median(rmses))

        median_tags = float(np.median(num_tags_list))
        median_corners = float(np.median(num_corners_list))

        status, reasons = classify_calibration(
            accepted=len(self.samples), rejected=self.rejected_count,
            height_std=h_std, tilt_std=t_std, roll_std=r_std,
            reproj_rmse_median=rmse_med,
            max_height_std_m=p['max_height_std_m'],
            max_tilt_std_deg=p['max_tilt_std_deg'],
            max_roll_std_deg=p['max_roll_std_deg'],
            max_reprojection_error_px=p['max_reprojection_error_px'],
        )

        # Pick the single accepted frame whose (height, tilt, roll) is
        # closest to the robust medians computed above, and use ITS real
        # R_board_camera/t_board_camera for the static TF below. We do NOT
        # synthesize a rotation matrix from the scalar median height/tilt/
        # roll: those three numbers under-determine a full 3-DoF rotation
        # (yaw about the vertical axis is intentionally never tracked, see
        # the TF section of __init__), so "closest real sample" is the only
        # geometrically honest way to get a representative R/t pair.
        representative = select_representative_sample(
            self.samples, h_med, h_std, t_med, t_std, r_med, r_std)

        log = self.get_logger().info
        log('=' * 60)
        log('APRILGRID CAMERA-TO-GROUND CALIBRATION')
        log('=' * 60)
        log('')
        if p['fiducial_type'] == 'aruco_single':
            log('Target:')
            log('    type               : single ArUco marker')
            log(f"    dictionary         : {p['aruco_dictionary']}")
            log(f"    marker id          : {p['aruco_marker_id']}")
            log(f"    marker size        : {p['tag_size']:.4f} m")
        else:
            log('Board:')
            log(f"    rows               : {p['tag_rows']}")
            log(f"    columns            : {p['tag_cols']}")
            log(f"    tag size           : {p['tag_size']:.4f} m")
            log(f"    tag spacing        : {p['tag_spacing']:.4f} m")
            log(f"    tag pitch          : {self.geometry.tag_pitch:.4f} m")
        log('')
        log('Samples:')
        log(f"    requested          : {p['samples_required']}")
        log(f'    accepted           : {len(self.samples)}')
        log(f'    rejected           : {self.rejected_count}')
        log('')
        log('Tag detections:')
        log(f'    median tags/frame  : {median_tags:.0f}')
        log(f'    median corners     : {median_corners:.0f}')
        log('')
        log('Reprojection:')
        log(f'    median RMSE        : {rmse_med:.2f} px')
        log('')
        log('CAMERA HEIGHT:')
        log(f'    median             : {h_med:.4f} m')
        log(f'    mean               : {h_mean:.4f} m')
        log(f'    std                : {h_std:.4f} m ({h_rej} outlier(s) rejected)')
        log('')
        log('CAMERA DOWNWARD TILT:')
        log(f'    median             : {t_med:.2f} deg')
        log(f'    mean               : {t_mean:.2f} deg')
        log(f'    std                : {t_std:.2f} deg ({t_rej} outlier(s) rejected)')
        log('')
        log('CAMERA ROLL:')
        log(f'    median             : {r_med:.2f} deg')
        log(f'    mean               : {r_mean:.2f} deg')
        log(f'    std                : {r_std:.2f} deg ({r_rej} outlier(s) rejected)')
        log('')
        log('=' * 60)
        log('FINAL VALUES')
        log('=' * 60)
        log('')
        log(f'camera_height_m = {h_med:.4f}')
        log(f'camera_tilt_deg = {t_med:.2f}')
        log(f'camera_roll_deg = {r_med:.2f}')
        log('')
        log(f'CALIBRATION STATUS: {status}')
        for reason in reasons:
            log(f'    - {reason}')
        log('=' * 60)

        if self.static_tf_broadcaster is not None:
            self._publish_static_tf(representative.R_board_camera, representative.t_board_camera)
            log(f"Published final /tf_static "
                f"'{p['board_frame_id']}' -> '{p['camera_optical_frame_id']}' "
                f"from the accepted frame closest to the robust median "
                f"(height diff {abs(representative.height_m - h_med) * 1000:.2f} mm, "
                f"tilt diff {abs(representative.downward_tilt_deg - t_med):.3f} deg, "
                f"roll diff {abs(representative.roll_deg - r_med):.3f} deg). "
                f"This is latched -- late-joining subscribers still receive it.")

        self._write_yaml(h_med, h_std, t_med, t_std, r_med, r_std, rmse_med, status, reasons)
        log('Calibration complete. Node remains alive (publishing stops); '
            'press Ctrl+C to exit.')

    def _write_yaml(self, h_med, h_std, t_med, t_std, r_med, r_std, rmse_med, status, reasons):
        def safe_round(x, n):
            if x is None or (isinstance(x, float) and math.isnan(x)):
                return None
            return round(x, n)

        p = self.p
        cm = self.camera_model
        data = {
            'camera_ground_calibration': {
                'height_m': safe_round(h_med, 6),
                'downward_tilt_deg': safe_round(t_med, 4),
                'roll_deg': safe_round(r_med, 4),

                'height_std_m': safe_round(h_std, 6),
                'tilt_std_deg': safe_round(t_std, 4),
                'roll_std_deg': safe_round(r_std, 4),

                'reprojection_rmse_px': safe_round(rmse_med, 4),

                'status': status,
                'status_reasons': reasons,

                'samples_accepted': len(self.samples),
                'samples_rejected': self.rejected_count,

                'fiducial_type': p['fiducial_type'],

                'camera': {
                    'width': cm.width if cm else None,
                    'height': cm.height if cm else None,
                    'fx': float(cm.K[0, 0]) if cm else None,
                    'fy': float(cm.K[1, 1]) if cm else None,
                    'cx': float(cm.K[0, 2]) if cm else None,
                    'cy': float(cm.K[1, 2]) if cm else None,
                    'distortion_model': cm.distortion_model if cm else None,
                },
            }
        }

        if p['fiducial_type'] == 'aruco_single':
            data['camera_ground_calibration']['aruco_marker'] = {
                'dictionary': p['aruco_dictionary'],
                'marker_id': p['aruco_marker_id'],
                'marker_size_m': p['tag_size'],
            }
        else:
            data['camera_ground_calibration']['aprilgrid'] = {
                'rows': p['tag_rows'],
                'cols': p['tag_cols'],
                'tag_size_m': p['tag_size'],
                'tag_spacing_m': p['tag_spacing'],
                'tag_family': p['tag_family'],
            }

        out_path = p['output_yaml_path']
        try:
            with open(out_path, 'w') as f:
                yaml.safe_dump(data, f, default_flow_style=False, sort_keys=False)
            self.get_logger().info(f'Wrote calibration result to: {out_path}')
        except OSError as exc:
            self.get_logger().error(f'Failed to write output YAML to {out_path}: {exc}')


def main(args=None):
    rclpy.init(args=args)
    node = AprilGridGroundCalibratorNode()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        # KeyboardInterrupt: local Ctrl+C (SIGINT) during rclpy.spin().
        # ExternalShutdownException: rclpy's own signal handler already
        # called rclpy.shutdown() for us (e.g. `ros2 launch` sending
        # SIGTERM, or `timeout`) -- exit quietly instead of printing a
        # traceback either way.
        pass
    finally:
        cv2.destroyAllWindows()
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
