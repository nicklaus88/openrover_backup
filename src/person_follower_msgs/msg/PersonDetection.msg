# A single person detected in one camera frame, before any tracking/ID has
# been assigned. Roughly the ROS2 analog of what tf-pose-estimation's
# `Person` message provided to monocular_people_tracking upstream.

# COCO-17 keypoints, in pixel coordinates of the source image.
# Order: nose, l_eye, r_eye, l_ear, r_ear,
#        l_shoulder, r_shoulder, l_elbow, r_elbow, l_wrist, r_wrist,
#        l_hip, r_hip, l_knee, r_knee, l_ankle, r_ankle
float32[17] keypoints_x
float32[17] keypoints_y
float32[17] keypoints_score   # 0 if not detected/low confidence

# Pixel-space bounding box of the person.
float32 bbox_x
float32 bbox_y
float32 bbox_w
float32 bbox_h

# Overall detection confidence from the pose model.
float32 score

# 3D position of the person's torso, back-projected from depth, in the
# camera optical frame. NaN in any field if depth was unavailable/invalid
# at the torso keypoints.
geometry_msgs/Point position
