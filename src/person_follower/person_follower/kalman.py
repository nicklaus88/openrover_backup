"""A minimal constant-velocity Kalman filter for smoothing a tracked
person's 3D position.

The original monocular_person_following stack (monocular_people_tracking)
used a full Unscented Kalman Filter over a ground-plane-constrained
observation model, because it only had a monocular camera and had to
*infer* depth from the ankle-to-ground projection. We have real stereo
depth from the OAK-D, so a plain linear constant-velocity KF over
(x, y, z, vx, vy, vz) is sufficient and much simpler to reason about.

Pure python / numpy, no ROS dependency, so it is unit-testable in
isolation (see test/test_kalman.py).
"""
import numpy as np


class ConstantVelocityKalmanFilter:
    """6-state (position + velocity) constant-velocity KF in 3D."""

    def __init__(self, initial_position, process_noise=0.05, measurement_noise=0.05):
        self.state = np.zeros(6)
        self.state[0:3] = initial_position

        self.P = np.eye(6) * 1.0

        self.process_noise = process_noise
        self.measurement_noise = measurement_noise

        self.H = np.zeros((3, 6))
        self.H[0, 0] = 1.0
        self.H[1, 1] = 1.0
        self.H[2, 2] = 1.0

    def predict(self, dt):
        if dt <= 0.0:
            return

        F = np.eye(6)
        F[0, 3] = dt
        F[1, 4] = dt
        F[2, 5] = dt

        # Simple constant process noise scaled by dt; good enough for a
        # person walking at human speed with a ~10-30Hz camera.
        Q = np.eye(6) * (self.process_noise * dt)

        self.state = F @ self.state
        self.P = F @ self.P @ F.T + Q

    def update(self, measured_position):
        z = np.asarray(measured_position, dtype=float)
        R = np.eye(3) * self.measurement_noise

        y = z - self.H @ self.state
        S = self.H @ self.P @ self.H.T + R
        K = self.P @ self.H.T @ np.linalg.inv(S)

        self.state = self.state + K @ y
        self.P = (np.eye(6) - K @ self.H) @ self.P

    @property
    def position(self):
        return self.state[0:3].copy()

    @property
    def velocity(self):
        return self.state[3:6].copy()
