import numpy as np

from person_follower.kalman import ConstantVelocityKalmanFilter


def test_converges_to_stationary_position():
    kf = ConstantVelocityKalmanFilter(initial_position=[0.0, 0.0, 0.0])
    target = np.array([1.0, 2.0, 3.0])

    for _ in range(50):
        kf.predict(dt=0.1)
        kf.update(target)

    assert np.allclose(kf.position, target, atol=0.05)


def test_tracks_constant_velocity():
    kf = ConstantVelocityKalmanFilter(initial_position=[0.0, 0.0, 1.0])
    velocity = np.array([0.5, 0.0, 0.0])
    pos = np.array([0.0, 0.0, 1.0])
    dt = 0.1

    for _ in range(80):
        pos = pos + velocity * dt
        kf.predict(dt=dt)
        kf.update(pos)

    assert np.allclose(kf.velocity, velocity, atol=0.1)
    assert np.allclose(kf.position, pos, atol=0.1)


def test_predict_with_nonpositive_dt_is_noop():
    kf = ConstantVelocityKalmanFilter(initial_position=[1.0, 1.0, 1.0])
    before = kf.position.copy()
    kf.predict(dt=0.0)
    kf.predict(dt=-1.0)
    assert np.allclose(kf.position, before)
