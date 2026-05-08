from __future__ import division

import numpy as np


class KinematicCarMotionModel:
    """The kinematic car motion model."""

    def __init__(self, **kwargs):
        """Initialize the kinematic car motion model.
        """
        required = {"vel_bias", "vel_std", "delta_bias", "delta_std", "x_bias", "x_std", "x_vel_std", "y_bias", "y_std",
                    "y_vel_std", "theta_bias", "theta_std", "car_length", "car_width", "car_wheel_radius",
                    "steering_to_servo_offset", "steering_to_servo_gain"
                    }
        if not set(kwargs) == required:
            raise ValueError("Invalid keyword argument provided")
        # These next two lines set the instance attributes from the defaults and
        # kwargs dictionaries. For example, the key "vel_std" becomes the
        # instance attribute self.vel_std.
        self.__dict__.update(kwargs)

        if self.car_length <= 0.0:
            raise ValueError(
                "The model is only defined for positive, non-zero car lengths"
            )

    def compute_changes(self, states, controls, dt, delta_threshold=1e-2):
        """Integrate the (deterministic) kinematic car model.

        Given vectorized states and controls, compute the changes in state when
        applying the control for duration dt.

        If the absolute value of the applied delta is below delta_threshold,
        round down to 0. We assume that the steering angle (and therefore the
        orientation component of state) does not change in this case.

        Args:
            states: np.array of states with shape M x 3
            controls: np.array of controls with shape M x 2
            dt (float): control duration

        Returns:
            M x 3 np.array, where the three columns are dx, dy, dtheta
            M x 4 np.array, where the four columns are left throttle, right throttle, left steer, right steer joint states

        """
        car_length, car_width, wheel_radius = self.car_length, self.car_width, self.car_wheel_radius
        changes = np.empty_like(states, dtype=float)
        num_states = states.shape[0]

        vel, delta = controls[:, 0], controls[:, 1]
        # print(vel, delta, dt)
        theta = states[:, 2]

        changes[:, 2] = (vel / car_length) * np.tan(delta) * dt
        changes[np.abs(delta) < delta_threshold, 2] = 0
        dtheta = changes[:, 2]
        changes[:, 0] = vel * np.cos(theta) * dt
        changes[:, 1] = vel * np.sin(theta) * dt

        # print(changes[0][0])

        val_indices = np.abs(delta) >= delta_threshold
        val_theta = theta[val_indices]
        val_new_theta = val_theta + dtheta[val_indices]
        tan_delta = np.tan(delta)
        share_value = car_length / tan_delta[val_indices]
        changes[val_indices, 0] = share_value * (
                np.sin(val_new_theta) - np.sin(val_theta)
        )
        changes[val_indices, 1] = share_value * (
                np.cos(val_theta) - np.cos(val_new_theta)
        )
        # New joint values
        half_width = 0.5 * car_width
        # Applt kinematic car model to compute wheel deltas
        h = np.zeros_like(theta)
        h[val_indices] = (car_length / tan_delta[val_indices]) - (car_width / 2.0)

        joint_outer_throttle = (
                ((car_width + h) / (half_width + h))
                * vel
                * dt
                / wheel_radius
        )
        joint_inner_throttle = (
                ((h) / (half_width + h))
                * vel
                * dt
                / wheel_radius
        )
        joint_outer_steer = np.arctan2(car_length, car_width + h)
        joint_inner_steer = np.arctan2(car_length, h)

        # New joint values
        joint_changes = np.empty((num_states, 4), dtype=float)
        joint_changes[:, :2] = vel * dt / wheel_radius
        joint_changes[:, 2:] = 0

        # Assign joint values according to whether we are turning left or right
        right_ind = np.logical_and(val_indices, delta > 0.0)
        joint_changes[right_ind, 0] = joint_inner_throttle
        joint_changes[right_ind, 1] = joint_outer_throttle
        joint_changes[right_ind, 2] = joint_inner_steer
        joint_changes[right_ind, 3] = joint_outer_steer

        # Left turn
        left_ind = np.logical_and(val_indices, delta < 0.0)
        joint_changes[left_ind, 0] = joint_outer_throttle[left_ind]
        joint_changes[left_ind, 1] = joint_inner_throttle[left_ind]
        joint_changes[left_ind, 2] = joint_outer_steer[left_ind] - np.pi
        joint_changes[left_ind, 3] = joint_inner_steer[left_ind] - np.pi

        return changes, joint_changes

    def apply_motion_model(self, states, nominal_controls, dt):
        """Propagate states through the noisy kinematic car motion model.

        Given the nominal control (vel, delta), sample M noisy controls.
        Then, compute the changes in state with the noisy controls.
        Finally, add noise to the resulting states.

        Returns the changes (deltas) applied

        >>> states = np.ones((3, 2))
        >>> states[2, :] = np.arange(2)  #  modifies the row at index 2
        >>> a = np.array([[1, 2], [3, 4], [5, 6]])
        >>> states[:] = a + a            # modifies states; note the [:]

        Args:
            states: np.array of states with shape M x 3
            vel (float): nominal control velocity
            delta (float): nominal control steering angle
            dt (float): control duration
        """
        n_states = states.shape[0]

        controls = np.random.normal(
            nominal_controls + [self.vel_bias, self.delta_bias],
            scale=[self.vel_std, self.delta_std],
            size=(n_states, 2),
            )
        state_changes, joint_changes = self.compute_changes(states, controls, dt)
        # We have two types of noise
        # Fixed noise that's independent of controls
        state_changes += np.random.normal([self.x_bias, self.y_bias, self.theta_bias],
                                          scale=[self.x_std, self.y_std, self.theta_std])
        # and scaled noise that's proportional to the velocity
        state_changes[:, [0, 1]] += np.random.normal([0, 0], scale=np.abs(controls) * [self.x_vel_std, self.y_vel_std])

        # Apply the change in-place (numpy += is in-place)
        states += state_changes

        # First ,normalize to [-pi, pi)
        states[:, 2] = (states[:, 2] + np.pi) % (2 * np.pi) - np.pi
        # Now fix to (-pi, pi]
        states[states[:, 2] == -np.pi, 2] = np.pi
        return state_changes, joint_changes
