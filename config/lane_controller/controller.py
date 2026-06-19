import numpy as np


class LaneController:
    """
    The Lane Controller can be used to compute control commands from pose estimations.

    Enhanced with lane boundary enforcement (never cross yellow/white lines)
    and obstacle-aware rightward offset.

    Args:
        ~v_bar (:obj:`float`): Nominal velocity in m/s
        ~k_d (:obj:`float`): Proportional term for lateral deviation
        ~k_theta (:obj:`float`): Proportional term for heading deviation
        ~k_Id (:obj:`float`): integral term for lateral deviation
        ~k_Iphi (:obj:`float`): integral term for heading deviation
        ~k_boundary (:obj:`float`): Gain for lane boundary repulsive force
        ~d_thres (:obj:`float`): Maximum value for lateral error
        ~theta_thres (:obj:`float`): Maximum value for heading error
        ~d_offset (:obj:`float`): Goal offset from center of the lane
        ~integral_bounds (:obj:`dict`): Bounds for integral term
        ~d_resolution (:obj:`float`): Resolution of lateral position estimate
        ~phi_resolution (:obj:`float`): Resolution of heading estimate
        ~omega_ff (:obj:`float`): Feedforward part of controller
        ~verbose (:obj:`bool`): Verbosity level (0,1,2)
        ~stop_line_slowdown (:obj:`dict`): Start and end distances for slowdown at stop lines
        ~boundary_yellow (:obj:`float`): d value for yellow line (left) boundary
        ~boundary_white (:obj:`float`): d value for white line (right) boundary
        ~boundary_margin (:obj:`float`): Soft zone margin before boundary
    """

    def __init__(self, parameters):
        self.parameters = parameters
        self.d_I = 0.0
        self.phi_I = 0.0
        self.prev_d_err = 0.0
        self.prev_phi_err = 0.0

    def update_parameters(self, parameters):
        self.parameters = parameters

    def compute_control_action(self, d_err, phi_err, dt, wheels_cmd_exec, stop_line_distance):
        if dt is not None:
            self.integrate_errors(d_err, phi_err, dt)

        self.d_I = self.adjust_integral(
            d_err, self.d_I, self.parameters["~integral_bounds"]["d"], self.parameters["~d_resolution"]
        )
        self.phi_I = self.adjust_integral(
            phi_err,
            self.phi_I,
            self.parameters["~integral_bounds"]["phi"],
            self.parameters["~phi_resolution"],
        )

        self.reset_if_needed(d_err, phi_err, wheels_cmd_exec)

        omega = (
            self.parameters["~k_d"].value * d_err
            + self.parameters["~k_theta"].value * phi_err
            + self.parameters["~k_Id"].value * self.d_I
            + self.parameters["~k_Iphi"].value * self.phi_I
        )

        # Lane boundary enforcement
        omega += self._compute_boundary_correction(d_err)

        self.prev_d_err = d_err
        self.prev_phi_err = phi_err

        v = self.compute_velocity(stop_line_distance)

        return v, omega

    def _compute_boundary_correction(self, d_err):
        boundary_yellow = self.parameters.get("~boundary_yellow", -0.09)
        boundary_white = self.parameters.get("~boundary_white", 0.09)
        margin = self.parameters.get("~boundary_margin", 0.03)
        k_boundary = self.parameters.get("~k_boundary", 8.0)

        correction = 0.0

        # Yellow line (left): d_err negative means too far left
        # When approaching yellow line, add positive omega (turn right)
        if d_err < boundary_yellow + margin:
            penetration = d_err - boundary_yellow
            if penetration < 0:
                penetration = max(penetration, -0.05)
                correction += -k_boundary * penetration

        # White line (right): d_err positive means too far right
        # When approaching white line, add negative omega (turn left)
        if d_err > boundary_white - margin:
            penetration = d_err - boundary_white
            if penetration > 0:
                penetration = min(penetration, 0.05)
                correction += -k_boundary * penetration

        return correction

    def compute_velocity(self, stop_line_distance):
        if stop_line_distance is None:
            return self.parameters["~v_bar"].value
        else:
            d1, d2 = (
                self.parameters["~stop_line_slowdown"]["start"],
                self.parameters["~stop_line_slowdown"]["end"],
            )
            c = (0.5 * (d1 - stop_line_distance) + (stop_line_distance - d2)) / (d1 - d2)
            v_new = self.parameters["~v_bar"].value * c
            v = np.max(
                [self.parameters["~v_bar"].value / 2.0, np.min([self.parameters["~v_bar"].value, v_new])]
            )
            return v

    def integrate_errors(self, d_err, phi_err, dt):
        self.d_I += d_err * dt
        self.phi_I += phi_err * dt

    def reset_if_needed(self, d_err, phi_err, wheels_cmd_exec):
        if np.sign(d_err) != np.sign(self.prev_d_err):
            self.d_I = 0
        if np.sign(phi_err) != np.sign(self.prev_phi_err):
            self.phi_I = 0
        if wheels_cmd_exec[0] == 0 and wheels_cmd_exec[1] == 0:
            self.d_I = 0
            self.phi_I = 0

    @staticmethod
    def adjust_integral(error, integral, bounds, resolution):
        if integral > bounds["top"]:
            integral = bounds["top"]
        elif integral < bounds["bot"]:
            integral = bounds["bot"]
        elif abs(error) < resolution:
            integral = 0
        return integral
