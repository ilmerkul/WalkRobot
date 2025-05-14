from math import acos, atan, cos, pi, sin
from typing import List, Tuple

from control.utils import get_joints
from description.utils import parse_file_xacro_constants

zero_angles: List[float] = [0.0, 0.0, 0.0, 0.0]


class InverseKinematics:

    def __init__(
        self,
        phi1: List[float] = zero_angles,
        phi2: List[float] = zero_angles,
    ):

        self.constants = parse_file_xacro_constants()
        self.joint_order = get_joints()

        self.l1 = self.constants["front_leg_length1"]
        self.l2 = self.constants["front_leg_length2"]

        self.phi1_border = (
            self.constants["leg1_angle_lower_x"],
            self.constants["leg1_angle_upper_x"],
        )
        self.phi2_border = (
            self.constants["leg2_angle_lower_x"],
            self.constants["leg2_angle_upper_x"],
        )

        self.set_phi(phi1=phi1, phi2=phi2)

        self._assert_phi_border(self.phi1_border)
        self._assert_phi_border(self.phi2_border)

    def _assert_phi(self, phi: float):
        assert -pi <= phi <= pi

    def _assert_phi_border(self, phi_border: Tuple[float, float]):
        self._assert_phi(phi_border[0])
        self._assert_phi(phi_border[1])
        assert phi_border[0] < phi_border[1]

    def set_phi(
        self, phi1: List[float] = zero_angles, phi2: List[float] = zero_angles
    ) -> None:
        self.update_phi(phi1, phi2)

        self.delta_phi1 = zero_angles
        self.delta_phi2 = zero_angles

        return None

    def r_clip(
        self, r: List[float], x: List[float], z: List[float]
    ) -> Tuple[List[float], List[float], List[float]]:
        r_new = r
        for i, r_i in enumerate(r):
            if r_i < abs(self.l1 - self.l2):
                r_new[i] = abs(self.l1 - self.l2)
            elif r_i > (self.l1 + self.l2):
                r_new[i] = self.l1 + self.l2

        x = [x_i * r_new_i / r_i for x_i, r_new_i, r_i in zip(x, r_new, r)]
        z = [z_i * r_new_i / r_i for z_i, r_new_i, r_i in zip(z, r_new, r)]

        return r_new, x, z

    def update_x_z(self, x: List[float], z: List[float]) -> None:
        if any(x_i == 0 for x_i in x) or any(z_i == 0 for z_i in z):
            raise ValueError

        r = [(x_i**2 + z_i**2) ** 0.5 for x_i, z_i in zip(x, z)]

        r, x, z = self.r_clip(r, x, z)

        self.x = x
        self.z = z

        phi1_new, phi2_new = self.get_phi()
        phi1_new, clip = InverseKinematics.clip_phi(phi1_new, self.phi1_border)
        if clip:
            raise ValueError
        phi1_new, clip = InverseKinematics.clip_phi(phi1_new, self.phi2_border)
        if clip:
            raise ValueError

        self.delta_phi1 = [phi_new - phi for phi_new, phi in zip(phi1_new, self.phi1)]
        self.delta_phi2 = [phi_new - phi for phi_new, phi in zip(phi2_new, self.phi2)]

        self.phi1 = phi1_new
        self.phi2 = phi2_new

        return None

    def update_phi(self, phi1: List[float], phi2: List[float]) -> None:
        self.phi1, _ = InverseKinematics.clip_phi(phi1, self.phi1_border)
        self.phi2, _ = InverseKinematics.clip_phi(phi2, self.phi2_border)

        self.x, self.z = self.get_x_z()

        return None

    def get_x_z(self) -> Tuple[List[float], List[float]]:
        angle1, angle2 = self.get_horizont_angles()
        z = [self.l1 * sin(a1) + self.l2 * sin(a2) for a1, a2 in zip(angle1, angle2)]
        x = [-self.l1 * cos(a1) + self.l2 * cos(a2) for a1, a2 in zip(angle1, angle2)]
        return x, z

    def get_phi(self) -> Tuple[List[float], List[float]]:
        r = [(x_i**2 + z_i**2) ** 0.5 for x_i, z_i in zip(self.x, self.z)]
        phi1 = [
            (
                pi
                - atan(z_i / x_i)
                - acos((self.l1**2 + r_i**2 - self.l2**2) / (2 * self.l1 * r_i))
            )
            - self.constants["leg1_angle_x"]
            for x_i, z_i, r_i in zip(self.x, self.z, r)
        ]
        phi2 = [
            self.constants["leg2_angle_x"]
            - acos((self.l1**2 + self.l2**2 - r_i**2) / (2 * self.l1 * self.l2))
            for r_i in r
        ]
        return phi1, phi2

    def get_horizont_angles(self) -> Tuple[List[float], List[float]]:
        angle1 = [self.constants["leg1_angle_x"] + phi for phi in self.phi1]
        angle2 = [
            self.constants["leg2_angle_x"] - phi1 - phi2
            for phi1, phi2 in zip(self.phi1, self.phi2)
        ]
        return angle1, angle2

    @staticmethod
    def clip_phi(
        phi: List[float], phi_border: Tuple[float, float]
    ) -> Tuple[List[float], bool]:
        clip_flag = False
        for i, phi_i in enumerate(phi):
            if phi_i < phi_border[0]:
                phi[i] = phi_border[0]
                clip_flag = True

            if phi_i > phi_border[1]:
                phi[i] = phi_border[1]
                clip_flag = True

        return phi, clip_flag
