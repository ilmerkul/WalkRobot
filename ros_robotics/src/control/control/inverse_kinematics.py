from math import acos
from typing import Tuple


class InverseKinematics:
    def __init__(self, l1: float = 40, l2: float = 35,
                 phi1: float = 60.0, phi2: float = 120.0,
                 phi1_border: Tuple[float, float] = (30.0, 120.0),
                 phi2_border: Tuple[float, float] = (20.0, 150.0)):
        self._assert_phi_border(phi1_border)
        self._assert_phi_border(phi2_border)

        self.l1 = l1
        self.l2 = l2

        self.phi1_border = phi1_border
        self.phi2_border = phi2_border

        self.clear(phi1=phi1,
                   phi2=phi2)

    def _assert_phi(self, phi: float):
        assert 0 < phi < 180

    def _assert_phi_border(self, phi_border: Tuple[float, float]):
        self._assert_phi(phi_border[0])
        self._assert_phi(phi_border[1])
        assert phi_border[0] < phi_border[1]

    def clear(self, phi1: float = 60.0, phi2: float = 120.0):
        self.phi1 = phi1
        self.phi2 = phi2

        self.delta_phi1 = 0.0
        self.delta_phi2 = 0.0

    def update(self, x: float, z: float):
        r = (x ** 2 + z ** 2) ** 0.5

        phi1_new = acos(x/r) - acos((r ** 2 + self.l1 ** 2 - self.l2 ** 2)
                                    / (2 * r * self.l1))
        phi2_new = acos((self.l1 ** 2 + self.l2 ** 2 - r ** 2)
                        / (2 * self.l1 * self.l2))

        phi1_new = self.clip_phi(phi1_new, self.phi1_border)
        phi2_new = self.clip_phi(phi2_new, self.phi2_border)

        self.delta_phi1 = phi1_new - self.phi1
        self.delta_phi2 = phi2_new - self.phi2

        self.phi1 = phi1_new
        self.phi2 = phi2_new

    def clip_phi(phi: float, phi_border: Tuple[float, float]) -> float:
        if phi < phi_border[0]:
            phi = phi_border[0]

        if phi > phi_border[1]:
            phi = phi_border[1]

        return phi
