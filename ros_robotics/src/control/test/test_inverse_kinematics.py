import pytest
from control import InverseKinematics


class TestInverseKinematics:
    @pytest.fixture
    def ik_default(self):
        return InverseKinematics()

    @pytest.fixture
    def ik_custom(self):
        return InverseKinematics(l1=30, l2=25, phi1=45.0, phi2=90.0,
                                 phi1_border=(20.0, 100.0),
                                 phi2_border=(30.0, 120.0))

    def test_init_default(self, ik_default):
        assert ik_default.l1 == 40
        assert ik_default.l2 == 35
        assert ik_default.phi1 == 60.0
        assert ik_default.phi2 == 120.0
        assert ik_default.phi1_border == (30.0, 120.0)
        assert ik_default.phi2_border == (20.0, 150.0)
        assert ik_default.delta_phi1 == 0.0
        assert ik_default.delta_phi2 == 0.0

    def test_init_custom(self, ik_custom):
        assert ik_custom.l1 == 30
        assert ik_custom.l2 == 25
        assert ik_custom.phi1 == 45.0
        assert ik_custom.phi2 == 90.0
        assert ik_custom.phi1_border == (20.0, 100.0)
        assert ik_custom.phi2_border == (30.0, 120.0)

    def test_init_invalid_phi_borders(self):
        with pytest.raises(AssertionError):
            InverseKinematics(phi1_border=(0, 120.0))
        with pytest.raises(AssertionError):
            InverseKinematics(phi1_border=(30.0, 180.0))
        with pytest.raises(AssertionError):
            InverseKinematics(phi1_border=(120.0, 30.0))

    def test_clear(self, ik_default):
        ik_default.delta_phi1 = 10.0
        ik_default.delta_phi2 = 20.0
        ik_default.clear(phi1=50.0, phi2=100.0)

        assert ik_default.phi1 == 50.0
        assert ik_default.phi2 == 100.0
        assert ik_default.delta_phi1 == 0.0
        assert ik_default.delta_phi2 == 0.0

    @pytest.mark.parametrize("x, z, expect_error", [
        (0.0, 0.0, True),
        (100.0, 100.0, False),
        (1e6, 1e6, False),
        (30.0, 40.0, False),
        (10.0, 10.0, False),
    ])
    def test_update_with_various_coords(self, x, z, expect_error):
        ik = InverseKinematics(l1=40, l2=35)

        if expect_error:
            with pytest.raises(ValueError):
                ik.update(x, z)
        else:
            ik.update(x, z)
            assert ik.phi1_border[0] <= ik.phi1 <= ik.phi1_border[1]
            assert ik.phi2_border[0] <= ik.phi2 <= ik.phi2_border[1]

    def test_clip_phi(self):
        phi_border = (30.0, 120.0)

        assert InverseKinematics.clip_phi(20.0, phi_border) == 30.0

        assert InverseKinematics.clip_phi(130.0, phi_border) == 120.0

        assert InverseKinematics.clip_phi(60.0, phi_border) == 60.0

        assert InverseKinematics.clip_phi(30.0, phi_border) == 30.0
        assert InverseKinematics.clip_phi(120.0, phi_border) == 120.0
