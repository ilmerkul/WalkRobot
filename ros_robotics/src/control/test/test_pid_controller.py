import pytest
from control.pid_controller import PIDController


class TestPIDController:
    @pytest.fixture
    def pid(self):
        return PIDController()

    def test_initial_state(self, pid):
        assert pid.SetPoint == 0.0

        assert pid.p_term == 0.0
        assert pid.i_term == 0.0
        assert pid.d_term == 0.0
        assert pid.last_error == 0.0

        assert pid.int_error == 0.0
        assert pid.windup_guard == 20.0

        assert pid.output == 0.0

    def test_update_response(self, pid):
        pid.update(10.0)
        assert pid.output != 0.0

        initial_output = pid.output
        pid.update(5.0, current_time=pid.current_time + 1.0)
        assert abs(pid.output) < abs(initial_output)

    def test_setpoint_change(self, pid):
        pid.SetPoint = 5.0
        pid.update(0.0, current_time=pid.current_time + 1.0)
        assert pid.output > 0
