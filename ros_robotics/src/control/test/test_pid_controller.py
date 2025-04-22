import pytest
from control.pid_controller import PIDController
import time


class TestPIDController:
    @pytest.fixture
    def pid(self):
        return PIDController()

    def test_initial_state(self, pid):
        assert pid.SetPoint == 0.0

        assert pid.PTerm == 0.0
        assert pid.ITerm == 0.0
        assert pid.DTerm == 0.0
        assert pid.last_error == 0.0

        assert pid.int_error == 0.0
        assert pid.windup_guard == 20.0

        assert pid.output == 0.0

    def test_update_response(self, pid):
        pid.update(10.0)
        assert pid.output != 0.0

        initial_output = pid.output
        time.sleep(1)
        pid.update(5.0)
        assert abs(pid.output) < abs(initial_output)

    def test_setpoint_change(self, pid):
        pid.SetPoint = 5.0
        time.sleep(1)
        pid.update(0.0)
        assert pid.output > 0
