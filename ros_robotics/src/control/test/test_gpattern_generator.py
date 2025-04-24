import pytest
from control import GPGenerator


class TestGPGenerator:
    @pytest.fixture
    def gp_default(self):
        return GPGenerator()

    @pytest.fixture
    def gp_custom_time(self):
        return GPGenerator(current_time=1000.0)

    def test_init_default(self, gp_default):
        assert gp_default.current_time is not None
        assert gp_default.last_time == gp_default.current_time
        assert gp_default.r == 0.0
        assert gp_default.teta == 0.0
        assert gp_default.phi == 0.0
        assert gp_default.mu == 1.0
        assert gp_default.omega == 1.0
        assert gp_default.xi == 1.0
        assert gp_default.a == 1.0

    def test_init_custom_time(self, gp_custom_time):
        assert gp_custom_time.current_time == 1000.0
        assert gp_custom_time.last_time == 1000.0

    def test_clear(self, gp_default):
        # Изменяем некоторые значения
        gp_default.r = 5.0
        gp_default.teta = 2.0
        gp_default.phi = 3.0
        gp_default.mu = 2.0
        gp_default.omega = 0.5
        gp_default.xi = 1.5
        gp_default.a = 0.8
        gp_default.r_prev = 1.0
        gp_default.dr_prev = 0.5
        gp_default.teta_prev = 1.0
        gp_default.phi_prev = 1.5

        gp_default.clear()

        # Проверяем сброс к значениям по умолчанию
        assert gp_default.r == 0.0
        assert gp_default.teta == 0.0
        assert gp_default.phi == 0.0
        assert gp_default.mu == 1.0
        assert gp_default.omega == 1.0
        assert gp_default.xi == 1.0
        assert gp_default.a == 1.0
        assert gp_default.r_prev == 0.0
        assert gp_default.dr_prev == 0.0
        assert gp_default.teta_prev == 0.0
        assert gp_default.phi_prev == 0.0

    def test_update_basic(self, gp_default):
        initial_time = gp_default.current_time
        gp_default.update(initial_time + 1.0)

        # Проверяем только свойства объекта после вызова update
        assert gp_default.r != 0.0  # Должно измениться от начального 0.0
        assert gp_default.teta == pytest.approx(1.0)  # omega=1.0 * t=1.0
        assert gp_default.phi == pytest.approx(1.0)   # xi=1.0 * t=1.0

    def test_update_with_prev_values(self, gp_default):
        # Устанавливаем предыдущие значения
        gp_default.r_prev = 2.0
        gp_default.dr_prev = 1.0
        gp_default.teta_prev = 0.5
        gp_default.phi_prev = 1.0

        initial_time = gp_default.current_time
        gp_default.update(initial_time + 2.0)

        # Проверяем только свойства объекта
        assert gp_default.r != 2.0  # Должно измениться от r_prev
        assert gp_default.teta == pytest.approx(2.0 + 0.5)  # omega=1.0 * t=2.0 + teta_prev
        assert gp_default.phi == pytest.approx(2.0 + 1.0)  # xi=1.0 * t=2.0 + phi_prev

    def test_modulate_basic(self, gp_default):
        initial_time = gp_default.current_time
        new_time = initial_time + 1.0
        gp_default.modulate(mu=2.0, omega=0.5, xi=1.5, current_time=new_time)

        # Проверяем обновленные параметры
        assert gp_default.mu == 2.0
        assert gp_default.omega == 0.5
        assert gp_default.xi == 1.5
        assert gp_default.last_time == new_time

        # Проверяем, что значения изменились
        assert gp_default.r != 0.0
        assert gp_default.dr_prev != 0.0
        assert gp_default.teta_prev != 0.0
        assert gp_default.phi_prev != 0.0

    def test_update_vs_modulate(self, gp_default):
        initial_time = gp_default.current_time

        # Проверяем, что update не изменяет параметры mu, omega, xi
        gp_default.update(initial_time + 1.0)
        assert gp_default.mu == 1.0
        assert gp_default.omega == 1.0
        assert gp_default.xi == 1.0

        # Проверяем, что modulate изменяет параметры
        gp_default.modulate(mu=2.0, omega=0.5, xi=1.5, current_time=initial_time + 2.0)
        assert gp_default.mu == 2.0
        assert gp_default.omega == 0.5
        assert gp_default.xi == 1.5

    def test_sequential_updates(self, gp_default):
        initial_time = gp_default.current_time

        # Первый вызов update
        gp_default.update(initial_time + 1.0)
        r1 = gp_default.r
        teta1 = gp_default.teta
        phi1 = gp_default.phi

        # Второй вызов update
        gp_default.update(initial_time + 2.0)

        # Проверяем, что значения изменились
        assert gp_default.r != r1
        assert gp_default.teta != teta1
        assert gp_default.phi != phi1
        assert gp_default.teta > teta1  # Должно увеличиться
        assert gp_default.phi > phi1    # Должно увеличиться
