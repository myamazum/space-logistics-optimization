import sys
from math import exp

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin


class ISRUDesign(InitMixin):
    def __init__(self, comp_designer) -> None:
        self.comp = comp_designer
        self.initialize_attributes(self.comp._input_data)

    def get_isru_O2_rate(self, isru_mass: float) -> float:
        if isru_mass >= 400:
            return isru_mass * (
                -0.43798
                + 6.96226 * (1 - exp(-isru_mass / 812.15628))
                + 2.01727 * (1 - exp(-isru_mass / 3967.2644))
            )
        else:
            return 0
