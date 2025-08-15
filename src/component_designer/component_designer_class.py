import sys
from .isru.isru_O2_rate_model import ISRUDesign
from .spacecraft.spacecraft_sizing import SCSizing

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin


class ComponentDesigner(InitMixin):
    """Class to design campaign components like spacecraft and ISRU plants."""

    def __init__(self, input_data) -> None:
        """
        Args:
            input_data: InputData dataclass containing data input from user
        """
        self.initialize_attributes(input_data)
        self._input_data = input_data
        self.sc_sizing = SCSizing(self)
        self.isru_des = ISRUDesign(self)
