from input_data_class import InputData
from initializer import InitMixin
from component_designer.component_designer_class import ComponentDesigner
from optimizer.optimizer_class import Optimizer
from optimizer.optimizer_v2_class import OptimizerV2

class SpaceLogistics(InitMixin):
    def __init__(self, input_data: InputData) -> None:
        """Class to connect all classes via composition.

        Arguments can be a class itself or an instance of a class,
        depending on the class being initialized.
        A class itself is passed when initialization of the class needs to be
        delayed and done by the class to which it is passed as an argument.

        Args:
            input_data: InputData dataclass containing data input from user
        """

        self.initialize_attributes(input_data)
        self.comp_design = ComponentDesigner(input_data)
        #self.optimizer = Optimizer(input_data, self.comp_design)
        self.optimizer = OptimizerV2(input_data, self.comp_design)




