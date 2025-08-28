import sys
from .pwl_approx_problems_v2 import PWLApproximation
from .fixed_sc_problem_v2 import FixedSCDesign
from .solver_interface_v2 import SolverInterface
from .output_manager_v2 import OutputManager

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin

try:
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from input_data_class_wrapper import NetworkBuilderV2 
    from opt_model_builder.opt_model_builder_v2_class import OptModelBuilderV2
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from input_data_class_wrapper import NetworkBuilderV2 
    from opt_model_builder.opt_model_builder_v2_class import OptModelBuilderV2

class OptimizerV2(InitMixin):
    """Class to optimize Pyomo models."""

    def __init__(
        self,
        input_data: InputData,
        comp_design: ComponentDesigner,
    ) -> None:
        """
        Args:
            input_data: InputData dataclass containing data input from user
            comp_design: ComponentDesigner instance
            network_builder_cls: NetworkBuilder class itself (not an instance)
            model_builder_cls: OptModelBuilder class itself (not an instance)
        """
        self.initialize_attributes(input_data)
        self._input_data = input_data
        self._comp_design = comp_design
        self._network_def = NetworkBuilderV2(input_data)
        self._model_builder = OptModelBuilderV2(input_data, comp_design)

        # composed (sub)classes
        self.pwl = PWLApproximation(self)
        self.fixed_sc = FixedSCDesign(self)
        self.solver = SolverInterface(self)
        self.output = OutputManager(self)
