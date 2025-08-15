from .indicies import Indices
from .variables import Variables
from .objective import Objective
from .constraints_cls import Constraints

from pyomo.kernel import block, variable
import numpy as np
import sys

try:
    from initializer import InitMixin
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from network_builder.network_builder_class import NetworkBuilder
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from network_builder.network_builder_class import NetworkBuilder


class OptModelBuilder(InitMixin):
    """Class to build Pyomo optimization model."""

    def __init__(
        self,
        input_data: InputData,
        comp_design: ComponentDesigner,
    ) -> None:
        """
        Args:
            input_data: InputData dataclass containing data input from user
            comp_design: ComponentDesigner instance
        """
        self.initialize_attributes(input_data)
        self.input_data = input_data
        self._comp_design = comp_design

        # assign network related attributes
        self._network_def = NetworkBuilder(input_data)
        self.time_steps: list[int] = self._network_def.time_steps
        self.first_mis_time_steps: list[int] = self._network_def.first_mis_time_steps
        self.second_mis_time_steps: list[int] = self._network_def.second_mis_time_steps
        self.is_feasible_arc = self._network_def.is_feasible_arc
        self.can_operate_ISRU = self._network_def.can_operate_ISRU
        self.delta_t: np.ndarray = self._network_def.delta_t
        self.isru_work_time: np.ndarray = self._network_def.isru_work_time
        self.fin_ini_mass_frac: np.ndarray = self._network_def.fin_ini_mass_frac
        self.is_holdover_arc = self._network_def.is_holdover_arc
        self.is_transportation_arc = self._network_def.is_transportation_arc

        # placeholder
        self.idx_name_dict: dict[str, list[str]] = {}

    @property
    def mode(self) -> str:
        """Mode of the optimization model. Based on this mode,
        the class define different variables/constraints/objective.
        """
        return self._mode

    @mode.setter
    def mode(self, mode: str) -> None:
        if mode not in [
            "Piecewise Linear Approx",
            "fixedSCdesign",
        ]:
            raise ValueError("Mode is invalid")
        self._mode = mode

    @property
    def fixed_sc_vars(self) -> np.ndarray:
        """User-defined or auto-generated spacecraft design variables"""
        return self._fixed_sc_vars

    @fixed_sc_vars.setter
    def fixed_sc_vars(self, fixed_sc_vars: np.ndarray) -> None:
        if not isinstance(fixed_sc_vars, np.ndarray):
            raise ValueError("Fixed SC variables is not a numpy array")
        if fixed_sc_vars.shape != (self.n_sc_design, self.n_sc_vars):
            raise ValueError(
                """Fixed SC variables has invalid nupmy array shape.
                Received: {}
                Expected: ({},{})""".format(
                    fixed_sc_vars.shape,
                    self.n_sc_design,
                    self.n_sc_vars,
                )
            )
        self._fixed_sc_vars = fixed_sc_vars

    def build_model(self, pwl_increment: float = 2500) -> block:
        """build the optimization model based on input data"""
        m: block = block()
        m = Indices(self).set_indices(m)
        m = Variables(self).set_variables(m)
        self._test_index_variable_mapping(m)
        m = Constraints(self).set_constraints(m, pwl_increment)
        m = Objective(self).set_objective(m)
        return m

    def _test_index_variable_mapping(self, model: block) -> None:
        """Check all variables in the model have a corresponding index mapping

        This function checks two things:
        1. All variables in the model have a corresponding index name list
        2. Each index name for each variable is recognized in the master index name list

        Args:
            model: constructed pyomo.kernel block model
        """
        for var in model.component_objects(
            ctype=variable, active=True, descend_into=True
        ):
            assert var.name in self.idx_name_dict.keys(), """
            Variable name {} not found in the variable-index dictionary.
            Each variable in the optimization model needs a list of its indicies.
            """.format(var.name)
            for key in self.idx_name_dict[var.name]:
                assert key in self.idx_name_dict["all"], """
                Index name {} for variable {} is not recognized.
                Make sure to select indicies from the following list: {}
                """.format(key, var.name, self.idx_name_dict["all"])
