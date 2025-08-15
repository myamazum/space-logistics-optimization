from __future__ import annotations
from typing import TYPE_CHECKING

import numpy as np
import sys

if TYPE_CHECKING:
    from .optimizer_class import Optimizer

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin


class FixedSCDesign(InitMixin):
    def __init__(self, optimizer: Optimizer) -> None:
        self.optimizer = optimizer
        self.initialize_attributes(self.optimizer._input_data)

    def solve_network_flow_MILP(self, fixed_sc_vars: np.ndarray) -> float:
        """solves network flow opt. with fixed SC design as MILP

        Args:
            fixed_sc_vars: fixed SC design variables,
                provided by user or calculated automatically
        Returns:
            IMLEO: optimal IMLEO value with the given SC design
        """
        self.optimizer._model_builder.mode = "fixedSCdesign"
        self.optimizer._model_builder.fixed_sc_vars = fixed_sc_vars
        model = self.optimizer._model_builder.build_model()
        model = self.optimizer.solver.solve_model(model)
        IMLEO = model.imleo.value
        if IMLEO is None:
            IMLEO = float("inf")
        return IMLEO
