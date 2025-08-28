from __future__ import annotations
from typing import TYPE_CHECKING
from pyomo.kernel import block
from .constraints.mass_balance import MassBalance
from .constraints.sc_capacity import SCCapacity
from .constraints.time_window import TimeWindow
from .constraints.sc_big_M import SCBigM
from .constraints.int_com_mass_conservation import IntComConservation
from .constraints.cnt_com_mass_conservation import CntComConservation
from .constraints.propellant_mass_conservation import PropellantConservation
from .constraints.piecewise_linear import PiecewiseLinearConstraints
from .constraints.fixed_sc_design import FixSCDesign

if TYPE_CHECKING:
    from .opt_model_builder_class import OptModelBuilder

class Constraints:
    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_constraints(self, m: block, pwl_increment) -> block:
        MassBalance(self.builder).set_mass_balance_constraints(m)
        SCCapacity(self.builder).set_sc_cap_constraints(m)
        SCBigM(self.builder).set_sc_big_M_constraints(m)
        TimeWindow(self.builder).set_time_window_constraints(m)
        IntComConservation(self.builder).set_integer_com_conserv_constraints(m)
        CntComConservation(
            self.builder
        ).set_non_prop_continuous_com_conserv_constraints(m)
        PropellantConservation(
            self.builder).set_propellant_conservation_constraints(m)

        if self.builder.mode == "Piecewise Linear Approx":
            PiecewiseLinearConstraints(self.builder).set_piecewise_linear_constraints(
                m, pwl_increment
            )
        if self.builder.mode == "fixedSCdesign":
            FixSCDesign(self.builder).fix_sc_design(m)

        return m
