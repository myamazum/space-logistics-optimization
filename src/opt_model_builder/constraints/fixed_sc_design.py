from __future__ import annotations
from typing import TYPE_CHECKING
from itertools import product
from pyomo.kernel import (
    constraint,
    constraint_dict,
    block,
)

if TYPE_CHECKING:
    from ..opt_model_builder_class import OptModelBuilder


class FixSCDesign:
    """
    Class to fix spacecraft design to user-defined values via constraints.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def fix_sc_design(self, m) -> block:
        m.fixed_pl_cap_def = constraint_dict()
        m.fixed_prop_cap_def = constraint_dict()
        m.fixed_dry_mass_def = constraint_dict()
        for sc_des, sc_var in product(m.sc_des_idx, m.sc_var_idx):
            sc_var_name = self.builder.sc_var_dict.inverse[sc_var]
            if sc_var_name == "payload":
                m.fixed_pl_cap_def[sc_des, sc_var] = constraint(
                    m.pl_cap[sc_des] == self.builder.fixed_sc_vars[sc_des][sc_var]
                )
            elif sc_var_name == "propellant":
                m.fixed_prop_cap_def[sc_des, sc_var] = constraint(
                    m.prop_cap[sc_des] == self.builder.fixed_sc_vars[sc_des][sc_var]
                )
            elif sc_var_name == "dry mass":
                m.fixed_dry_mass_def[sc_des, sc_var] = constraint(
                    m.dry_mass[sc_des] == self.builder.fixed_sc_vars[sc_des][sc_var]
                )
            else:
                raise ValueError(
                    "Unknown SC design variable type: ", sc_var_name)

        return m
