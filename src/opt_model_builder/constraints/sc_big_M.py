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


class SCBigM:
    """Class to add big-M constraints for the spacecraft variables.
    Comoposed to the OptModelBuilder class.

    Spacecraft flight variables (sc_fly_var) is defined as the product of
    the spacecraft flight indicator (sc_fly_ind) and the spacecraft variable,
    which represents the amount of the variable that is actually flying.
    For example, if the dry mass of spacecraft is 1000 kg, the corresponding
    spacecraft flight variable is 1000 if the spacecraft is flying, and 0 o.w.

    This binary-continuous bilinear term can be expresses as a disjunction of
    two polytopes: {0} and [0, M], where M is a big-M constant (variable upper
    bound). Refer to any integer programming textbook for info on disjunction.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_sc_big_M_constraints(self, m: block) -> block:
        """Set big-M constraints for the spacecraft flight variables."""
        big_M_val: float = max(self.builder.sc.var_ub)
        m.sc_bigM_const_1 = constraint_dict()
        m.sc_bigM_const_2 = constraint_dict()
        m.sc_bigM_const_3 = constraint_dict()
        for sc_des, sc_cp, sc_var, i, j, io, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.sc_var_idx,
            m.dep_node_idx,
            m.arr_node_idx,
            m.io_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue

            m.sc_bigM_const_1[sc_des, sc_cp, sc_var, i, j, io, t] = constraint(
                m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                <= m.sc_fly_ind[sc_des, sc_cp, i, j, io, t] * big_M_val
            )

            if self.builder.sc_var_dict.inverse[sc_var] == "payload":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        <= m.pl_cap[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        >= m.pl_cap[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, i, j, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )
            if self.builder.sc_var_dict.inverse[sc_var] == "propellant":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        <= m.prop_cap[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        >= m.prop_cap[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, i, j, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )
            if self.builder.sc_var_dict.inverse[sc_var] == "dry mass":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        <= m.dry_mass[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, i, j, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t]
                        >= m.dry_mass[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, i, j, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )

        return m
