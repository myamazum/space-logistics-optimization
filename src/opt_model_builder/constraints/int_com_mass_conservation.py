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


class IntComConservation:
    """Class to set integer commodity conservation constraints."""

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_integer_com_conserv_constraints(self, m: block) -> block:
        self._set_crew_conservation_constraints(m)
        self._set_sc_conservation_constraints(m)
        return m

    def _set_crew_conservation_constraints(self, m: block) -> block:
        """crew outflow must be equal to crew inflow"""
        m.int_com_mass_cnsv = constraint_dict()
        for i, j, t in product(
            m.dep_node_idx,
            m.arr_node_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            m.int_com_mass_cnsv[i, j, self.builder.int_com_dict["crew #"], t] = (
                constraint(
                    sum(
                        m.int_com[
                            sc_des,
                            sc_cp,
                            i,
                            j,
                            self.builder.int_com_dict["crew #"],
                            self.builder.flow_dict["in"],
                            t,
                        ]
                        for sc_des in m.sc_des_idx
                        for sc_cp in m.sc_copy_idx
                    )
                    == sum(
                        m.int_com[
                            sc_des,
                            sc_cp,
                            i,
                            j,
                            self.builder.int_com_dict["crew #"],
                            self.builder.flow_dict["out"],
                            t,
                        ]
                        for sc_des in m.sc_des_idx
                        for sc_cp in m.sc_copy_idx
                    )
                )
            )
        return m

    def _set_sc_conservation_constraints(self, m: block) -> block:
        """spacecraft outflow must be equal to spacecraft inflow"""
        m.sc_cnsv = constraint_dict()
        for sc_des, sc_cp, i, j, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.dep_node_idx,
            m.arr_node_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            m.sc_cnsv[sc_des, sc_cp, i, j, t] = constraint(
                m.sc_fly_ind[sc_des, sc_cp, i, j, self.builder.flow_dict["in"], t]
                == m.sc_fly_ind[
                    sc_des, sc_cp, i, j, self.builder.flow_dict["out"], t
                ]
            )
        return m
