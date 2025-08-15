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


class TimeWindow:
    """Class to set time window constraints.
    Commodities and spacecraft cannot fly over arcs outside of their time window.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_time_window_constraints(self, m: block) -> block:
        """
        Set commodities and spacecraft flow to 0
        if outside of the time window.

        There is no need to explicitly set the flow to >= 0
        since it is already enforced by the variable domain.
        (Removing redundant constraints led to performance improvements)
        """
        m.int_time_window_const = constraint_dict()
        m.cnt_time_window_const = constraint_dict()
        m.sc_time_window_const = constraint_dict()
        for sc_des, sc_cp, i, j, io, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.dep_node_idx,
            m.arr_node_idx,
            m.io_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            if t in self.builder._network_def.allowed_time_window[i][j]:
                continue
            for pi in m.int_com_idx:
                m.int_time_window_const[sc_des, sc_cp, i, j, pi, io, t] = (
                    constraint(m.int_com[sc_des, sc_cp, i, j, pi, io, t] == 0)
                )
            for pc in m.cnt_com_idx:
                m.cnt_time_window_const[sc_des, sc_cp, i, j, pc, io, t] = (
                    constraint(m.cnt_com[sc_des, sc_cp, i, j, pc, io, t] == 0)
                )
            m.sc_time_window_const[sc_des, sc_cp, i, j, io, t] = constraint(
                m.sc_fly_ind[sc_des, sc_cp, i, j, io, t] == 0
            )
        return m
