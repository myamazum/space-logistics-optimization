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


class MassBalance:
    """Class to set mass balance constraints for commodities and SC.

    At each node, the sum of inflow in the previous time step must be greater
    than the sum of outflow in the current time step. If there is demand for
    the commodity of interest, the inflow must be greater than the demand plus
    the outflow. Demand is negative and supply is positive.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_mass_balance_constraints(self, m: block) -> block:
        m = self._set_int_com_mass_balance_constraints(m)
        m = self._set_cnt_com_mass_balance_constraints(m)
        m = self._set_sc_balance_constraints(m)
        return m

    def _set_int_com_mass_balance_constraints(self, m: block) -> block:
        """Enforce mass balance for each integer commodity"""
        m.int_com_mass_balance_const = constraint_dict()
        for i, j, int_com_id, t in product(
            m.dep_node_idx, m.arr_node_idx, m.int_com_idx, m.time_idx
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            t_id = self.builder._network_def.date_to_time_idx_dict[t]
            m.int_com_mass_balance_const[i, j, int_com_id, t] = constraint(
                sum(
                    m.int_com[
                        sc_des,
                        sc_cp,
                        i,
                        j,
                        int_com_id,
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                )
                - sum(
                    m.int_com[
                        sc_des,
                        sc_cp,
                        j,
                        i,
                        int_com_id,
                        self.builder.flow_dict["in"],
                        t - self.builder.delta_t[i][j][t_id],
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                    if t - self.builder.delta_t[i][j][t_id] in m.time_idx
                )
                <= self.builder._network_def.int_com_demand[i][int_com_id][t_id]
            )
        return m

    def _set_cnt_com_mass_balance_constraints(self, m: block) -> block:
        """Enforce mass balance for each continuous commodity"""
        m.cnt_com_mass_balance_const = constraint_dict()
        for i, j, cnt_com_id, t in product(
            m.dep_node_idx, m.arr_node_idx, m.cnt_com_idx, m.time_idx
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            t_id = self.builder._network_def.date_to_time_idx_dict[t]
            m.cnt_com_mass_balance_const[i, j, cnt_com_id, t] = constraint(
                sum(
                    m.cnt_com[
                        sc_des,
                        sc_cp,
                        i,
                        j,
                        cnt_com_id,
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                )
                - sum(
                    m.cnt_com[
                        sc_des,
                        sc_cp,
                        j,
                        i,
                        cnt_com_id,
                        self.builder.flow_dict["in"],
                        t - self.builder.delta_t[i][j][t_id],
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                    if t - self.builder.delta_t[i][j][t_id] in m.time_idx
                )
                <= self.builder._network_def.cnt_com_demand[i][cnt_com_id][t_id]
            )
        return m

    def _set_sc_balance_constraints(self, m: block) -> block:
        """
        Enforce mass balance for each spacecraft and each copy.
        Unless at the Earth node, the inflow must be greater than the outflow.
        Since new SC can be launched at earth node, the outflow can be greater
        than the inflow by 1 (i.e., new SC with sc_des and sc_cp launched).
        """
        m.sc_balance_const = constraint_dict()
        for sc_des, sc_cp, i, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.dep_node_idx,
            m.time_idx,
        ):
            t_id = self.builder._network_def.date_to_time_idx_dict[t]
            m.sc_balance_const[sc_des, sc_cp, i, t] = constraint(
                sum(
                    m.sc_fly_ind[
                        sc_des, sc_cp, i, j, self.builder.flow_dict["out"], t
                    ]
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                )
                - sum(
                    m.sc_fly_ind[
                        sc_des,
                        sc_cp,
                        j,
                        i,
                        self.builder.flow_dict["in"],
                        t - self.builder.delta_t[i][j][t_id],
                    ]
                    for j in m.arr_node_idx
                    if self.builder.is_feasible_arc(i, j)
                    if t - self.builder.delta_t[i][j][t_id] in m.time_idx
                )
                <= (1 if i == self.builder.node_dict["Earth"] else 0)
            )
        return m
