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


class CntComConservation:
    """
    Class to set continuous commodity conservation and consumption constraints.

    Propellant conservation and consumptions are enforced via different sets of constraints.
    It is assumed that spacecraft can exchange their commodities as long
     - they are present at the same node at the same time, and
     - the sum of commodities do not exceed their payload capacity
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_non_prop_continuous_com_conserv_constraints(self, m: block) -> block:
        if self.builder.use_isru:
            m = self._set_isru_plant_mass_defition(m)
        m.cnt_com_cnsv = constraint_dict()
        for i, j, pc, t in product(
            m.dep_node_idx,
            m.arr_node_idx,
            m.cnt_com_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            elif self.builder.cnt_com_dict.inverse[pc] in self.builder.prop_com_names:
                continue
            if pc == self.builder.cnt_com_dict["plant"]:
                if self.builder.can_operate_ISRU(i, j):
                    self._set_isru_plant_decay_constraint(m, i, j, pc, t)
                else:
                    self._equalize_outflow_inflow(m, i, j, pc, t)
            elif pc == self.builder.cnt_com_dict["maintenance"]:
                if self.builder.is_transportation_arc(i, j):
                    self._set_sc_maintenance_constraint(m, i, j, pc, t)
                elif self.builder.can_operate_ISRU(i, j):
                    self._set_isru_maintenance_constraint(m, i, j, pc, t)
                else:
                    self._equalize_outflow_inflow(m, i, j, pc, t)
            elif pc == self.builder.cnt_com_dict["consumption"]:
                self._set_consumable_constaraints(m, i, j, pc, t)
            else:
                self._equalize_outflow_inflow(m, i, j, pc, t)
        return m

    def _set_consumable_constaraints(self, m, i, j, pc, t):
        """
        Crew consumes 'consumable' commodity (e.g. food)
        with mass proportional to number of crew and time of flight
        """
        m.cnt_com_cnsv[i, j, pc, t] = constraint(
            sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["in"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            == sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["out"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            - self.builder.mis.consumption_cost
            * self.builder._network_def.real_arc_time[i][j]
            * sum(
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
        return m

    def _set_sc_maintenance_constraint(self, m, i, j, pc, t):
        """
        Flying spacecraft requires maintenance supplies (commodities)
        with mass proportional to their dry mass
        """
        m.cnt_com_cnsv[i, j, pc, t] = constraint(
            sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["in"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            == sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["out"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            - self.builder.mis.maintenance_cost
            * sum(
                m.sc_fly_var[
                    sc_des,
                    sc_cp,
                    self.builder.sc_var_dict["dry mass"],
                    i,
                    j,
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
        )
        return m

    def _set_isru_maintenance_constraint(self, m, i, j, pc, t):
        """
        Operating ISRU requires maintenance supplies (commodities)
        with mass proportional to plant mass and work time
        """
        t_id = self.builder._network_def.date_to_time_idx_dict[t]
        m.cnt_com_cnsv[i, j, pc, t] = constraint(
            sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["in"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            == sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["out"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            - (
                self.builder.isru.maintenance_cost
                * self.builder.isru_work_time[i][t_id]
                / 365
            )
            * sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    i,
                    j,
                    self.builder.cnt_com_dict["plant"],
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
        )
        return m

    def _equalize_outflow_inflow(self, m, i, j, pc, t):
        """when nothing is consumed, outflow and inflow must be the same"""
        m.cnt_com_cnsv[i, j, pc, t] = constraint(
            sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["in"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            == sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["out"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
        )
        return m

    def _set_isru_plant_decay_constraint(self, m, i, j, pc, t) -> block:
        """
        ISRU plants decay proportionally to their work time
        This decay is expressed as decrease in plant mass
        """
        t_id = self.builder._network_def.date_to_time_idx_dict[t]
        m.cnt_com_cnsv[i, j, pc, t] = constraint(
            sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["in"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            == sum(
                m.cnt_com[sc_des, sc_cp, i, j, pc, self.builder.flow_dict["out"], t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            * (
                1
                - (
                    self.builder.isru.decay_rate
                    * self.builder.isru_work_time[i][t_id]
                    / 365
                )
            )
        )
        return m

    def _set_isru_plant_mass_defition(self, m) -> block:
        """Define total ISRU plant mass as constraints"""
        m.isru_plant_cnsv = constraint_dict()
        for t, scnr in product(m.time_idx, m.scnr_idx):
            m.isru_plant_cnsv[t, scnr] = constraint(
                m.isru_mass[t, scnr]
                == sum(
                    m.cnt_com[
                        sc_des,
                        sc_cp,
                        self.builder.node_dict["LS"],
                        self.builder.node_dict["LS"],
                        self.builder.cnt_com_dict["plant"],
                        self.builder.flow_dict["out"],
                        t,
                        scnr,
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                )
            )
        return m
