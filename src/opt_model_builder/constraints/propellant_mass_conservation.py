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


class PropellantConservation:
    """
    Class to set propellant conservation and consumption constraints.

    It is assumed that spacecraft can exchange their propellant as long
     - they are present at the same node at the same time, and
     - the sum of propellant do not exceed their propellant capacity
    Propellant consumptions is applied to the sum of propellant of all
    spacecraft flying over the same arc; that is,
        total prop out - total prop required = total prop in
    This allows 'stacking' of spacecraft. For example, SC1 can fly over an arc
    with no propellant consumption if SC2 flies over the same arc
    at the same time and provide required propellant for SC1.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_propellant_conservation_constraints(self, m: block) -> block:
        m.prop_mass_cnsv = constraint_dict()

        for i, j, t, prop_name in product(
            m.dep_node_idx,
            m.arr_node_idx,
            m.time_idx,
            self.builder.prop_com_names,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            if prop_name == "hydrogen":
                self.isru_ratio = self.builder.isru.H2_H2O_ratio
                self.prop_ratio = self.builder.sc.fuel_prop_ratio
            elif prop_name == "oxygen":
                self.isru_ratio = self.builder.isru.O2_H2O_ratio
                self.prop_ratio = self.builder.sc.oxi_prop_ratio
            else:
                NotImplementedError(
                    "Currently only H2 and O2 are supported as propellant"
                )
            if self.builder.can_operate_ISRU(i, j):
                self._set_isru_prop_generation_constraints(m, i, j, t, prop_name)
            else:
                self._set_flight_prop_consumption_constraint(m, i, j, t, prop_name)
        return m

    def _set_flight_prop_consumption_constraint(self, m, i, j, t, prop_name):
        """
        SC consumes propellant for flights, following rocket equation

        final prop mass <= itinial prop mass - consumed prop mass, where
        consumed prop mass = final-initial mass ratio * initial mass
        Fuel and oxydizer are defined as different commodities, so
        the consumed fuel (oxydizer) mass is calculated as:
        consumed prop mass * fuel(oxydizer)-propellant ratio
        """
        prop_id = self.builder.cnt_com_dict[prop_name]
        t_id = self.builder._network_def.date_to_time_idx_dict[t]
        m.prop_mass_cnsv[i, j, prop_id, t] = constraint(
            sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    i,
                    j,
                    prop_id,
                    self.builder.flow_dict["in"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            <= sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    i,
                    j,
                    prop_id,
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            - self.prop_ratio
            * self.builder.fin_ini_mass_frac[i][j][t_id]
            * sum(
                sum(
                    self.builder.int_com_costs[self.builder.int_com_dict[pl_name]]
                    * m.int_com[
                        sc_des,
                        sc_cp,
                        i,
                        j,
                        self.builder.int_com_dict[pl_name],
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for pl_name in self.builder.int_com_names
                )
                + sum(
                    m.cnt_com[
                        sc_des,
                        sc_cp,
                        i,
                        j,
                        self.builder.cnt_com_dict[pl_name],
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for pl_name in self.builder.cnt_com_names
                )
                + m.sc_fly_var[
                    sc_des,
                    sc_cp,
                    self.builder.sc_var_dict["dry mass"],
                    i,
                    j,
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_cp in m.sc_copy_idx
                for sc_des in m.sc_des_idx
            )
        )

    def _set_isru_prop_generation_constraints(self, m, i, j, t, prop_name):
        """
        ISRU plants can generate H2 and O2, where production rate is
        propotional to the mass and work time of the plants.
        H2 and O2 are two distinct commodities.
        """
        prop_id = self.builder.cnt_com_dict[prop_name]
        t_id = self.builder._network_def.date_to_time_idx_dict[t]
        m.prop_mass_cnsv[i, j, prop_id, t] = constraint(
            sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    i,
                    j,
                    prop_id,
                    self.builder.flow_dict["in"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            <= sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    i,
                    j,
                    prop_id,
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
            )
            + (self.builder.isru_work_time[i][t_id] / 365)
            * self.isru_ratio
            * m.isru_mass[t]
            * self.builder.isru.production_rate
        )
