from __future__ import annotations
from typing import TYPE_CHECKING
from itertools import product
from pyomo.kernel import constraint, constraint_dict, block

if TYPE_CHECKING:
    from ..opt_model_builder_class import OptModelBuilder


class SCCapacity:
    """Class to set Spacecraft capacity constraints.

    Spacecraft cannot carry more than their capacity of payload and propellant.
    The propellant capacity is divided into oxygen (oxydizer) and fuel.
    The oxydizer is assumed to be oxygen.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_sc_cap_constraints(self, m: block) -> block:
        m.pl_cap_const = constraint_dict()
        m.oxy_cap_const = constraint_dict()
        m.fuel_cap_const = constraint_dict()
        for sc_des, sc_cp, i, j, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.dep_node_idx,
            m.arr_node_idx,
            m.time_idx,
        ):
            if not self.builder.is_feasible_arc(i, j):
                continue
            if (
                self.builder.node_dict.inverse[i] == "Earth"
                and self.builder.node_dict.inverse[j] == "LEO"
            ):
                continue
            m = self._set_payload_cap_constraints(m, sc_des, sc_cp, i, j, t)
            m = self._set_oxy_cap_constraints(m, sc_des, sc_cp, i, j, t)
            m = self._set_fuel_cap_constraints(m, sc_des, sc_cp, i, j, t)
        return m

    def _set_payload_cap_constraints(self, m: block, sc_des, sc_cp, i, j, t) -> block:
        """Sum of non-propellant commodities cannot exceed SC payload capacity"""
        m.pl_cap_const[sc_des, sc_cp, i, j, t] = constraint(
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
                if pl_name not in self.builder.prop_com_names
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
                if pl_name not in self.builder.prop_com_names
            )
            <= m.sc_fly_var[
                sc_des,
                sc_cp,
                self.builder.sc_var_dict["payload"],
                i,
                j,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m

    def _set_oxy_cap_constraints(self, m: block, sc_des, sc_cp, i, j, t) -> block:
        """Oxygen mass cannot exceed SC oxygen capacity"""
        m.oxy_cap_const[sc_des, sc_cp, i, j, t] = constraint(
            m.cnt_com[
                sc_des,
                sc_cp,
                i,
                j,
                self.builder.cnt_com_dict["oxygen"],
                self.builder.flow_dict["out"],
                t,
            ]
            <= self.builder.sc.oxi_prop_ratio
            * m.sc_fly_var[
                sc_des,
                sc_cp,
                self.builder.sc_var_dict["propellant"],
                i,
                j,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m

    def _set_fuel_cap_constraints(self, m: block, sc_des, sc_cp, i, j, t) -> block:
        """
        Hydrogen mass cannot exceed SC fuel capacity.
        We assume that the fuel is hydrogen and the oxidizer is oxygen.
        """
        # TODO: non-hydrogen fuel
        m.fuel_cap_const[sc_des, sc_cp, i, j, t] = constraint(
            m.cnt_com[
                sc_des,
                sc_cp,
                i,
                j,
                self.builder.cnt_com_dict["hydrogen"],
                self.builder.flow_dict["out"],
                t,
            ]
            <= self.builder.sc.fuel_prop_ratio
            * m.sc_fly_var[
                sc_des,
                sc_cp,
                self.builder.sc_var_dict["propellant"],
                i,
                j,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m
