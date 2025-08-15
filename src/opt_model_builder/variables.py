from __future__ import annotations
from typing import TYPE_CHECKING
from pyomo.kernel import (
    variable,
    variable_dict,
    block,
    NonNegativeIntegers,
    NonNegativeReals,
    Reals,
    Binary,
)
from itertools import product

if TYPE_CHECKING:
    from .opt_model_builder_class import OptModelBuilder


class Variables:
    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_variables(self, m: block) -> block:
        self._set_commodity_vars(m)
        self._set_sc_design_vars(m)
        if self.builder.use_isru:
            self._set_isru_vars(m)
        # IMLEO result storage, not really a design variable
        m.imleo = variable(domain=NonNegativeReals)
        self.builder.idx_name_dict["imleo"] = []

        return m

    def _set_commodity_vars(self, m: block):
        """Define commodity variables

        - int_com: number of integer commodities, such as number of crew,
            flying over from node i to j at time t under a certain scenario
            by spacecraft identified by its design and copy indices.
            The mass of such commodities is:
            # of commodity * mass of commodity per quantity
        - cnt_com: mass of continuous commodities, such as mass of propellant,
          for arc (i,j) at time t and ... (see int_com indices)
        - sc_fly_ind: binary variable indicating whether spacecraft (sc_des, sc_cp)
          flies from node i to j at time t.
        - sc_fly_var: product of spacecraft variable * sc_fly_ind.
          E.g., If spacecraft with certain dry mass is not flying, the corresponding
          sc_fly_var is 0. If it is flying, the corresponding sc_fly_var is
          exaclty the dry mass.
        """
        m.int_com = variable_dict()
        m.cnt_com = variable_dict()
        m.sc_fly_ind = variable_dict()
        m.sc_fly_var = variable_dict()
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
            for pl_i in m.int_com_idx:
                m.int_com[sc_des, sc_cp, i, j, pl_i, io, t] = variable(
                    domain=NonNegativeIntegers
                )
            for pl_c in m.cnt_com_idx:
                m.cnt_com[sc_des, sc_cp, i, j, pl_c, io, t] = variable(
                    domain=NonNegativeReals
                )
            m.sc_fly_ind[sc_des, sc_cp, i, j, io, t] = variable(domain=Binary)
            for sc_var in m.sc_var_idx:
                m.sc_fly_var[sc_des, sc_cp, sc_var, i, j, io, t] = variable(
                    domain=NonNegativeReals
                )
        self.builder.idx_name_dict["int_com"] = [
            "sc_des",
            "sc_cp",
            "dep_node",
            "arr_node",
            "int_com",
            "io",
            "time",
        ]
        self.builder.idx_name_dict["cnt_com"] = [
            "sc_des",
            "sc_cp",
            "dep_node",
            "arr_node",
            "cnt_com",
            "io",
            "time",
        ]
        self.builder.idx_name_dict["sc_fly_ind"] = [
            "sc_des",
            "sc_cp",
            "dep_node",
            "arr_node",
            "io",
            "time",
        ]
        self.builder.idx_name_dict["sc_fly_var"] = [
            "sc_des",
            "sc_cp",
            "sc_var",
            "dep_node",
            "arr_node",
            "io",
            "time",
        ]
        return m

    def _set_sc_design_vars(self, m: block):
        """Define variables related to spacecraft design

        In current model, spacecraft design is characterized by
        its payload capacity, propellant capacity, and dry mass.
        """
        m.pl_cap = variable_dict()
        m.prop_cap = variable_dict()
        m.dry_mass = variable_dict()
        for sc_des in m.sc_des_idx:
            m.pl_cap[sc_des] = variable(
                domain=Reals,
                lb=self.builder.sc.var_lb[self.builder.sc_var_dict["payload"]],
                ub=self.builder.sc.var_ub[self.builder.sc_var_dict["payload"]],
            )
            m.prop_cap[sc_des] = variable(
                domain=Reals,
                lb=self.builder.sc.var_lb[self.builder.sc_var_dict["propellant"]],
                ub=self.builder.sc.var_ub[self.builder.sc_var_dict["propellant"]],
            )
            m.dry_mass[sc_des] = variable(
                domain=Reals,
                lb=self.builder.sc.var_lb[self.builder.sc_var_dict["dry mass"]],
                ub=self.builder.sc.var_ub[self.builder.sc_var_dict["dry mass"]],
            )
        self.builder.idx_name_dict["pl_cap"] = ["sc_des"]
        self.builder.idx_name_dict["prop_cap"] = ["sc_des"]
        self.builder.idx_name_dict["dry_mass"] = ["sc_des"]
        return m

    def _set_isru_vars(self, m: block):
        """Define variables related to ISRU size and performance"""
        m.isru_mass = variable_dict()
        m.isru_O2rate = variable_dict()
        for t in product(m.time_idx):
            m.isru_mass[t] = variable(domain=Reals, lb=0, ub=10000)
            m.isru_O2rate[t] = variable(domain=NonNegativeReals)
        self.builder.idx_name_dict["isru_mass"] = ["time"]
        self.builder.idx_name_dict["isru_O2rate"] = ["time"]
        return m
