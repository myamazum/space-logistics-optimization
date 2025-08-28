from __future__ import annotations
from itertools import product
from typing import TYPE_CHECKING

from pyomo.kernel import (
    constraint,
    constraint_dict,
    block,
)

from .constraints.piecewise_linear import PiecewiseLinearConstraintsV2
from .constraints.fixed_sc_design import FixSCDesignV2

if TYPE_CHECKING:
    from .opt_model_builder_v2_class import OptModelBuilderV2

class Constraints:
    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder
    
    def set_constraints(self,m,pwl_increment):
        nb = self.builder._network_def

        m.A = range(len(nb.arc_list))           # arc index
        m.T = range(len(nb.time_steps))
        OUT = self.builder.flow_dict["out"]; INN = self.builder.flow_dict["in"]

        # arc → 属性
        arc_allowed = {a.id: set(nb.allowed_times_by_arc.get(a.id, [])) for a in nb.arc_list}
        arc_alpha   = {a.id: a.alpha for a in nb.arc_list}

        # 変数は許可された (arc,time) のみ生成しているため、窓外の 0 固定は不要

        # ---- SCBigM（飛ぶときだけ dry/payload/propellant を有効化）----
        SD = self.builder.sc_var_dict
        DM_UB = float(self.builder.sc.var_ub[SD["dry mass"]])
        PL_UB = float(self.builder.sc.var_ub[SD["payload"]])
        PR_UB = float(self.builder.sc.var_ub[SD["propellant"]])
        # constraint containers for Big-M linking
        m.c_dm_up = constraint_dict(); m.c_dm_lo0 = constraint_dict(); m.c_dm_cap = constraint_dict(); m.c_dm_eq = constraint_dict()
        m.c_pl_up = constraint_dict(); m.c_pl_lo0 = constraint_dict(); m.c_pl_cap = constraint_dict(); m.c_pl_eq = constraint_dict()
        m.c_pr_up = constraint_dict(); m.c_pr_lo0 = constraint_dict(); m.c_pr_cap = constraint_dict(); m.c_pr_eq = constraint_dict()
        for a in m.A:
            for t in m.T:
                if t not in arc_allowed[a]:
                    continue
                for d in m.sc_des_idx:
                    for p in m.sc_copy_idx:
                        y = m.sc_fly_ind[d,p,a,t]
                        # dry
                        x = m.sc_fly_var[d,p,SD["dry mass"],a,t]
                        m.c_dm_up   [d,p,a,t] = constraint(x <= DM_UB * y)
                        m.c_dm_lo0  [d,p,a,t] = constraint(x >= 0)
                        m.c_dm_cap  [d,p,a,t] = constraint(x <= m.dry_mass[d])
                        m.c_dm_eq   [d,p,a,t] = constraint(x >= m.dry_mass[d] - DM_UB*(1-y))
                        # payload
                        x = m.sc_fly_var[d,p,SD["payload"],a,t]
                        m.c_pl_up   [d,p,a,t] = constraint(x <= PL_UB * y)
                        m.c_pl_lo0  [d,p,a,t] = constraint(x >= 0)
                        m.c_pl_cap  [d,p,a,t] = constraint(x <= m.pl_cap[d])
                        m.c_pl_eq   [d,p,a,t] = constraint(x >= m.pl_cap[d] - PL_UB*(1-y))
                        # propellant
                        x = m.sc_fly_var[d,p,SD["propellant"],a,t]
                        m.c_pr_up   [d,p,a,t] = constraint(x <= PR_UB * y)
                        m.c_pr_lo0  [d,p,a,t] = constraint(x >= 0)
                        m.c_pr_cap  [d,p,a,t] = constraint(x <= m.prop_cap[d])
                        m.c_pr_eq   [d,p,a,t] = constraint(x >= m.prop_cap[d] - PR_UB*(1-y))

        # ---- SCCapacity（非推薬と推進剤の容量上限）----
        prop_names = set(self.builder.prop_com_names)           # {"oxygen","hydrogen"}
        is_prop = lambda k: (self.builder.cnt_com_names[k] in prop_names)

        m.c_sc_payload = constraint_dict(); m.c_sc_prop = constraint_dict()
        for a in m.A:
            for t in m.T:
                if t not in arc_allowed[a]:
                    continue
                # 非推薬 out の合計 ≤ 飛んだ機体の payload cap 合計
                payload_out = (
                    sum(
                        m.cnt_com[d,p,a,k,OUT,t]
                        for d in m.sc_des_idx for p in m.sc_copy_idx for k in m.cnt_com_idx
                        if not is_prop(k)
                    )
                    + sum(
                        float(self.builder.int_com_costs[i]) * m.int_com[d,p,a,i,OUT,t]
                        for d in m.sc_des_idx for p in m.sc_copy_idx for i in m.int_com_idx
                    )
                )
                payload_cap = sum(m.sc_fly_var[d,p,SD["payload"],a,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                m.c_sc_payload[a,t] = constraint(payload_out <= payload_cap)

                # 推進剤（O2+H2） out の合計 ≤ 飛んだ機体の prop cap 合計
                o2, h2 = self.builder.cnt_com_dict["oxygen"], self.builder.cnt_com_dict["hydrogen"]
                prop_out = sum(m.cnt_com[d,p,a,c,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx for c in (o2,h2))
                prop_cap = sum(m.sc_fly_var[d,p,SD["propellant"],a,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                m.c_sc_prop[a,t] = constraint(prop_out <= prop_cap)

        # ---- PropellantConservation（ロケット方程式の燃焼要求）----
        # O2/H2 比率は SC パラメータから取得（他箇所と整合）
        phi_O2 = float(self.builder.sc.oxi_prop_ratio)
        phi_H2 = float(self.builder.sc.fuel_prop_ratio)
        int_cost = self.builder.int_com_costs
        o2, h2 = self.builder.cnt_com_dict["oxygen"], self.builder.cnt_com_dict["hydrogen"]
        m.c_burn_O2 = constraint_dict(); m.c_burn_H2 = constraint_dict()
        for a in m.A:
            alpha = arc_alpha[a]
            if alpha <= 0.0:
                continue  # 非輸送アークは不要
            for t in m.T:
                if t not in arc_allowed[a]:
                    continue
                dry = sum(m.sc_fly_var[d,p,SD["dry mass"],a,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                nonprop = sum(
                    m.cnt_com[d,p,a,k,OUT,t]
                    for d in m.sc_des_idx for p in m.sc_copy_idx for k in m.cnt_com_idx
                    if not is_prop(k)
                )
                int_mass = sum(float(int_cost[i]) * m.int_com[d,p,a,i,OUT,t]
                            for d in m.sc_des_idx for p in m.sc_copy_idx for i in m.int_com_idx)
                burn = alpha * (dry + nonprop + int_mass)

                m.c_burn_O2[a,t] = constraint(
                    sum(m.cnt_com[d,p,a,o2,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx) >= phi_O2 * burn
                )
                m.c_burn_H2[a,t] = constraint(
                    sum(m.cnt_com[d,p,a,h2,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx) >= phi_H2 * burn
                )
        
        if self.builder.mode == "Piecewise Linear Approx":
            PiecewiseLinearConstraintsV2(self.builder).set_piecewise_linear_constraints(
                m, pwl_increment
            )
        if self.builder.mode == "fixedSCdesign":
            FixSCDesignV2(self.builder).fix_sc_design(m)

        return m

''' overlay
class setTimeWindow:
    """Class to set time window constraints.
    Commodities and spacecraft cannot fly over arcs outside of their time window.
    """

    def __init__(self, builder: OptModelBuilderV2) -> None:
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
        for sc_des, sc_cp, a, io, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.arc_idx,
            m.io_idx,
            m.time_idx,
        ):
            # 時刻窓でフィルタ（その arc が t に許可されていなければ skip）
            if t not in m.arc_allowed_times[a]:
                continue
            for pi in m.int_com_idx:
                m.int_time_window_const[sc_des, sc_cp, a, pi, io, t] = (
                    constraint(m.int_com[sc_des, sc_cp, a, pi, io, t] == 0)
                )
            for pc in m.cnt_com_idx:
                m.cnt_time_window_const[sc_des, sc_cp, a, pc, io, t] = (
                    constraint(m.cnt_com[sc_des, sc_cp, a, pc, io, t] == 0)
                )
            m.sc_time_window_const[sc_des, sc_cp, a, io, t] = constraint(
                m.sc_fly_ind[sc_des, sc_cp, a, io, t] == 0
            )
        return m

class setSCBigM:
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

    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_sc_big_M_constraints(self, m: block) -> block:
        """Set big-M constraints for the spacecraft flight variables."""
        big_M_val: float = max(self.builder.sc.var_ub)
        m.sc_bigM_const_1 = constraint_dict()
        m.sc_bigM_const_2 = constraint_dict()
        m.sc_bigM_const_3 = constraint_dict()
        for sc_des, sc_cp, sc_var, a, io, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.sc_var_idx,
            m.arc_idx,
            m.io_idx,
            m.time_idx,
        ):
            m.sc_bigM_const_1[sc_des, sc_cp, sc_var, a, io, t] = constraint(
                m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                <= m.sc_fly_ind[sc_des, sc_cp, a, io, t] * big_M_val
            )

            if self.builder.sc_var_dict.inverse[sc_var] == "payload":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        <= m.pl_cap[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        >= m.pl_cap[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, a, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )
            if self.builder.sc_var_dict.inverse[sc_var] == "propellant":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        <= m.prop_cap[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        >= m.prop_cap[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, a, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )
            if self.builder.sc_var_dict.inverse[sc_var] == "dry mass":
                m.sc_bigM_const_2[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        <= m.dry_mass[sc_des]
                    )
                )
                m.sc_bigM_const_3[sc_des, sc_cp, sc_var, a, io, t] = (
                    constraint(
                        m.sc_fly_var[sc_des, sc_cp, sc_var, a, io, t]
                        >= m.dry_mass[sc_des]
                        - (1 - m.sc_fly_ind[sc_des, sc_cp, a, io, t])
                        * self.builder.sc.var_ub[sc_var]
                    )
                )

        return m

class setMassBalance:
    """Class to set mass balance constraints for commodities and SC.

    At each node, the sum of inflow in the previous time step must be greater
    than the sum of outflow in the current time step. If there is demand for
    the commodity of interest, the inflow must be greater than the demand plus
    the outflow. Demand is negative and supply is positive.
    """

    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_mass_balance_constraints(self, m: block) -> block:
        m = self._set_int_com_mass_balance_constraints(m)
        m = self._set_cnt_com_mass_balance_constraints(m)
        m = self._set_sc_balance_constraints(m)
        return m

    def _set_int_com_mass_balance_constraints(self, m: block) -> block:
        """Enforce mass balance for each integer commodity"""
        m.int_com_mass_balance_const = constraint_dict()
        for a, int_com_id, t in product(
            m.arc_idx, m.int_com_idx, m.time_idx
        ):
            t_id = self.builder._network_def.date_to_time_idx_dict[t]
            m.int_com_mass_balance_const[a, int_com_id, t] = constraint(
                sum(
                    m.int_com[
                        sc_des,
                        sc_cp,
                        a,
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
                        a,
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

class setSCCapacity:
    """Class to set Spacecraft capacity constraints.

    Spacecraft cannot carry more than their capacity of payload and propellant.
    The propellant capacity is divided into oxygen (oxydizer) and fuel.
    The oxydizer is assumed to be oxygen.
    """

    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_sc_cap_constraints(self, m: block) -> block:
        m.pl_cap_const = constraint_dict()
        m.oxy_cap_const = constraint_dict()
        m.fuel_cap_const = constraint_dict()
        for sc_des, sc_cp, a, t in product(
            m.sc_des_idx,
            m.sc_copy_idx,
            m.arc_idx,
            m.time_idx,
        ):
            # 時刻窓でフィルタ（その arc が t に許可されていなければ skip）
            if t not in m.arc_allowed_times[a]:
                continue
            m = self._set_payload_cap_constraints(m, sc_des, sc_cp, a, t)
            m = self._set_oxy_cap_constraints(m, sc_des, sc_cp, a, t)
            m = self._set_fuel_cap_constraints(m, sc_des, sc_cp, a, t)
        return m

    def _set_payload_cap_constraints(self, m: block, sc_des, sc_cp, a, t) -> block:
        """Sum of non-propellant commodities cannot exceed SC payload capacity"""
        m.pl_cap_const[sc_des, sc_cp, a, t] = constraint(
            sum(
                self.builder.int_com_costs[self.builder.int_com_dict[pl_name]]
                * m.int_com[
                    sc_des,
                    sc_cp,
                    a,
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
                    a,
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
                a,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m

    def _set_oxy_cap_constraints(self, m: block, sc_des, sc_cp, a, t) -> block:
        """Oxygen mass cannot exceed SC oxygen capacity"""
        m.oxy_cap_const[sc_des, sc_cp, a, t] = constraint(
            m.cnt_com[
                sc_des,
                sc_cp,
                a,
                self.builder.cnt_com_dict["oxygen"],
                self.builder.flow_dict["out"],
                t,
            ]
            <= self.builder.sc.oxi_prop_ratio
            * m.sc_fly_var[
                sc_des,
                sc_cp,
                self.builder.sc_var_dict["propellant"],
                a,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m

    def _set_fuel_cap_constraints(self, m: block, sc_des, sc_cp, a, t) -> block:
        """
        Hydrogen mass cannot exceed SC fuel capacity.
        We assume that the fuel is hydrogen and the oxidizer is oxygen.
        """
        # TODO: non-hydrogen fuel
        m.fuel_cap_const[sc_des, sc_cp, a, t] = constraint(
            m.cnt_com[
                sc_des,
                sc_cp,
                a,
                self.builder.cnt_com_dict["hydrogen"],
                self.builder.flow_dict["out"],
                t,
            ]
            <= self.builder.sc.fuel_prop_ratio
            * m.sc_fly_var[
                sc_des,
                sc_cp,
                self.builder.sc_var_dict["propellant"],
                a,
                self.builder.flow_dict["out"],
                t,
            ]
        )
        return m

class setPropellantConservation:
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

    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_propellant_conservation_constraints(self, m: block) -> block:
        m.prop_mass_cnsv = constraint_dict()

        for a, t, prop_name in product(
            m.arc_idx,
            m.time_idx,
            self.builder.prop_com_names,
        ):
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
            if self.builder.can_operate_ISRU(a):
                self._set_isru_prop_generation_constraints(m, a, t, prop_name)
            else:
                self._set_flight_prop_consumption_constraint(m, a, t, prop_name)
        return m

    def _set_flight_prop_consumption_constraint(self, m, a, t, prop_name):
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
        m.prop_mass_cnsv[a, prop_id, t] = constraint(
            sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    a,
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
                    a,
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
                        a,
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
                        a,
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
                    a,
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_cp in m.sc_copy_idx
                for sc_des in m.sc_des_idx
            )
        )

    def _set_isru_prop_generation_constraints(self, m, a, t, prop_name):
        """
        ISRU plants can generate H2 and O2, where production rate is
        propotional to the mass and work time of the plants.
        H2 and O2 are two distinct commodities.
        """
        prop_id = self.builder.cnt_com_dict[prop_name]
        t_id = self.builder._network_def.date_to_time_idx_dict[t]
        m.prop_mass_cnsv[a, prop_id, t] = constraint(
            sum(
                m.cnt_com[
                    sc_des,
                    sc_cp,
                    a,
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
                    a,
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
'''
