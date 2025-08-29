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
        # Use actual time values (consistent with variable keys)
        m.T = list(self.builder.time_steps)
        OUT = self.builder.flow_dict["out"]; INN = self.builder.flow_dict["in"]

        # arc → 属性
        arc_allowed = {a.id: set(nb.allowed_times_by_arc.get(a.id, [])) for a in nb.arc_list}
        arc_alpha   = {a.id: a.alpha for a in nb.arc_list}
        arc_kind    = {a.id: a.kind  for a in nb.arc_list}

        # v1 互換: 時刻窓外の 0 固定（変数は全 (arc,time) で生成する前提）
        m.int_time_window_const = constraint_dict()
        m.cnt_time_window_const = constraint_dict()
        m.sc_time_window_const  = constraint_dict()
        m.scvar_time_window_const = constraint_dict()
        for a in m.A:
            not_allowed_times = [t for t in m.T if t not in arc_allowed.get(a, set())]
            if not not_allowed_times:
                continue
            for t in not_allowed_times:
                for d in m.sc_des_idx:
                    for p in m.sc_copy_idx:
                        # flows (both io)
                        for io in m.io_idx:
                            for ic in m.int_com_idx:
                                m.int_time_window_const[d,p,a,ic,io,t] = constraint(
                                    m.int_com[d,p,a,ic,io,t] == 0
                                )
                            for cc in m.cnt_com_idx:
                                m.cnt_time_window_const[d,p,a,cc,io,t] = constraint(
                                    m.cnt_com[d,p,a,cc,io,t] == 0
                                )
                        # spacecraft presence and linked value variables (both io)
                        for io in m.io_idx:
                            m.sc_time_window_const[d,p,a,io,t] = constraint(
                                m.sc_fly_ind[d,p,a,io,t] == 0
                            )
                            for sv in m.sc_var_idx:
                                m.scvar_time_window_const[d,p,sv,a,io,t] = constraint(
                                    m.sc_fly_var[d,p,sv,a,io,t] == 0
                                )

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
                        for io in m.io_idx:
                            y = m.sc_fly_ind[d,p,a,io,t]
                            # dry
                            x = m.sc_fly_var[d,p,SD["dry mass"],a,io,t]
                            m.c_dm_up   [d,p,a,io,t] = constraint(x <= DM_UB * y)
                            m.c_dm_lo0  [d,p,a,io,t] = constraint(x >= 0)
                            m.c_dm_cap  [d,p,a,io,t] = constraint(x <= m.dry_mass[d])
                            m.c_dm_eq   [d,p,a,io,t] = constraint(x >= m.dry_mass[d] - DM_UB*(1-y))
                            # payload
                            x = m.sc_fly_var[d,p,SD["payload"],a,io,t]
                            m.c_pl_up   [d,p,a,io,t] = constraint(x <= PL_UB * y)
                            m.c_pl_lo0  [d,p,a,io,t] = constraint(x >= 0)
                            m.c_pl_cap  [d,p,a,io,t] = constraint(x <= m.pl_cap[d])
                            m.c_pl_eq   [d,p,a,io,t] = constraint(x >= m.pl_cap[d] - PL_UB*(1-y))
                            # propellant
                            x = m.sc_fly_var[d,p,SD["propellant"],a,io,t]
                            m.c_pr_up   [d,p,a,io,t] = constraint(x <= PR_UB * y)
                            m.c_pr_lo0  [d,p,a,io,t] = constraint(x >= 0)
                            m.c_pr_cap  [d,p,a,io,t] = constraint(x <= m.prop_cap[d])
                            m.c_pr_eq   [d,p,a,io,t] = constraint(x >= m.prop_cap[d] - PR_UB*(1-y))

        # ---- SCCapacity（非推薬と推進剤の容量上限：各機体ごと）----
        prop_names = set(self.builder.prop_com_names)           # {"oxygen","hydrogen"}
        is_prop = lambda k: (self.builder.cnt_com_names[k] in prop_names)

        m.c_sc_payload = constraint_dict(); m.c_sc_prop = constraint_dict()
        for a in m.A:
            for t in m.T:
                if t not in arc_allowed[a]:
                    continue
                for d in m.sc_des_idx:
                    for p in m.sc_copy_idx:
                        # 非推薬 out の合計（当該機体） ≤ その機体の payload cap
                        payload_out_dp = (
                            sum(
                                m.cnt_com[d,p,a,k,OUT,t]
                                for k in m.cnt_com_idx if not is_prop(k)
                            )
                            + sum(
                                float(self.builder.int_com_costs[i]) * m.int_com[d,p,a,i,OUT,t]
                                for i in m.int_com_idx
                            )
                        )
                        m.c_sc_payload[d,p,a,t] = constraint(
                            payload_out_dp <= m.sc_fly_var[d,p,SD["payload"],a,OUT,t]
                        )

                        # 推進剤（O2+H2） out の合計（当該機体） ≤ その機体の propellant cap
                        o2, h2 = self.builder.cnt_com_dict["oxygen"], self.builder.cnt_com_dict["hydrogen"]
                        prop_cap_dp = m.sc_fly_var[d,p,SD["propellant"],a,OUT,t]
                        prop_out_dp = m.cnt_com[d,p,a,o2,OUT,t] + m.cnt_com[d,p,a,h2,OUT,t]
                        m.c_sc_prop[d,p,a,t] = constraint(prop_out_dp <= prop_cap_dp)

                        # 組成比キャップ（O2/H2 は各割合以内）
                        m.c_sc_o2_ratio = getattr(m, 'c_sc_o2_ratio', constraint_dict())
                        m.c_sc_h2_ratio = getattr(m, 'c_sc_h2_ratio', constraint_dict())
                        m.c_sc_o2_ratio[d,p,a,t] = constraint(
                            m.cnt_com[d,p,a,o2,OUT,t] <= self.builder.sc.oxi_prop_ratio * prop_cap_dp
                        )
                        m.c_sc_h2_ratio[d,p,a,t] = constraint(
                            m.cnt_com[d,p,a,h2,OUT,t] <= self.builder.sc.fuel_prop_ratio * prop_cap_dp
                        )

        # ---- v1-style conservation/consumption constraints ----
        # Integer commodity conservation (crew) per arc/time: IN == OUT
        m.int_com_mass_cnsv = constraint_dict()
        crew_id = self.builder.int_com_dict.get("crew #", None)
        if crew_id is not None:
            for a in m.A:
                for t in m.T:
                    m.int_com_mass_cnsv[a, t] = constraint(
                        sum(m.int_com[d,p,a,crew_id,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                        ==
                        sum(m.int_com[d,p,a,crew_id,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                    )

        # Spacecraft conservation: for each SC design/copy and arc/time: IN == OUT
        m.sc_cnsv = constraint_dict()
        for a in m.A:
            for t in m.T:
                for d in m.sc_des_idx:
                    for p in m.sc_copy_idx:
                        m.sc_cnsv[d,p,a,t] = constraint(
                            m.sc_fly_ind[d,p,a,INN,t] == m.sc_fly_ind[d,p,a,OUT,t]
                        )

        # Continuous commodities (non-propellant) conservation and consumption
        m.cnt_com_cnsv = constraint_dict()
        for a in m.A:
            i = nb.arc_list[a].dep; j = nb.arc_list[a].arr
            for t in m.T:
                t_id = nb.date_to_time_idx_dict[t]
                for cc in m.cnt_com_idx:
                    name = self.builder.cnt_com_names[cc]
                    if name in self.builder.prop_com_names:
                        continue
                    if name == "plant":
                        if self.builder.can_operate_ISRU(a):
                            m.cnt_com_cnsv[a,cc,t] = constraint(
                                sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                ==
                                sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                * (1 - (self.builder.isru.decay_rate * self.builder.isru_work_time[i][t_id] / 365.0))
                            )
                        else:
                            m.cnt_com_cnsv[a,cc,t] = constraint(
                                sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                ==
                                sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            )
                    elif name == "maintenance":
                        if arc_kind[a] == "transport":
                            m.cnt_com_cnsv[a,cc,t] = constraint(
                                sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                ==
                                sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                - self.builder.mis.maintenance_cost
                                * sum(m.sc_fly_var[d,p,SD["dry mass"],a,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            )
                        elif self.builder.can_operate_ISRU(a):
                            m.cnt_com_cnsv[a,cc,t] = constraint(
                                sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                ==
                                sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                - (self.builder.isru.maintenance_cost * self.builder.isru_work_time[i][t_id] / 365.0)
                                * sum(m.cnt_com[d,p,a,self.builder.cnt_com_dict["plant"],OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            )
                        else:
                            m.cnt_com_cnsv[a,cc,t] = constraint(
                                sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                                ==
                                sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            )
                    elif name == "consumption":
                        m.cnt_com_cnsv[a,cc,t] = constraint(
                            sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            ==
                            sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            - self.builder.mis.consumption_cost
                            * self.builder._network_def.real_arc_time[i][j]
                            * sum(m.int_com[d,p,a,self.builder.int_com_dict["crew #"],OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                        )
                    else:
                        m.cnt_com_cnsv[a,cc,t] = constraint(
                            sum(m.cnt_com[d,p,a,cc,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            ==
                            sum(m.cnt_com[d,p,a,cc,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                        )

        # Propellant conservation and consumption (v1-style)
        m.prop_mass_cnsv = constraint_dict()
        for a in m.A:
            i = nb.arc_list[a].dep; j = nb.arc_list[a].arr
            for t in m.T:
                t_id = nb.date_to_time_idx_dict[t]
                for prop_name in self.builder.prop_com_names:
                    prop_id = self.builder.cnt_com_dict[prop_name]
                    if self.builder.can_operate_ISRU(a):
                        ratio = (self.builder.isru.H2_H2O_ratio if prop_name == "hydrogen" else self.builder.isru.O2_H2O_ratio)
                        m.prop_mass_cnsv[a, prop_id, t] = constraint(
                            sum(m.cnt_com[d,p,a,prop_id,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            <=
                            sum(m.cnt_com[d,p,a,prop_id,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            + (self.builder.isru_work_time[i][t_id] / 365.0) * ratio * m.isru_mass[t] * self.builder.isru.production_rate
                        )
                    else:
                        prop_ratio = (self.builder.sc.fuel_prop_ratio if prop_name == "hydrogen" else self.builder.sc.oxi_prop_ratio)
                        m.prop_mass_cnsv[a, prop_id, t] = constraint(
                            sum(m.cnt_com[d,p,a,prop_id,INN,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            <=
                            sum(m.cnt_com[d,p,a,prop_id,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            - prop_ratio * self.builder.fin_ini_mass_frac[i][j][t_id]
                            * (
                                sum(float(self.builder.int_com_costs[ii]) * m.int_com[d,p,a,ii,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx for ii in m.int_com_idx)
                                + sum(m.cnt_com[d,p,a,kk,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx for kk in m.cnt_com_idx)
                                + sum(m.sc_fly_var[d,p,SD["dry mass"],a,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx)
                            )
                        )
        
        if self.builder.mode == "Piecewise Linear Approx":
            PiecewiseLinearConstraintsV2(self.builder).set_piecewise_linear_constraints(
                m, pwl_increment
            )
        if self.builder.mode == "fixedSCdesign":
            FixSCDesignV2(self.builder).fix_sc_design(m)

        # ---- Mass Balance & SC Continuity (minimal v2) ----
        m.int_balance = constraint_dict(); m.cnt_balance = constraint_dict(); m.sc_balance = constraint_dict()
        date_to_idx = nb.date_to_time_idx_dict
        # Node-level balances using arc sets
        for n in range(self.builder.n_nodes):
            dep_arcs = nb.arcs_by_dep.get(n, [])
            arr_arcs = nb.arcs_by_arr.get(n, [])
            for t in m.T:
                t_id = date_to_idx[t]
                # Integer commodities
                for ic in m.int_com_idx:
                    out_sum = sum(
                        m.int_com[d,p,a,ic,OUT,t]
                        for a in dep_arcs if t in arc_allowed.get(a, set())
                        for d in m.sc_des_idx for p in m.sc_copy_idx
                    )
                    in_sum = 0
                    for a in arr_arcs:
                        i = m.arc_dep[a]; j = m.arc_arr[a]
                        dt = int(nb.delta_t[i][j][t_id]) if 0 <= t_id < len(self.builder.time_steps) else 0
                        prev_t = t - dt
                        if prev_t in arc_allowed.get(a, set()):
                            in_sum += sum(
                                m.int_com[d,p,a,ic,INN,prev_t]
                                for d in m.sc_des_idx for p in m.sc_copy_idx
                            )
                    rhs = nb.int_com_demand[n][ic][t_id]
                    m.int_balance[n, ic, t] = constraint(out_sum - in_sum <= rhs)

                # Continuous commodities
                for cc in m.cnt_com_idx:
                    out_sum = sum(
                        m.cnt_com[d,p,a,cc,OUT,t]
                        for a in dep_arcs if t in arc_allowed.get(a, set())
                        for d in m.sc_des_idx for p in m.sc_copy_idx
                    )
                    in_sum = 0
                    for a in arr_arcs:
                        i = m.arc_dep[a]; j = m.arc_arr[a]
                        dt = int(nb.delta_t[i][j][t_id]) if 0 <= t_id < len(self.builder.time_steps) else 0
                        prev_t = t - dt
                        if prev_t in arc_allowed.get(a, set()):
                            in_sum += sum(
                                m.cnt_com[d,p,a,cc,INN,prev_t]
                                for d in m.sc_des_idx for p in m.sc_copy_idx
                            )
                    rhs = nb.cnt_com_demand[n][cc][t_id]
                    m.cnt_balance[n, cc, t] = constraint(out_sum - in_sum <= rhs)

                # Spacecraft continuity (per design/copy)
                for d in m.sc_des_idx:
                    for p in m.sc_copy_idx:
                        out_sc = sum(
                            m.sc_fly_ind[d,p,a,OUT,t]
                            for a in dep_arcs if t in arc_allowed.get(a, set())
                        )
                        in_sc = 0
                        for a in arr_arcs:
                            i = m.arc_dep[a]; j = m.arc_arr[a]
                            dt = int(nb.delta_t[i][j][t_id]) if 0 <= t_id < len(self.builder.time_steps) else 0
                            prev_t = t - dt
                            if prev_t in arc_allowed.get(a, set()):
                                in_sc += m.sc_fly_ind[d,p,a,INN,prev_t]
                        cap = 1 if n == self.builder.node_dict["Earth"] else 0
                        m.sc_balance[d, p, n, t] = constraint(out_sc - in_sc <= cap)

        # NOTE: Do not restrict a spacecraft to at most one arc per time.
        # In this model, arcs are instantaneous per mission boundary; a craft may
        # traverse multiple arcs within the same time bucket. Keeping this
        # unrestricted preserves feasibility for path-graph missions.

        # ---- Flow propagation along arcs ----
        # v1 互換モードではノード収支と時刻窓で整合をとるため、
        # ここでのアーク間の等式連結は設定しない（過拘束回避）。
        m.int_flow_link = constraint_dict(); m.cnt_flow_link = constraint_dict()

        # ---- Arc parallelism cap (if specified per arc) ----
        m.arc_parallel_cap = constraint_dict()
        for a in m.A:
            cap = nb.arc_list[a].max_parallel
            if cap is None:
                continue
            for t in m.T:
                if t not in arc_allowed.get(a, set()):
                    continue
                m.arc_parallel_cap[a,t] = constraint(
                    sum(m.sc_fly_ind[d,p,a,OUT,t] for d in m.sc_des_idx for p in m.sc_copy_idx) <= cap
                )

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
