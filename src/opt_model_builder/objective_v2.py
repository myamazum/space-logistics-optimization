from __future__ import annotations
from typing import TYPE_CHECKING
from pyomo.kernel import (
    objective,
    constraint,
    minimize,
    block,
)
from pyomo.core.expr.numeric_expr import SumExpression

if TYPE_CHECKING:
    from .opt_model_builder_v2_class import OptModelBuilderV2


class Objective:
    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_objective(self, m: block) -> block:
        """
        Define the objective function of the model.

        Args:
            m: pyomo.kernel model
        """

        m.imleo_def = constraint(m.imleo == self._get_obj_term(m, m.time_idx))
        m.obj = objective(m.imleo, sense=minimize)

        return m

    def _get_obj_term(self, m: block, time_list: list[int]) -> SumExpression:
        """
        Earth→LEO に打ち上げる“重み付き有効質量”（IMLEO）を arc 添字で集計。
        連続/整数コモディティはそれぞれ cnt_com_costs / int_com_costs を重みとして掛ける。
        機体乾燥重量は重み1で加算。
        """
        nb = self.builder._network_def  # NetworkBuilderV2
        node_name = self.builder.node_dict.inverse
        OUT = self.builder.flow_dict["out"]; INN = self.builder.flow_dict["in"]

        # Earth->LEO の輸送アーク集合（複数アークあれば全部含める）
        A_E2L = tuple(
            a.id for a in nb.arc_list
            if a.kind == "transport"
            and node_name[a.dep] == "Earth"
            and node_name[a.arr] == "LEO"
        )

        # 変数は許可された (a,t) のみに作成しているため、その集合に限定
        allowed = {a: set(nb.allowed_times_by_arc.get(a, [])) for a in A_E2L}
        T_eff = {a: [t for t in time_list if t in allowed[a]] for a in A_E2L}

        # 連続コモディティ：重み cnt_com_costs[k]
        term_cnt = sum(
            self.builder.cnt_com_costs[cnt_com] *
            sum(
                m.cnt_com[sc_des, sc_cp, a, cnt_com, OUT, t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
                for t in T_eff[a]
            )
            for a in A_E2L
            for cnt_com in m.cnt_com_idx
        )

        # 整数コモディティ：重み int_com_costs[i]（kg換算）
        term_int = sum(
            self.builder.int_com_costs[int_com] *
            sum(
                m.int_com[sc_des, sc_cp, a, int_com, OUT, t]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
                for t in T_eff[a]
            )
            for a in A_E2L
            for int_com in m.int_com_idx
        )

        # 機体乾燥重量（dry mass）は重み1で加算
        SD = self.builder.sc_var_dict  # {"dry mass":..., "payload":..., "propellant":...}
        term_dry = sum(
            m.sc_fly_var[sc_des, sc_cp, SD["dry mass"], a, OUT, t]
            for a in A_E2L
            for sc_des in m.sc_des_idx
            for sc_cp in m.sc_copy_idx
            for t in T_eff[a]
        )

        return term_cnt + term_int + term_dry
