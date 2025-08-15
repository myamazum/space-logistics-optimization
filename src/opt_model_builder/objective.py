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
    from .opt_model_builder_class import OptModelBuilder


class Objective:
    def __init__(self, builder: OptModelBuilder) -> None:
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
        """Returns sum of commodities and sc mass launched from Earth to LEO
        for a specific scenario over given time interval.

        Args:
            m: pyomo.kernel model
            time_list: list of time steps
        Returns:
            SumExpression: sum of commodities and sc mass launched to LEO
        """
        term = (
            sum(
                self.builder.int_com_costs[int_com]
                * sum(
                    m.int_com[
                        sc_des,
                        sc_cp,
                        self.builder.node_dict["Earth"],
                        self.builder.node_dict["LEO"],
                        int_com,
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for t in time_list
                )
                for int_com in m.int_com_idx
            )
            + sum(
                self.builder.cnt_com_costs[cnt_com]
                * sum(
                    m.cnt_com[
                        sc_des,
                        sc_cp,
                        self.builder.node_dict["Earth"],
                        self.builder.node_dict["LEO"],
                        cnt_com,
                        self.builder.flow_dict["out"],
                        t,
                    ]
                    for sc_des in m.sc_des_idx
                    for sc_cp in m.sc_copy_idx
                    for t in time_list
                )
                for cnt_com in m.cnt_com_idx
            )
            + sum(
                m.sc_fly_var[
                    sc_des,
                    sc_cp,
                    self.builder.sc_var_dict["dry mass"],
                    self.builder.node_dict["Earth"],
                    self.builder.node_dict["LEO"],
                    self.builder.flow_dict["out"],
                    t,
                ]
                for sc_des in m.sc_des_idx
                for sc_cp in m.sc_copy_idx
                for t in time_list
            )
        )
        return term
