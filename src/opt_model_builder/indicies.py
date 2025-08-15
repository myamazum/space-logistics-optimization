from __future__ import annotations
from typing import TYPE_CHECKING
from pyomo.kernel import block

if TYPE_CHECKING:
    from .opt_model_builder_class import OptModelBuilder


class Indices:
    """Class to define indices for variables and constraints"""

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_indices(self, m: block) -> block:
        """Define indices and create a dict of all indices

        sc_des_idx: spacecraft design index
        sc_copy_idx: spacecraft copy index/how many spacecraft with
            the same design are allowed in mission
        sc_var_idx: spacecraft design variables index
        dep_node_idx: departure node index
        arr_node_idx: arrival node index
        int_com_idx: integer commodity index
        cnt_com_idx: continuous commodity index
        io_idx: inflow/outflow index: 1-inflow; 0-outflow
        time_idx: time index. For computational puposes, the model is constructed
            so that outbound and inbound (return) missions happen in a single day
            (one day for outbound and another day for inbound). To calculate
            time-related quantities (eg, crew consumables per day), the actual
            time of flight is used.
        """
        m.sc_des_idx = range(self.builder.n_sc_design)
        m.sc_copy_idx = range(self.builder.n_sc_per_design)
        m.sc_var_idx = range(self.builder.n_sc_vars)
        m.dep_node_idx = range(self.builder.n_nodes)
        m.arr_node_idx = range(self.builder.n_nodes)
        m.int_com_idx = range(self.builder.n_int_com)
        m.cnt_com_idx = range(self.builder.n_cnt_com)
        m.io_idx = [0, 1]
        m.time_idx = [time for time in self.builder.time_steps]

        self.builder.idx_name_dict["all"] = [
            "sc_des",
            "sc_cp",
            "sc_var",
            "dep_node",
            "arr_node",
            "int_com",
            "cnt_com",
            "io",
            "time",
        ]

        if self.builder.use_isru:
            m.isru_des_idx = range(self.builder.n_isru_design)
            self.builder.idx_name_dict["all"] += ["isru"]

        return m
