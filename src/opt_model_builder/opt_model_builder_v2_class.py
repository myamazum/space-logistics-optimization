from .objective_v2 import Objective
from .constraints_v2_cls import Constraints

from pyomo.kernel import block, variable
import numpy as np
import sys

try:
    from initializer import InitMixin
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from input_data_class_wrapper import NetworkBuilderV2

except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin
    from input_data_class import InputData
    from component_designer.component_designer_class import ComponentDesigner
    from input_data_class_wrapper import NetworkBuilderV2

class OptModelBuilderV2(InitMixin):
    def __init__(
        self,
        input_data: InputData,
        comp_design: ComponentDesigner,
    ) -> None:
        """
        Args:
            input_data: InputData dataclass containing data input from user
            comp_design: ComponentDesigner instance
        """
        self.initialize_attributes(input_data)
        self.input_data = input_data
        self._comp_design = comp_design

        # assign network related attributes
        self._network_def = NetworkBuilderV2(input_data)
        self.time_steps: list[int] = self._network_def.time_steps
        self.first_mis_time_steps: list[int] = self._network_def.first_mis_time_steps
        self.second_mis_time_steps: list[int] = self._network_def.second_mis_time_steps
        self.delta_t: np.ndarray = self._network_def.delta_t
        self.isru_work_time: np.ndarray = self._network_def.isru_work_time
        self.fin_ini_mass_frac: np.ndarray = self._network_def.fin_ini_mass_frac

        # V1
        self.is_feasible_arc = self._network_def.is_feasible_arc
        self.can_operate_ISRU = self._network_def.can_operate_ISRU_arc
        self.is_holdover_arc = self._network_def.is_holdover_arc
        self.is_transportation_arc = self._network_def.is_transportation_arc

        # placeholder
        self.idx_name_dict: dict[str, list[str]] = {}

    @property
    def mode(self) -> str:
        """Mode of the optimization model. Based on this mode,
        the class define different variables/constraints/objective.
        """
        return self._mode

    @mode.setter
    def mode(self, mode: str) -> None:
        if mode not in [
            "Piecewise Linear Approx",
            "fixedSCdesign",
        ]:
            raise ValueError("Mode is invalid")
        self._mode = mode

    @property
    def fixed_sc_vars(self) -> np.ndarray:
        """User-defined or auto-generated spacecraft design variables"""
        return self._fixed_sc_vars

    @fixed_sc_vars.setter
    def fixed_sc_vars(self, fixed_sc_vars: np.ndarray) -> None:
        if not isinstance(fixed_sc_vars, np.ndarray):
            raise ValueError("Fixed SC variables is not a numpy array")
        if fixed_sc_vars.shape != (self.n_sc_design, self.n_sc_vars):
            raise ValueError(
                """Fixed SC variables has invalid nupmy array shape.
                Received: {}
                Expected: ({},{})""".format(
                    fixed_sc_vars.shape,
                    self.n_sc_design,
                    self.n_sc_vars,
                )
            )
        self._fixed_sc_vars = fixed_sc_vars

    def build_model(self, pwl_increment: float = 2500) -> block:
        """build the optimization model based on input data"""
        m: block = block()
        m = Indices(self).set_indices(m)
        m = Variables(self).set_variables(m)
        self._test_index_variable_mapping(m)
        m = Constraints(self).set_constraints(m, pwl_increment)
        m = Objective(self).set_objective(m)
        return m

    def _test_index_variable_mapping(self, model: block) -> None:
        """Check all variables in the model have a corresponding index mapping

        This function checks two things:
        1. All variables in the model have a corresponding index name list
        2. Each index name for each variable is recognized in the master index name list

        Args:
            model: constructed pyomo.kernel block model
        """
        for var in model.component_objects(
            ctype=variable, active=True, descend_into=True
        ):
            assert var.name in self.idx_name_dict.keys(), """
            Variable name {} not found in the variable-index dictionary.
            Each variable in the optimization model needs a list of its indicies.
            """.format(var.name)
            for key in self.idx_name_dict[var.name]:
                assert key in self.idx_name_dict["all"], """
                Index name {} for variable {} is not recognized.
                Make sure to select indicies from the following list: {}
                """.format(key, var.name, self.idx_name_dict["all"])

class Indices:

    def __init__(self, builder: OptModelBuilderV2) -> None:
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

        # InitMixin
        m.sc_des_idx = range(self.builder.n_sc_design)
        m.sc_copy_idx = range(self.builder.n_sc_per_design)
        m.sc_var_idx = range(self.builder.n_sc_vars)
        m.int_com_idx = range(self.builder.n_int_com)
        m.cnt_com_idx = range(self.builder.n_cnt_com)
        m.io_idx = [0, 1]
        m.time_idx = [time for time in self.builder.time_steps]

        # === NEW: arc index ===
        nb = self.builder._network_def   # NetworkBuilderV2 インスタンス推奨
        m.arc_idx = range(len(nb.arc_list))
        # arc→dep/arr の定数マップ（pyomo から参照しやすいように）
        m.arc_dep = {a.id: a.dep for a in nb.arc_list}
        m.arc_arr = {a.id: a.arr for a in nb.arc_list}
        m.arc_alpha = {a.id: a.alpha for a in nb.arc_list}
        m.arc_kind  = {a.id: a.kind for a in nb.arc_list}
        m.arc_allowed_times = {a.id: tuple(nb.allowed_times_by_arc.get(a.id, [])) for a in nb.arc_list}
        # in/out アーク集合（在庫収支で使用）
        m.arcs_by_dep = {n: tuple(nb.arcs_by_dep.get(n, [])) for n in range(self.builder.n_nodes)}
        m.arcs_by_arr = {n: tuple(nb.arcs_by_arr.get(n, [])) for n in range(self.builder.n_nodes)}

        # 既存の dep_node_idx/arr_node_idx は残してもよいが、以降は使用しない
        ...
        # 以降の idx_name_dict は arc ベースに更新
        self.builder.idx_name_dict["int_com"] = ["sc_des","sc_cp","arc","io","time"]
        self.builder.idx_name_dict["cnt_com"] = ["sc_des","sc_cp","arc","io","time"]
        self.builder.idx_name_dict["sc_fly_ind"] = ["sc_des","sc_cp","arc","time"]
        self.builder.idx_name_dict["sc_fly_var"] = ["sc_des","sc_cp","sc_var","arc","time"]

        # DataFrame 出力の都合で dep_node/arr_node 欄も用意（後で arc→(dep,arr) に展開）
        self.builder.idx_name_dict["all"] = [
            "sc_des",
            "sc_cp",
            "sc_var",
            "int_com",
            "cnt_com",
            "arc",
            "dep_node",
            "arr_node",
            "io",
            "time",
        ]

        if self.builder.use_isru:
            m.isru_des_idx = range(self.builder.n_isru_design)
            self.builder.idx_name_dict["all"] += ["isru"]

        return m

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
class Variables:
    def __init__(self, builder: OptModelBuilderV2) -> None:
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

        # 変数添字： (dep,arr) → arc に置換
        for sc_des, sc_cp, a, io, t in product(
                m.sc_des_idx, m.sc_copy_idx, m.arc_idx, m.io_idx, m.time_idx):
            # 時刻窓でフィルタ（その arc が t に許可されていなければ skip）
            if t not in m.arc_allowed_times[a]:
                continue

            # 整数/連続フロー
            for pl_i in m.int_com_idx:
                m.int_com[sc_des, sc_cp, a, pl_i, io, t] = variable(domain=NonNegativeIntegers)
            for pl_c in m.cnt_com_idx:
                m.cnt_com[sc_des, sc_cp, a, pl_c, io, t] = variable(domain=NonNegativeReals)

        # フライト指示 と 連結用の“飛ぶときだけ値を持つ”実体（io に依存しないので別ループで作成）
        for sc_des, sc_cp, a, t in product(
                m.sc_des_idx, m.sc_copy_idx, m.arc_idx, m.time_idx):
            if t not in m.arc_allowed_times[a]:
                continue
            m.sc_fly_ind[sc_des, sc_cp, a, t] = variable(domain=Binary)
            for sc_var in m.sc_var_idx:  # "dry mass","payload","propellant"
                m.sc_fly_var[sc_des, sc_cp, sc_var, a, t] = variable(domain=NonNegativeReals)

        self.builder.idx_name_dict["int_com"] = [
            "sc_des",
            "sc_cp",
            "arc",
            "int_com",
            "io",
            "time",
        ]
        self.builder.idx_name_dict["cnt_com"] = [
            "sc_des",
            "sc_cp",
            "arc",
            "cnt_com",
            "io",
            "time",
        ]
        self.builder.idx_name_dict["sc_fly_ind"] = [
            "sc_des",
            "sc_cp",
            "arc",
            "time",
        ]
        self.builder.idx_name_dict["sc_fly_var"] = [
            "sc_des",
            "sc_cp",
            "sc_var",
            "arc",
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
        for t in m.time_idx:
            m.isru_mass[t] = variable(domain=Reals, lb=0, ub=10000)
            m.isru_O2rate[t] = variable(domain=NonNegativeReals)
        self.builder.idx_name_dict["isru_mass"] = ["time"]
        self.builder.idx_name_dict["isru_O2rate"] = ["time"]
        return m
