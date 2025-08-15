from input_data_class import (
    InputData,
    MissionParameters,
    SCParameters,
    ISRUParameters,
    CommodityDetails,
    NodeDetails,
    RuntimeSettings,
)
from bidict import bidict


class InitMixin:
    """Mixin class to initialize attributes for different class instances."""

    def initialize_attributes(self, input_data: InputData):
        """
        Args:
            input_data: InputData dataclass containing data input from user
        """
        # dataclass arguments
        self.input_data: InputData = input_data
        self.mis: MissionParameters = input_data.mission
        self.sc: SCParameters = input_data.sc

        self.isru: ISRUParameters = input_data.isru
        self.comdty: CommodityDetails = input_data.comdty
        self.node: NodeDetails = input_data.node
        self.runtime: RuntimeSettings = input_data.runtime

        # individual data from dataclasses
        self.n_mis: int = input_data.mission.n_mis
        self.n_sc_design: int = input_data.mission.n_sc_design
        self.n_sc_per_design: int = input_data.mission.n_sc_per_design
        self.t_mis_tot: float = input_data.mission.t_mis_tot
        self.t_surf_mis: float = input_data.mission.t_surf_mis
        self.n_crew: int = input_data.mission.n_crew
        self.sample_mass_ls: list[float] = input_data.mission.sample_mass_ls
        self.habit_pl_mass_ls: list[float] = input_data.mission.habit_pl_mass_ls
        self.use_increased_pl: bool = input_data.mission.use_increased_pl
        self.use_isru: bool = input_data.isru.use_isru
        self.n_isru_design: int = input_data.isru.n_isru_design
        self.n_isru_vars: int = input_data.isru.n_isru_vars
        self.com_dict: bidict[str, int] = input_data.com_dict
        self.int_com_dict: bidict[str, int] = input_data.int_com_dict
        self.cnt_com_dict: bidict[str, int] = input_data.cnt_com_dict
        self.node_dict: bidict[str, int] = input_data.node_dict
        self.flow_dict: bidict[str, int] = input_data.flow_dict
        self.sc_var_dict: bidict[str, int] = input_data.sc_var_dict
        self.n_com: int = input_data.comdty.n_com
        self.n_int_com: int = input_data.comdty.n_int_com
        self.n_cnt_com: int = input_data.comdty.n_cnt_com
        self.int_com_names: list[str] = input_data.comdty.int_com_names
        self.int_com_costs: list[float] = input_data.comdty.int_com_costs
        self.cnt_com_names: list[str] = input_data.comdty.cnt_com_names
        self.cnt_com_costs: list[float] = input_data.comdty.cnt_com_costs
        self.prop_com_names: list[str] = input_data.comdty.prop_com_names
        self.n_sc_vars: int = input_data.sc.n_sc_vars
        self.n_nodes: int = input_data.node.n_nodes
