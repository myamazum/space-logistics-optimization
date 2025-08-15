import pytest
import numpy as np
from space_logistics import SpaceLogistics
from input_data_class import (
    InputData,
    MissionParameters,
    SCParameters,
    ISRUParameters,
    CommodityDetails,
    NodeDetails,
    RuntimeSettings,
)

mission_parameters = MissionParameters(
    n_mis=2,  # number of missions
    n_sc_design=1,  # number of SC design
    n_sc_per_design=6,  # number of SC per design
    t_mis_tot=13,  # total single mission duration, days
    t_surf_mis=3,  # lunar surface mission duration, days
    n_crew=4,  # number of crew needed on lunar surface
    sample_mass=1000,  # sample collected from lunar surface, kg
    habit_pl_mass=2000,  # habitat and payload mass, kg
    # consumption cost (food+water+oxygen), kg/(day*person)
    consumption_cost=8.655,
    # maintenance cost, fraction/flight (0.01 means 1% per flight)
    maintenance_cost=0.01,
    time_interval=365,  # time interval between missions, days
    use_increased_pl=False,  # true if increased demand is used
)

sc_parameters = SCParameters(
    isp=420,  # specific impulse, s
    oxi_fuel_ratio=5.5,  # oxidizer to fuel ratio
    prop_density=360,  # propellant density, kg/m^3
    misc_mass_fraction=0.05,  # misc mass factor
    aggressive_SC_design=False,  # true if aggressive sizng model is used
)

isru_parameters = ISRUParameters(
    use_isru=False,  # True if ISRU is used
    n_isru_design=0,  # number of ISRU design
    H2_H2O_ratio=1 / 9,  # H2 production per H2O
    O2_H2O_ratio=1 - 1 / 9,  # O2 production per H2O
    production_rate=5,  # production [kg] per year and per mass [kg]
    decay_rate=0.1,  # productivity decay rate per year
    maintenance_cost=0.05,  # cost[kg] per year and per ISRU mass [kg]
)


comdty_details = CommodityDetails(
    int_com_names=["crew #"],  # list of integer commodity names
    int_com_costs=[100],  # list of integer commodity costs
    # list of continuous commodity names
    cnt_com_names=[
        "plant",
        "maintenance",
        "consumption",
        "habitat",
        "sample",
        "oxygen",
        "hydrogen",
    ],
    # list of propellant commodity names
    prop_com_names=["oxygen", "hydrogen"],
)

node_details = NodeDetails(
    node_names=["Earth", "LEO", "LLO", "LS"],  # list of node names
)

runtime_settings = RuntimeSettings(
    pwl_increment_list=[5000],  # List of PWL increment to try
    store_results_to_csv=False,  # True if results stored to a .csv file
    solver_verbose=False,
)

input_data = InputData(
    mission=mission_parameters,
    sc=sc_parameters,
    isru=isru_parameters,
    comdty=comdty_details,
    node=node_details,
    runtime=runtime_settings,
)

sl = SpaceLogistics(input_data)
known_fixed_sc_imleo = 694264.4171277


def test_fixed_sc_design_optimization():
    ref_sc_des = np.array(
        [
            [
                2837.11768506991,
                44362.37800273237,
                13071.917273428024,
            ]
        ]
    )
    fixed_sc_imleo = sl.optimizer.fixed_sc.solve_network_flow_MILP(
        fixed_sc_vars=ref_sc_des
    )
    assert fixed_sc_imleo == pytest.approx(
        expected=known_fixed_sc_imleo, rel=1e-4)
