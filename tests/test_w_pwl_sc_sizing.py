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
    pwl_increment_list=[2500],  # List of PWL increment to try
    store_results_to_csv=False,  # True if results stored to a .csv file
    solver_verbose=False,
)

input_data_1sc = InputData(
    mission=mission_parameters,
    sc=sc_parameters,
    isru=isru_parameters,
    comdty=comdty_details,
    node=node_details,
    runtime=runtime_settings,
)
sl_1sc = SpaceLogistics(input_data_1sc)

ref_imleo_1sc_pwl = 677034.5575013
ref_sc_des_1sc_pwl = np.array(
    [[2827.61503357, 42879.43140199, 12725.75167836]])


def test_cvx_plw_1sc():
    res_1sc_pwl = sl_1sc.optimizer.pwl.solve_w_pwl_approx(2500)
    assert res_1sc_pwl["obj"] == pytest.approx(
        expected=ref_imleo_1sc_pwl, rel=1e-3)
    np.testing.assert_allclose(
        res_1sc_pwl["design vars"],
        ref_sc_des_1sc_pwl,
        rtol=0.05,
    )


mission_parameters_2sc = MissionParameters(
    n_mis=2,  # number of missions
    n_sc_design=2,  # number of SC design
    n_sc_per_design=3,  # number of SC per design
    t_mis_tot=13,  # total single mission duration, days
    t_surf_mis=3,  # lunar surface mission duration, days
    n_crew=4,  # number of crew needed on lunar surface
    sample_mass=1000,  # sample collected from lunar surface, kg
    habit_pl_mass=2000,  # habitat and payload mass, kg
)
input_data_2sc = InputData(
    mission=mission_parameters_2sc,
    sc=sc_parameters,
    isru=isru_parameters,
    comdty=comdty_details,
    node=node_details,
    runtime=runtime_settings,
)
sl_2sc = SpaceLogistics(input_data_2sc)
ref_imleo_2sc_pwl = 401327.7125853
ref_sc_des_2sc_pwl = np.array(
    [
        [2723.30697999, 15349.05682545, 7510.34899967],
        [507.53094029, 55695.85864014, 13829.06781459],
    ]
)


def test_cvx_plw_2sc():
    res_2sc_pwl = sl_2sc.optimizer.pwl.solve_w_pwl_approx(2500)
    assert res_2sc_pwl["obj"] == pytest.approx(
        expected=ref_imleo_2sc_pwl, rel=1e-3)
    np.testing.assert_allclose(
        res_2sc_pwl["design vars"],
        ref_sc_des_2sc_pwl,
        rtol=0.05,
    )
