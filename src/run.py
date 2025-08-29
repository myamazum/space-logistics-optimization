"""
This code specifies parameters/settings needed to run space logsitics/mission planning optimization
with nonlinar spacecraft (SC) sizing constraint
One of the most impactful parameters is the increment used to generate a mesh for
piecewise linear (PWL) approximation of the nonlinear SC sizing constraint.
A user can pass a list of increments, and the code will run for each increment.

For details, refer to:
Multidisciplinary Design Optimization Approach to Integrated Space Logistics and Mission Planning and Spacecraft Design
by M. Isaji, Y. Takubo, and K. Ho
doi: https://doi.org/10.2514/1.A35284
"""

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
import numpy as np

def main():
    mission_parameters = MissionParameters(
        n_mis=2,  # number of missions
        n_sc_design=2,  # number of SC design
        n_sc_per_design=2,  # number of SC per design
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
        is_path_graph=True,
        holdover_nodes=["LEO", "LLO", "LS"],
        outbound_path=["Earth", "LEO", "LLO", "LS"],
    )

    runtime_settings = RuntimeSettings(
        pwl_increment_list=[2500],  # List of PWL increment to try
        store_results_to_csv=True,  # True if results stored to a .csv file
        #mip_solver="scip",
        solver_verbose=True,
        max_time=3600 * 3,  # maximum time allowed for optimization in seconds
    )

    input_data = InputData(
        mission=mission_parameters,
        sc=sc_parameters,
        isru=isru_parameters,
        comdty=comdty_details,
        node=node_details,
        runtime=runtime_settings,
    )

    #fixed_sc_vars=np.array([[1683,16977,7227],[1876,4492,5420]])
    fixed_sc_vars=np.array([[3978,198580,24745],[3978,198580,24745]])

    SpaceLogistics(input_data).optimizer.pwl.solve_w_pwl_approx(pwl_increment=2500)
    #SpaceLogistics(input_data).optimizer.fixed_sc.solve_network_flow_MILP(fixed_sc_vars=fixed_sc_vars)
if __name__ == "__main__":
    main()
