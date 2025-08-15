# Tutorial

## Quick Start
1. (When using it for the first time) Set up the ``slpy`` environment by installing the packages (follow the non-Linux guide in `windows-installation-guide.md`).
2. Execute ``src/run.py `` in the ``slpy`` environment (after changing any parameters in it as needed). For example, in Anaconda Prompt, move to the src/ folder, and then run
```sh
conda activate slpy
python run.py
```
3. Check the output .csv file in the src/ folder for results.


## Detailed steps
The current version of this library has two space logistics commodity flow solvers: one with fixed, pre-defined spacecraft designs, and one that concurrently optimizes the spacecraft design along with the commodity flow using piecewise linear approximations.

Respectively, these solvers are called using:

```py
model.optimizer.fixed_sc.solve_network_flow_MILP(<fixed spacecraft design>)
```

and

```py
model.optimizer.pwl.solve_w_pwl_approx(<piecewise linear step size>)
```

Where ``model`` should be an already initialized SpaceLogistics object with the input data as an argument, e.g.:

```py
input_data = InputData(
        mission=mission_parameters,
        sc=sc_parameters,
        isru=isru_parameters,
        comdty=comdty_details,
        node=node_details,
        runtime=runtime_settings,
    )
model = SpaceLogistics(input_data)
```

The input data is a collection of all of the architecture components, e.g. the mission requirements, spacecraft parameters, ISRU capability, etc. 

The ``runtime`` parameter contains the inputs to the optimizer itself, rather than defining the model.

An example of these parameter definitions (again from ``src/run.py``) for setting up a concurrent commodity flow/vehicle design problem is copied below:

```py
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
        var_names=["payload", "propellant", "dry mass"],
        misc_mass_fraction=0.05,  # misc mass factor
        aggressive_SC_design=False,  # true if aggressive sizing model is used
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
        cnt_com_names=[
            "plant",
            "maintenance",
            "consumption",
            "habitat",
            "sample",
            "oxygen",
            "hydrogen",
        ],
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
        store_results_to_csv=False,  # True if results stored to a .csv file
        mip_solver="scip",
        solver_verbose=True,
        max_time=3600 * 3,  # maximum time allowed for optimization in seconds
    )
```

Note that for the fixed spacecraft design case, the dry mass, payload capacity, and propellant capacity are not defined in ``sc_parameters``. Instead, they are given as inputs to ``fixed_sc.solve_network_flow_MILP(<fixed spacecraft design>)``.
By default, ``<fixed spacecraft design>`` should be a numpy array in the form ``np.array([<payload>, <propellant>, <dry mass>])``.

To execute our optimization problem, we follow the above points in reverse order:
- Define all inputs parameters
- Create ``SpaceLogistics`` object with the input parameters as its argument.
- Run ``SpaceLogistics.optimizer.pwl.solve_w_pwl_approx()`` or ``SpaceLogistics.optimizer.fixed_sc.solve_network_flow_MILP()``
