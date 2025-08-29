import numpy as np

from input_data_class import (
    InputData,
    MissionParameters,
    SCParameters,
    ISRUParameters,
    CommodityDetails,
    NodeDetails,
    RuntimeSettings,
)
from component_designer.component_designer_class import ComponentDesigner
from opt_model_builder.opt_model_builder_v2_class import OptModelBuilderV2


def _make_min_input() -> InputData:
    mission_parameters = MissionParameters(
        n_mis=2,
        n_sc_design=1,
        n_sc_per_design=2,
        t_mis_tot=13,
        t_surf_mis=3,
        n_crew=4,
        sample_mass=1000,
        habit_pl_mass=2000,
        consumption_cost=8.655,
        maintenance_cost=0.01,
        time_interval=365,
        use_increased_pl=False,
    )
    sc_parameters = SCParameters(
        isp=420,
        oxi_fuel_ratio=5.5,
        prop_density=360,
        misc_mass_fraction=0.05,
        aggressive_SC_design=False,
    )
    isru_parameters = ISRUParameters(
        use_isru=False,
        n_isru_design=0,
        H2_H2O_ratio=1 / 9,
        O2_H2O_ratio=1 - 1 / 9,
        production_rate=5,
        decay_rate=0.1,
        maintenance_cost=0.05,
    )
    comdty_details = CommodityDetails(
        int_com_names=["crew #"],
        int_com_costs=[100],
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
    node_details = NodeDetails(node_names=["Earth", "LEO", "LLO", "LS"])
    runtime_settings = RuntimeSettings(
        pwl_increment_list=[5000],
        store_results_to_csv=False,
        solver_verbose=False,
    )
    return InputData(
        mission=mission_parameters,
        sc=sc_parameters,
        isru=isru_parameters,
        comdty=comdty_details,
        node=node_details,
        runtime=runtime_settings,
    )


def _build_fixed_model():
    input_data = _make_min_input()
    comp = ComponentDesigner(input_data)
    b = OptModelBuilderV2(input_data, comp)
    b.mode = "fixedSCdesign"
    fixed = np.zeros((input_data.mission.n_sc_design, input_data.sc.n_sc_vars))
    fixed[0, b.sc_var_dict["payload"]] = 3000
    fixed[0, b.sc_var_dict["propellant"]] = 45000
    fixed[0, b.sc_var_dict["dry mass"]] = 12000
    b.fixed_sc_vars = fixed
    m = b.build_model()
    return b, m


def test_v2_idx_name_dict_consistency():
    b, m = _build_fixed_model()
    assert b.idx_name_dict["sc_fly_ind"] == ["sc_des", "sc_cp", "arc", "time"]
    assert b.idx_name_dict["sc_fly_var"] == ["sc_des", "sc_cp", "sc_var", "arc", "time"]
    # ensure output columns include arc and projected dep/arr
    for key in ["sc_des", "sc_cp", "sc_var", "int_com", "cnt_com", "arc", "dep_node", "arr_node", "io", "time"]:
        assert key in b.idx_name_dict["all"]


def test_v2_bigM_and_capacity_constraints_exist():
    b, m = _build_fixed_model()
    nb = b._network_def
    # pick first arc with at least one allowed time
    a = next(a.id for a in nb.arc_list if len(nb.allowed_times_by_arc.get(a.id, [])) > 0)
    t = nb.allowed_times_by_arc[a][0]
    # Big-M linking constraints for first design/copy exist
    assert (0, 0, a, t) in m.c_dm_up
    assert (0, 0, a, t) in m.c_pl_up
    assert (0, 0, a, t) in m.c_pr_up
    # Capacity constraints defined per (a,t)
    assert (a, t) in m.c_sc_payload
    assert (a, t) in m.c_sc_prop


def test_v2_burn_constraints_on_transport_arcs_only():
    b, m = _build_fixed_model()
    nb = b._network_def
    # any transport arc should have alpha>0
    transport_arcs = [x.id for x in nb.arc_list if x.kind == "transport" and x.alpha > 0]
    assert len(transport_arcs) > 0
    a = transport_arcs[0]
    if len(nb.allowed_times_by_arc.get(a, [])) == 0:
        return  # no allowed times to test
    t = nb.allowed_times_by_arc[a][0]
    assert (a, t) in m.c_burn_O2
    assert (a, t) in m.c_burn_H2


def test_v2_flow_propagation_equalities_exist():
    b, m = _build_fixed_model()
    nb = b._network_def
    # pick any arc/time allowed and ensure link constraints exist
    for arc in nb.arc_list:
        times = nb.allowed_times_by_arc.get(arc.id, [])
        if not times:
            continue
        t = times[0]
        # At least for one commodity, equality should be present
        found = any(((arc.id, cc, t) in m.cnt_flow_link) for cc in range(b.n_cnt_com)) or 
                any(((arc.id, ic, t) in m.int_flow_link) for ic in range(b.n_int_com))
        assert found
        break
