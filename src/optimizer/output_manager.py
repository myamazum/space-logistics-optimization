from __future__ import annotations
from typing import TYPE_CHECKING

if TYPE_CHECKING:
    from .optimizer_class import Optimizer
import pandas as pd
import numpy as np
import os
import sys
import warnings
import pyomo.kernel as kernel
from copy import deepcopy
from datetime import datetime
from itertools import product
from pyomo.kernel import (
    block,
    variable,
    variable_dict,
)

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin


class OutputManager(InitMixin):
    """Class to handle solved pyomo model object in human-readable format"""

    def __init__(self, optimizer: Optimizer) -> None:
        self.optimizer = optimizer
        self.initialize_attributes(self.optimizer._input_data)
        self._network = deepcopy(self.optimizer._network_def)

    def get_sc_vars(self, model: block) -> np.ndarray:
        """extract SC design variables from the solved pyomo model

        Args:
            model: solved pyomo model
        Returns:
            sc_vars: extracted SC design variables,
                shape is (# of sc type, # of sc design variables)
        """
        sc_vars: np.ndarray = np.zeros((self.n_sc_design, self.n_sc_vars))
        for sc_des in range(self.n_sc_design):
            sc_vars[sc_des, self.sc_var_dict["payload"]] = model.pl_cap[sc_des].value
            sc_vars[sc_des, self.sc_var_dict["propellant"]] = model.prop_cap[
                sc_des
            ].value
            sc_vars[sc_des, self.sc_var_dict["dry mass"]] = model.dry_mass[sc_des].value
        return sc_vars

    def write_results(self, model: block) -> pd.DataFrame | None:
        """Write the results of the optimization model to a .csv file

        pyomo.kernel block is the only supported format.
        Saves the results to a .csv file in the data/opt_results directory if specified.

        Args:
            model: Solved optimization model defined as a block in pyomo.kernel
        Returns:
            Pandas dataframe of the reusults
        """
        assert isinstance(model, kernel.block), """
            Model format not supported. Make sure the model is
            pyomo.kernel.block object"""
        df_col = [
            "Variable Name",
            "Value",
        ] + self.optimizer._model_builder.idx_name_dict["all"]
        df = pd.DataFrame(self._extract_var_data(model), columns=df_col)
        df = self.convert_idx_to_name(df)
        df = self._apply_real_dates_to_df(df)
        df = df.drop(df[abs(df["Value"]) < 1e-4].index)
        df["Variable Name"] = pd.Categorical(
            df["Variable Name"], categories=df["Variable Name"].unique(), ordered=True
        )
        df = df.sort_values(by=["Variable Name", "time"], ascending=[True, True])
        if self.runtime.store_results_to_csv:
            filename: str = datetime.now().strftime("%Y_%m_%d_%H%M%S") + ".csv"
            dir = os.path.join("data", "opt_results")
            if os.path.exists(dir):
                df.to_csv(os.path.join(dir, filename), index=False)
            else:
 #               print(
 #                   "Could not find specified directory, output data is saved in the working directory."
 #               )
                df.to_csv(filename, index=False)
        return df

    def _extract_var_data(self, model: block) -> list:
        """Extract variable or variable dict from pyomo kernel block

        Args:
            model: pyomo.kernel.block object
        Returns:
            list: list of optimal design variable name, value, index
        """
        var_data_list = []
        for var in model.component_objects(
            ctype=variable, active=True, descend_into=True
        ):
            if isinstance(var, variable):
                var_data = self._extract_var_name_and_value(var=var)
                var_data_list.append(var_data)
            elif isinstance(var, variable_dict):
                for var_dict_key, var_dict_entry in var.items():
                    if not isinstance(var_dict_entry, variable):
                        # WARNING: This if statement filters PWL variables out.
                        # This is a temporary fix.
                        continue
                    var_data = self._extract_var_dict_entry_name_and_value(
                        var=var,
                        var_dict_key=var_dict_key,
                        var_dict_entry=var_dict_entry,
                    )
                    var_data_list.append(var_data)
            else:
                NotImplementedError("""Data extraction for variables that are
                not defined as variable or variable_dict is not supproted.
                If you use variable_list or variable_tuple, consider using
                variable_dict instead.""")
        return var_data_list

    def _extract_var_name_and_value(
        self,
        var: kernel.variable,
    ) -> dict[str, str | float | None]:
        """Extracts the name and value of pyomo kernel variable obj.

        If the variable's value is defined and its domain type is IntegerSet,
        the value is rounded.

        Args:
            var: The variable to process.

        Returns:
            dict[str, str | float | None]: A dictionary containing the variable name and its value.
        """
        assert isinstance(var, kernel.variable), """
            Expected pyomo.kernel.variable object, but received {}.
            """.format(type(var))
        if var.domain_type is kernel.IntegerSet and var.value:
            var_data = {"Variable Name": var.name, "Value": round(var.value)}
        else:
            var_data = {"Variable Name": var.name, "Value": var.value}
        return var_data

    def _extract_var_dict_entry_name_and_value(
        self,
        var: kernel.variable_dict,
        var_dict_key: int | tuple[int, ...],
        var_dict_entry: kernel.variable,
    ) -> dict[str, str | float | None]:
        """Extracts the name and value of a pyomo kernel variable dictionary entry.

        This method processes a variable dictionary entry and extracts its name and value.
        If the variable's value is defined and its domain type is IntegerSet,
        the value is rounded. Additionally, it handles both integer and tuple keys,
        (int key means var only has one index, while tuple means multiple indices)
        adding the appropriate index names to the resulting dictionary.

        Args:
            var: The variable dictionary containing the variables.
            var_dict_key: The key of the variable dictionary entry,
                which can be an integer or a tuple of integers.
            var_dict_entry: The variable dictionary entry to process.

        Returns:
            dict[str, str | float | None]: A dictionary containing the variable name and its value,
                                            along with any index names if applicable.
        """
        assert isinstance(var_dict_key, int) or isinstance(var_dict_key, tuple), """
        Solved model's index for variable {} is not int or tuple of int.""".format(
            var.name
        )
        if var_dict_entry.value and var_dict_entry.domain_type is kernel.IntegerSet:
            var_data = {
                "Variable Name": var.name,
                "Value": round(var_dict_entry.value),
            }
        else:
            var_data = {"Variable Name": var.name, "Value": var_dict_entry.value}
        if isinstance(var_dict_key, tuple):
            for count, key in enumerate(var_dict_key):
                idx_name = self.optimizer._model_builder.idx_name_dict[var.name][count]
                var_data[idx_name] = key
        elif isinstance(var_dict_key, int):
            if len(self.optimizer._model_builder.idx_name_dict[var.name]) == 1:
                idx_name = self.optimizer._model_builder.idx_name_dict[var.name][0]
                var_data[idx_name] = var_dict_key
        return var_data

    def convert_idx_to_name(self, df: pd.DataFrame) -> pd.DataFrame:
        """Convert index of variables in dataframe to index names

        e.g., node index 0 -> "Earth", node index 1 -> "LEO"
        First convert indecies to integer (e.g., 0.0 -> 0),
        then convert int indecies to names.

        Args:
            df: pandas dataframe of the results
        Returns:
            pandas dataframe with index names
        """
        df.loc[:, ~df.columns.isin(["Value", "Variable Name"])] = df.loc[
            :, ~df.columns.isin(["Value", "Variable Name"])
        ].astype(float)

        df["sc_var"] = df["sc_var"].apply(
            lambda x: self.sc_var_dict.inverse[x] if pd.notna(x) else x
        )
        df["int_com"] = df["int_com"].apply(
            lambda x: self.int_com_dict.inverse[x] if pd.notna(x) else x
        )
        df["cnt_com"] = df["cnt_com"].apply(
            lambda x: self.cnt_com_dict.inverse[x] if pd.notna(x) else x
        )
        df["dep_node"] = df["dep_node"].apply(
            lambda x: self.node_dict.inverse[x] if pd.notna(x) else x
        )
        df["arr_node"] = df["arr_node"].apply(
            lambda x: self.node_dict.inverse[x] if pd.notna(x) else x
        )
        df["io"] = df["io"].apply(
            lambda x: self.flow_dict.inverse[x] if pd.notna(x) else x
        )
        return df

    # WARNING: Construction Zone
    def _apply_real_dates_to_df(
        self,
        df: pd.DataFrame,
    ) -> pd.DataFrame:
        """Apply real dates to solution data

        Needed since outbound and inbound flights happen instanteneously.
        It first offsets the inbound/return mission date by the number of days
        it takes to go to the destination node, complete the mission, and start
        the returning flight. This prevents outbound mission dates from being
        mistakenly considered as inbound mission dates. (e.g., outbound mission
        with real date 1 can be confusing if outbound happens instanteneously
        in date 0 and the inbound in date 1.
        Then, it applies the real dates for transportation and holdover arcs.

        Args:
            df: pandas dataframe containing optimization results
        Returns:
            pd.DataFrame: pandas dataframe with real dates
        """
        if not self.node.is_path_graph:
            warnings.warn("""
            Currently, output file with real time of flight is only supported
            for a path graph. Proceeding with simplified time of flight""")
            return df

        df["time"] = df["time"].apply(
            lambda x: x - 1 + self._network.n_dates_until_return_mis
            if pd.notna(x) and x in self._network.mis_end_dates
            else x
        )
        for dep_node_id, arr_node_id in product(
            range(self.n_nodes),
            range(self.n_nodes),
        ):
            if not self._network.is_feasible_arc(dep_node_id, arr_node_id):
                continue
            if self._network.is_transportation_arc(dep_node_id, arr_node_id):
                df = self._apply_real_dates_in_transportation_arcs(
                    df, dep_node_id, arr_node_id
                )
            if self._network.is_holdover_arc(dep_node_id, arr_node_id):
                df = self._apply_real_dates_in_holdover_arcs(df, dep_node_id)
        df["time"] = df["time"].apply(lambda x: int(x) if pd.notna(x) else x)
        return df

    def _apply_real_dates_in_transportation_arcs(
        self, df: pd.DataFrame, dep_node_id: int, arr_node_id: int
    ) -> pd.DataFrame:
        """Apply real dates to transportation arcs

        Args:
            df: pandas dataframe containing optimization results
            dep_node_id: departure node id
            arr_node_id: arrival node id
        Returns:
            pd.DataFrame: pandas dataframe with real dates for transportation arcs
        """
        dep_node_name, arr_node_name = (
            self.node_dict.inverse[dep_node_id],
            self.node_dict.inverse[arr_node_id],
        )
        mask_out = (
            (df["dep_node"] == dep_node_name)
            & (df["arr_node"] == arr_node_name)
            & (df["io"] == "out")
        )
        mask_in = (
            (df["dep_node"] == dep_node_name)
            & (df["arr_node"] == arr_node_name)
            & (df["io"] == "in")
        )

        df.loc[mask_out, "time"] = (
            df.loc[mask_out, "time"]
            + self._network.get_real_date_from_mis_start(
                dep_node_name,
                arr_node_name,
                self._network.is_outbound_arc(dep_node_id, arr_node_id),
            )
            - self._network.real_arc_time[dep_node_id][arr_node_id]
        )
        df.loc[mask_in, "time"] = df.loc[
            mask_in, "time"
        ] + self._network.get_real_date_from_mis_start(
            dep_node_name,
            arr_node_name,
            self._network.is_outbound_arc(dep_node_id, arr_node_id),
        )
        return df

    # TODO: refactor, too long and complex
    def _apply_real_dates_in_holdover_arcs(
        self,
        df: pd.DataFrame,
        node_id: int,
    ) -> pd.DataFrame:
        """Apply real dates to holdover arcs

        It first deals with the holdover at the designated destination node.
        Then it calculates and applies real dates in holdover at 'middle' nodes.
        In middle nodes, holdover can start in (i) outbound flights,
        where the SC waits until the other SC comes back, or (ii) inbound flights,
        where the SC waits until the other SC joins in the next mission.
        Note that in the raw/un-processed data, holdovers starting in the
        outbound flights (outflow) end in the same outbound flights (inflow),
        and same for inbound. Same for (ii), meaning that the inflow and outflow
        of a holdover can happen on "day 1", but it means it waits
        until the next mission.

        Args:
            df: pandas dataframe containing optimization results
            node_id: holdover node id
        Returns:
            pd.DataFrame: pandas dataframe with real dates for holdover arcs
        """
        node_name = self.node_dict.inverse[node_id]
        mask_out = (
            (df["dep_node"] == node_name)
            & (df["arr_node"] == node_name)
            & (df["io"] == "out")
        )
        mask_in = (
            (df["dep_node"] == node_name)
            & (df["arr_node"] == node_name)
            & (df["io"] == "in")
        )
        if node_name == self.node.destination_node:
            # WARNING: ad-hoc
            # TODO: holdover to next mission at desitnation?
            df.loc[mask_out, "time"] = (
                df.loc[mask_out, "time"]
                + self._network.n_dates_until_return_mis
                - self._network.real_arc_time[node_id][node_id]
            )
            df.loc[mask_in, "time"] = (
                df.loc[mask_in, "time"] + self._network.n_dates_until_return_mis
            )
        else:
            prv_node_name_outbound = self.node.outbound_path[node_id - 1]
            prv_node_name_inbound = self.node.outbound_path[node_id + 1]

            # if holdover arc starts in the inbound trip
            offset_mis_end_dates = [
                date - 1 + self._network.n_dates_until_return_mis
                for date in self._network.mis_end_dates
            ]
            df.loc[
                mask_out & df["time"].isin(offset_mis_end_dates),
                "time",
            ] = df.loc[
                mask_out & df["time"].isin(offset_mis_end_dates),
                "time",
            ] + self._network.get_real_date_from_mis_start(
                dep_node=prv_node_name_inbound,
                arr_node=node_name,
                is_outbound=False,
            )
            # TODO: What if it is the last mission?
            n_dates_until_next_outbout_mission = (
                self.mis.time_interval
                + self._network.get_real_date_from_mis_start(
                    dep_node=prv_node_name_outbound,
                    arr_node=node_name,
                    is_outbound=True,
                )
            )
            df.loc[
                mask_in & df["time"].isin(offset_mis_end_dates),
                "time",
            ] = (
                df.loc[
                    mask_in & df["time"].isin(offset_mis_end_dates),
                    "time",
                ]
                - self._network.n_dates_until_return_mis
                + n_dates_until_next_outbout_mission
            )

            # if holdover arc starts in the outbound trip
            holdover_in_outboud_start_date = self._network.get_real_date_from_mis_start(
                dep_node=prv_node_name_outbound,
                arr_node=node_name,
                is_outbound=True,
            )
            df.loc[
                mask_out & df["time"].isin(self._network.mis_start_dates),
                "time",
            ] = (
                df.loc[
                    mask_out & df["time"].isin(self._network.mis_start_dates),
                    "time",
                ]
                + holdover_in_outboud_start_date
            )
            holdover_in_outboud_end_date = (
                holdover_in_outboud_start_date
                + self._network.real_arc_time[node_id][node_id]
            )
            df.loc[
                mask_in & df["time"].isin(self._network.mis_start_dates),
                # because the inflow for holdover starting
                # in the outbound trip is still in the same outbound trip
                "time",
            ] = (
                df.loc[
                    mask_in & df["time"].isin(self._network.mis_start_dates),
                    "time",
                ]
                + holdover_in_outboud_end_date
            )
        return df
