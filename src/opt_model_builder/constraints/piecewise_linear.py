from __future__ import annotations
from typing import TYPE_CHECKING
from pyomo.kernel import (
    block,
    block_dict,
    piecewise_nd,
    piecewise,
)
import numpy as np
from scipy.optimize import root
from scipy.spatial import Delaunay

if TYPE_CHECKING:
    from ..opt_model_builder_class import OptModelBuilder
    from ..opt_model_builder_v2_class import OptModelBuilderV2



class PiecewiseLinearConstraints:
    """
    Class to set piecewise linear constraints for
    spacecraft drymass and ISRU O2 production rate.
    """

    def __init__(self, builder: OptModelBuilder) -> None:
        self.builder = builder

    def set_piecewise_linear_constraints(self, m: block, pwl_increment: float) -> block:
        """
        Defines piecewise linear (PWL) constraints for the pyomo model.
        To express a function in n dimension with PWL, triangulation of
        breakpoints (mesh points) and function values at those breakpoints are
        required. This method calls the helper methods to generate these
        breakpoints and function values for spacecraft dry mass and ISRU O2,
        then defines the PWL constraints the pyomo model.
        """
        m.pwl_drymass = block_dict()
        sc_pwl_bp: dict = self._generate_sc_pwl_breakpoints(pwl_increment)
        for sc_des in m.sc_des_idx:
            m.pwl_drymass[sc_des] = piecewise_nd(
                tri=sc_pwl_bp["triangulation"],
                values=sc_pwl_bp["function values"],
                input=[m.pl_cap[sc_des], m.prop_cap[sc_des]],
                output=m.dry_mass[sc_des],
                bound="eq",
            )

        if self.builder.use_isru:
            m.pwl_isru_O2rate = block_dict()
            isru_pwl_bp: dict = self._generate_isru_pwl_breakpoints()
            for isru in m.isru_des_idx:
                m.pwl_isru_O2rate[isru] = piecewise(
                    breakpoints=isru_pwl_bp["triangulation"],
                    values=isru_pwl_bp["function values"],
                    input=m.isru_mass[isru],
                    output=m.isru_O2rate[isru],
                    bound="eq",
                )
        return m

    def _generate_sc_pwl_breakpoints(
        self, pwl_increment: float, root_tol: float = 1e-9, sol_exist_tol: float = 1e-6
    ) -> dict[str, np.ndarray | Delaunay]:
        """
        Generate data points for piecewise linear approximation of spacecraft dry mass.
        With the current model, spacecraft dry is a function of its payload
        and propellant capacity, along with other fixed parameters. Finding dry
        mass for a given payload and propellant capacity requires root solving.
        This method first tries to find the dry mass for the lower bounds of
        payload capacity and propellant capacity. Then, it increments
        propellant capacity for the fixed payload capacity until the solution
        for the root finding problem does not exist. Since the spacecraft model
        is based on empirical data, the solution may not exist for some extreme
        values of payload and propellant capacity (root solving exits with
        non-zero root gap). Once this point is reached, the method increments
        the payload capacity and repeats the process.
        Args:
            pwl_increment: increment value for piecewise linear approximation
            root_tol: tolerance for root solving
            sol_exist_tol: tolerance for solution existence. If the violation
                of the root solving is less than this value,
                the solution is considered to exist.
        Returns:
            dict: triangulation and function values for dry mass breakpoints
        """
        pl_cap_breakpoints = []
        prop_cap_breakpoints = []
        dry_mass_breakpoints = []
        pl_cap_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["payload"]]
        prop_cap_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["propellant"]]
        dry_mass_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["dry mass"]]
        pl_cap_ub = (
            self.builder.sc.var_ub[self.builder.sc_var_dict["payload"]
                                   ] + pwl_increment
        )

        for pl_cap in range(pl_cap_lb, pl_cap_ub, pwl_increment):
            prop_cap = prop_cap_lb
            dry_mass = dry_mass_lb
            sol = root(
                fun=self.builder._comp_design.sc_sizing.get_drymass_violation,
                x0=dry_mass,
                args=(pl_cap, prop_cap_lb),
                method="hybr",
                tol=root_tol,
            )
            dry_mass = sol.x[0]
            drymass_vio = self.builder._comp_design.sc_sizing.get_drymass_violation(
                dry_mass, pl_cap, prop_cap_lb
            )
            if abs(drymass_vio) >= sol_exist_tol:
                if pl_cap == pl_cap_lb and prop_cap == prop_cap_lb:
                    raise ValueError("""
                    For the given lower bound values of spacecraft payload capacity
                    and propellant capacity, a valid solution for dry mass does not
                    exist. Adjust the lower bounds of payload capacity, propellant
                    capacity, and dry mass according to the spacecraft model.
                    """)
                else:
                    continue

            while abs(drymass_vio) <= sol_exist_tol:
                dry_mass_breakpoints.append(dry_mass)
                pl_cap_breakpoints.append(pl_cap)
                prop_cap_breakpoints.append(prop_cap)

                prop_cap += pwl_increment
                sol = root(
                    fun=self.builder._comp_design.sc_sizing.get_drymass_violation,
                    x0=dry_mass,  # use prev iteration solution as initial guess
                    args=(pl_cap, prop_cap),
                    method="hybr",
                    tol=root_tol,
                )
                dry_mass = sol.x[0]
                drymass_vio = self.builder._comp_design.sc_sizing.get_drymass_violation(
                    dry_mass, pl_cap, prop_cap
                )

        pl_cap_breakpoints = np.array(pl_cap_breakpoints)
        prop_cap_breakpoints = np.array(prop_cap_breakpoints)
        dry_mass_breakpoints = np.array(dry_mass_breakpoints)

        # format the obtained data into the triangulation form
        breakpoints = np.stack(
            (pl_cap_breakpoints, prop_cap_breakpoints), axis=0).T
        triangulation_for_dry_mass = Delaunay(breakpoints)
        n_sc_bp = len(dry_mass_breakpoints)
        print("Number of Breakpoints for SC Design:", n_sc_bp)
        return {
            "triangulation": triangulation_for_dry_mass,
            "function values": dry_mass_breakpoints,
        }

    # FIXME: The ISRU mass breakpoints are hard-coded.
    def _generate_isru_pwl_breakpoints(self) -> dict[str, list]:
        isru_mass_breakpoints: list = [0, 400, 2000, 4000, 6000, 8000, 10000]
        isru_O2rate_breakpoints: list = []
        for isru_mass in isru_mass_breakpoints:
            isru_O2rate = self.builder._comp_design.isru_des.get_isru_O2_rate(
                isru_mass)
            isru_O2rate_breakpoints.append(isru_O2rate)
        return {
            "triangulation": isru_mass_breakpoints,
            "function values": isru_O2rate_breakpoints,
        }

class PiecewiseLinearConstraintsV2:
    """
    Class to set piecewise linear constraints for
    spacecraft drymass and ISRU O2 production rate.
    """

    def __init__(self, builder: OptModelBuilderV2) -> None:
        self.builder = builder

    def set_piecewise_linear_constraints(self, m: block, pwl_increment: float) -> block:
        """
        Defines piecewise linear (PWL) constraints for the pyomo model.
        To express a function in n dimension with PWL, triangulation of
        breakpoints (mesh points) and function values at those breakpoints are
        required. This method calls the helper methods to generate these
        breakpoints and function values for spacecraft dry mass and ISRU O2,
        then defines the PWL constraints the pyomo model.
        """
        m.pwl_drymass = block_dict()
        sc_pwl_bp: dict = self._generate_sc_pwl_breakpoints(pwl_increment)
        for sc_des in m.sc_des_idx:
            m.pwl_drymass[sc_des] = piecewise_nd(
                tri=sc_pwl_bp["triangulation"],
                values=sc_pwl_bp["function values"],
                input=[m.pl_cap[sc_des], m.prop_cap[sc_des]],
                output=m.dry_mass[sc_des],
                bound="eq",
            )

        if self.builder.use_isru:
            m.pwl_isru_O2rate = block_dict()
            isru_pwl_bp: dict = self._generate_isru_pwl_breakpoints()
            for isru in m.isru_des_idx:
                m.pwl_isru_O2rate[isru] = piecewise(
                    breakpoints=isru_pwl_bp["triangulation"],
                    values=isru_pwl_bp["function values"],
                    input=m.isru_mass[isru],
                    output=m.isru_O2rate[isru],
                    bound="eq",
                )
        return m

    def _generate_sc_pwl_breakpoints(
        self, pwl_increment: float, root_tol: float = 1e-9, sol_exist_tol: float = 1e-6
    ) -> dict[str, np.ndarray | Delaunay]:
        """
        Generate data points for piecewise linear approximation of spacecraft dry mass.
        With the current model, spacecraft dry is a function of its payload
        and propellant capacity, along with other fixed parameters. Finding dry
        mass for a given payload and propellant capacity requires root solving.
        This method first tries to find the dry mass for the lower bounds of
        payload capacity and propellant capacity. Then, it increments
        propellant capacity for the fixed payload capacity until the solution
        for the root finding problem does not exist. Since the spacecraft model
        is based on empirical data, the solution may not exist for some extreme
        values of payload and propellant capacity (root solving exits with
        non-zero root gap). Once this point is reached, the method increments
        the payload capacity and repeats the process.
        Args:
            pwl_increment: increment value for piecewise linear approximation
            root_tol: tolerance for root solving
            sol_exist_tol: tolerance for solution existence. If the violation
                of the root solving is less than this value,
                the solution is considered to exist.
        Returns:
            dict: triangulation and function values for dry mass breakpoints
        """
        pl_cap_breakpoints = []
        prop_cap_breakpoints = []
        dry_mass_breakpoints = []
        pl_cap_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["payload"]]
        prop_cap_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["propellant"]]
        dry_mass_lb = self.builder.sc.var_lb[self.builder.sc_var_dict["dry mass"]]
        pl_cap_ub = (
            self.builder.sc.var_ub[self.builder.sc_var_dict["payload"]
                                   ] + pwl_increment
        )

        for pl_cap in range(pl_cap_lb, pl_cap_ub, pwl_increment):
            prop_cap = prop_cap_lb
            dry_mass = dry_mass_lb
            sol = root(
                fun=self.builder._comp_design.sc_sizing.get_drymass_violation,
                x0=dry_mass,
                args=(pl_cap, prop_cap_lb),
                method="hybr",
                tol=root_tol,
            )
            dry_mass = sol.x[0]
            drymass_vio = self.builder._comp_design.sc_sizing.get_drymass_violation(
                dry_mass, pl_cap, prop_cap_lb
            )
            if abs(drymass_vio) >= sol_exist_tol:
                if pl_cap == pl_cap_lb and prop_cap == prop_cap_lb:
                    raise ValueError("""
                    For the given lower bound values of spacecraft payload capacity
                    and propellant capacity, a valid solution for dry mass does not
                    exist. Adjust the lower bounds of payload capacity, propellant
                    capacity, and dry mass according to the spacecraft model.
                    """)
                else:
                    continue

            while abs(drymass_vio) <= sol_exist_tol:
                dry_mass_breakpoints.append(dry_mass)
                pl_cap_breakpoints.append(pl_cap)
                prop_cap_breakpoints.append(prop_cap)

                prop_cap += pwl_increment
                sol = root(
                    fun=self.builder._comp_design.sc_sizing.get_drymass_violation,
                    x0=dry_mass,  # use prev iteration solution as initial guess
                    args=(pl_cap, prop_cap),
                    method="hybr",
                    tol=root_tol,
                )
                dry_mass = sol.x[0]
                drymass_vio = self.builder._comp_design.sc_sizing.get_drymass_violation(
                    dry_mass, pl_cap, prop_cap
                )

        pl_cap_breakpoints = np.array(pl_cap_breakpoints)
        prop_cap_breakpoints = np.array(prop_cap_breakpoints)
        dry_mass_breakpoints = np.array(dry_mass_breakpoints)

        # format the obtained data into the triangulation form
        breakpoints = np.stack(
            (pl_cap_breakpoints, prop_cap_breakpoints), axis=0).T
        triangulation_for_dry_mass = Delaunay(breakpoints)
        n_sc_bp = len(dry_mass_breakpoints)
        print("Number of Breakpoints for SC Design:", n_sc_bp)
        return {
            "triangulation": triangulation_for_dry_mass,
            "function values": dry_mass_breakpoints,
        }

    # FIXME: The ISRU mass breakpoints are hard-coded.
    def _generate_isru_pwl_breakpoints(self) -> dict[str, list]:
        isru_mass_breakpoints: list = [0, 400, 2000, 4000, 6000, 8000, 10000]
        isru_O2rate_breakpoints: list = []
        for isru_mass in isru_mass_breakpoints:
            isru_O2rate = self.builder._comp_design.isru_des.get_isru_O2_rate(
                isru_mass)
            isru_O2rate_breakpoints.append(isru_O2rate)
        return {
            "triangulation": isru_mass_breakpoints,
            "function values": isru_O2rate_breakpoints,
        }
