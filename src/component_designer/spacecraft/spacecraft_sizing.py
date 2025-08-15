"""
Nonlinear Vehicle Sizing model
Dry mass of a single stage lunar lander is an implicit function of
its payload and propellant capacity.
See https://doi.org/10.2514/1.A35284 for more details.
"""

import sys
import numpy as np
from scipy.optimize import root

try:
    from initializer import InitMixin
except (ModuleNotFoundError, ImportError):
    sys.path.append("..")
    from initializer import InitMixin


class SCSizing(InitMixin):
    def __init__(self, comp_designer) -> None:
        self.comp = comp_designer
        self.initialize_attributes(self.comp._input_data)

    def get_drymass_violation(
        self,
        dry_mass: float,
        payload_cap: float,
        prop_cap: float,
    ) -> float:
        """function to calculate drymass constraint violation
        Equality constraint where the drymass of the SC is a function of
        payload and propellant capacity is defined here.
        If you need to change SC sizing model, this should be
        the only function you need to modify.
        Args:
            dry_mass:    SC dry mass, kg
            payload_cap: SC payload capacity, kg
            prop_cap:    SC fuel capacity, kg
        Returns:
            constraint violation amount
        """

        mstr = 0.3238 * dry_mass + 693.7 * payload_cap**0.04590
        if self.sc.aggressive_SC_design:
            mstr = 0.2694 * dry_mass + 693.7 * payload_cap**0.04590
        mprop = 0.1648 * (dry_mass + payload_cap) + 20.26 * (
            prop_cap / self.sc.prop_density
        )
        mpower = 7.277e-8 * dry_mass**2.443 + 137.0
        mavi = 1.014 * mpower**0.8423 + 22.33 * self.mis.t_surf_mis
        mECLSS = (
            0.004190 * self.mis.n_crew * self.mis.t_surf_mis * dry_mass**0.9061 + 434.7
        )
        mother = self.sc.misc_mass_fraction * dry_mass
        drymass_const_vio = mstr + mprop + mpower + mavi + mECLSS + mother - dry_mass
        return drymass_const_vio

    def get_drymass_violation_wrapper(self, x: np.ndarray) -> float:
        """drymass_violation function wrapper for pygmo"""
        return self.get_drymass_violation(
            dry_mass=x[self.sc_var_dict["dry mass"]],
            payload_cap=x[self.sc_var_dict["payload"]],
            prop_cap=x[self.sc_var_dict["propellant"]],
        )

    def reeval_drymass(self, sc_vars: np.ndarray, tol=1e-6) -> np.ndarray:
        """Calculate dry_mass given payload_cap, prop_cap, and SCparam
        It first calculates an initial guess for root solving of dry mass,
        using even more simplified model of dry mass as a funtion of
        payload and propellant capacity. Then, it solves for the dry mass.
        If the root solving fails, it assigns a very high dry mass.
        Args:
            sc_vars: SC design variables, shape of (n_sc_design, 3)
            tol: tolerance for root solving of equality constraint
        Returns:
            sc_vars (np.ndarray): SC design variables with newly calculated drymass
        """
        assert sc_vars.shape == (self.mis.n_sc_design, self.n_sc_vars), """
        SC design variables must be a np.ndarray with shape of (n_sc_design, 3)."""
        # calculate drymass for each SC design type
        for sc_des in range(self.mis.n_sc_design):
            payload_cap = sc_vars[sc_des][self.sc_var_dict["payload"]]
            prop_cap = sc_vars[sc_des][self.sc_var_dict["propellant"]]
            if self.sc.aggressive_SC_design:
                drymass_guess = (
                    1.26951 * payload_cap**0.920591
                    + 0.00147949 * prop_cap**1.42466
                    + 2717.95
                )
            else:
                drymass_guess = (
                    1.45726089 * payload_cap**0.923022831
                    + (3.78647792e-03) * prop_cap**1.36691195
                    + 2.85597068e03
                )
            sol = root(
                self.get_drymass_violation,
                drymass_guess,
                (payload_cap, prop_cap),
                method="hybr",
                tol=tol,
            )
            dry_mass = sol.x[0]
            if self.get_drymass_violation(dry_mass, payload_cap, prop_cap) > tol:
                dry_mass = 40000
            sc_vars[sc_des][self.sc_var_dict["dry mass"]] = dry_mass
        return sc_vars
