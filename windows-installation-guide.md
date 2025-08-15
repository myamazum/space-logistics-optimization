Building this repo's environment for any OS other than Linux can be unstable, and you might face some unexpected behaviors. Using Linux is ***strongly encouraged*** (if you're on Windows, you can try WSL) and behaviors on other platforms are not guaranteed. 

The following Windows installation was tested successfully with Anaconda 2.6.6 and Visual Studio Code 1.102.0.

```sh
conda create -y --name slpy python==3.11.2
conda activate slpy
conda install -y pygmo==2.19.5
conda install poetry
poetry install
```
Poetry will stop as it cannot find `pygmo` in PyPI channels.
Manually install the rest of dependencies using pip or conda:
```sh
pip install bidict gurobipy icecream pandas pyomo
```
**This list of dependencies is not updated regularly.**

(Optional) Please run `poetry show` to make sure the only uninstalled packages via PyPI (shown in red) are `pygmo` and `pygmo-plugins-nonfree`.
If you see other packages, install them manually as needed.

Also, if necessary, please install the following:
```sh
conda install -c conda-forge mpir
conda install -c conda-forge scip
```

To run the code, run ``run.py``. See ``docs/tutorials.md`` for a quick start and an example usage.

(Optional) To test the code, run the tests at the root directory for this repo and `pygmo`
Note that testing the code requires Gurobi (and therefore requires its active lincese), even though that is not needed for ``run.py`` (which uses open-source SCIP optimizer).
```sh
set PYTHONPATH=src
pytest
python -c "import pygmo; pygmo.test.run_test_suite()"
```
**`pytest` might fail for some tests.**

For any failed tests, check the warning message and look for the relative or absolute error values. If they are not too large, you can ignore the failed tests; otherwise, manually figure out the environment and repeat the process.
