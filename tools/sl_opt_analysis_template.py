# (content truncated for brevity in this cell) -- The full script was already executed above.
# Rewriting full content to file:

import os
import sys
import math
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt

COLUMN_MAP = {
    "value": "value",
    "variable": "var",
    "commodity": "commodity",
    "from_node": "from",
    "to_node": "to",
    "time": "time",
    "direction": "direction",
    "design_id": "design_id",
    "copy_id": "copy_id",
}

COMMODITY_GROUP_RULES = {
    "crew": ["crew"],
    "propellant": ["oxygen", "hydrogen", "prop"],
    "consumables": ["consumption", "food", "water", "oxyg"],
    "habitat": ["habitat"],
    "sample": ["sample"],
    "maintenance": ["maintenance"],
    "plant": ["plant", "isru"],
    "other": [],
}

EARTH_NODE_NAME = "Earth"
LEO_NODE_NAME = "LEO"

def _require_cols(df, needed):
    missing = [c for c in needed if c not in df.columns]
    if missing:
        raise ValueError(f"Missing required columns: {missing}. Columns found: {list(df.columns)}.")

def load_results(csv_path, scenario_name):
    df = pd.read_csv(csv_path)
    df["scenario"] = scenario_name
    return df

def normalize_columns(df):
    ren = {COLUMN_MAP[k]: k for k in COLUMN_MAP if COLUMN_MAP[k] is not None and COLUMN_MAP[k] in df.columns}
    df = df.rename(columns=ren)
    _require_cols(df, ["value", "variable", "from_node", "to_node", "time"])
    for opt in ["commodity", "direction", "design_id", "copy_id"]:
        if opt not in df.columns:
            df[opt] = None
    return df

def group_commodity(name):
    if name is None or (isinstance(name, float) and math.isnan(name)):
        return "unknown"
    n = str(name).lower()
    for g, keys in COMMODITY_GROUP_RULES.items():
        for k in keys:
            if k.lower() in n:
                return g
    return "other"

def filter_e2leo_outbound(df):
    mask = (df["from_node"] == EARTH_NODE_NAME) & (df["to_node"] == LEO_NODE_NAME)
    if "direction" in df.columns and df["direction"].notna().any():
        out_mask = df["direction"].astype(str).str.lower().str.contains("out")
        mask = mask & out_mask
    return df.loc[mask].copy()

def compute_tlmleo(df):
    e2leo = filter_e2leo_outbound(df)
    if "commodity" in e2leo.columns:
        e2leo["com_group"] = e2leo["commodity"].apply(group_commodity)
    else:
        e2leo["com_group"] = "unknown"
    g = e2leo.groupby(["scenario", "time"])["value"].sum().reset_index(name="TLMLEO")
    return g

def compute_tlmleo_breakdown(df):
    e2leo = filter_e2leo_outbound(df)
    e2leo["com_group"] = e2leo["commodity"].apply(group_commodity)
    g = e2leo.groupby(["scenario", "time", "com_group"])["value"].sum().reset_index()
    g = g.rename(columns={"value": "mass"})
    return g

def compute_design_summary(df):
    cap_mask = df["variable"].astype(str).str.contains("pl_cap|payload", case=False, regex=True)
    prop_mask = df["variable"].astype(str).str.contains("prop_cap|propellant", case=False, regex=True)
    dry_mask = df["variable"].astype(str).str.contains("dry", case=False, regex=True)

    keys = ["scenario"]
    if "design_id" in df.columns:
        keys.append("design_id")
    if "copy_id" in df.columns:
        keys.append("copy_id")

    rows = []
    if cap_mask.any():
        rows.append(df.loc[cap_mask, keys + ["value"]].assign(metric="payload_cap"))
    if prop_mask.any():
        rows.append(df.loc[prop_mask, keys + ["value"]].assign(metric="prop_cap"))
    if dry_mask.any():
        rows.append(df.loc[dry_mask, keys + ["value"]].assign(metric="dry_mass"))

    if not rows:
        return pd.DataFrame()

    out = pd.concat(rows, ignore_index=True)
    pivot = out.pivot_table(index=keys, columns="metric", values="value", aggfunc="max").reset_index()
    return pivot

def plot_tlmleo_over_time(tlmleo, savepath):
    plt.figure()
    for scen, dsg in tlmleo.groupby("scenario"):
        dsg = dsg.sort_values("time")
        plt.plot(dsg["time"], dsg["TLMLEO"], marker="o", label=str(scen))
    plt.xlabel("Time")
    plt.ylabel("TLMLEO (kg)")
    plt.title("Total Launch Mass to LEO over Time")
    plt.legend()
    plt.tight_layout()
    plt.savefig(savepath)
    plt.close()

def plot_tlmleo_breakdown(bd, savepath):
    for scen, dsg in bd.groupby("scenario"):
        pivot = dsg.pivot_table(index="time", columns="com_group", values="mass", aggfunc="sum", fill_value=0)
        pivot = pivot.sort_index()
        plt.figure()
        bottom = np.zeros(len(pivot))
        for col in pivot.columns:
            plt.bar(pivot.index.values, pivot[col].values, bottom=bottom, label=str(col))
            bottom += pivot[col].values
        plt.xlabel("Time")
        plt.ylabel("Mass (kg)")
        plt.title(f"Earth→LEO Outbound Mass Breakdown — {scen}")
        plt.legend()
        plt.tight_layout()
        fname = os.path.splitext(savepath)[0] + f"_{scen}.png"
        plt.savefig(fname)
        plt.close()

def plot_design_scatter(design_df, savepath):
    if design_df.empty or "payload_cap" not in design_df.columns or "prop_cap" not in design_df.columns:
        return
    plt.figure()
    for scen, dsg in design_df.groupby("scenario"):
        x = dsg["payload_cap"]
        y = dsg["prop_cap"]
        labels = None
        if "design_id" in dsg.columns and "copy_id" in dsg.columns:
            labels = dsg["design_id"].astype(str) + "-" + dsg["copy_id"].astype(str)
        plt.scatter(x, y, label=str(scen))
        if labels is not None:
            for xi, yi, lb in zip(x, y, labels):
                plt.annotate(str(lb), (xi, yi), textcoords="offset points", xytext=(5, 3), fontsize=8)
    plt.xlabel("Payload capacity")
    plt.ylabel("Propellant capacity")
    plt.title("Design tradeoff: payload vs propellant capacity")
    plt.legend()
    plt.tight_layout()
    plt.savefig(savepath)
    plt.close()

def build_summary_tables(df):
    tlmleo = compute_tlmleo(df)
    tlm_total = tlmleo.groupby("scenario")["TLMLEO"].sum().reset_index(name="TLMLEO_total")

    flights = None
    sc_fly_mask = df["variable"].astype(str).str.contains("sc_fly_ind", case=False, regex=False)
    if sc_fly_mask.any():
        flights = df.loc[sc_fly_mask].groupby("scenario")["value"].sum().reset_index(name="num_flights")

    design = compute_design_summary(df)

    out = {"tlmleo_over_time": tlmleo, "tlmleo_total": tlm_total, "design": design}
    if flights is not None:
        out["flights"] = flights
    return out

def export_summary_csv(tables, basename):
    for k, v in tables.items():
        if isinstance(v, pd.DataFrame) and not v.empty:
            v.to_csv(f"{basename}_{k}.csv", index=False)

def run_analysis(input_csvs, scenario_names, out_prefix="analysis"):
    if len(input_csvs) != len(scenario_names):
        raise ValueError("input_csvs and scenario_names must have the same length.")

    dfs = []
    for path, name in zip(input_csvs, scenario_names):
        d = load_results(path, name)
        d = normalize_columns(d)
        dfs.append(d)
    df = pd.concat(dfs, ignore_index=True)

    tables = build_summary_tables(df)
    export_summary_csv(tables, out_prefix)

    tlmleo = tables["tlmleo_over_time"]
    plot_tlmleo_over_time(tlmleo, f"{out_prefix}_tlmleo_timeseries.png")

    breakdown = compute_tlmleo_breakdown(df)
    plot_tlmleo_breakdown(breakdown, f"{out_prefix}_tlmleo_breakdown.png")

    design = tables.get("design", pd.DataFrame())
    plot_design_scatter(design, f"{out_prefix}_design_scatter.png")

    return {
        "outputs": {
            "tables": [k for k, v in tables.items() if isinstance(v, pd.DataFrame) and not v.empty],
            "plots": [
                f"{out_prefix}_tlmleo_timeseries.png",
                f"{out_prefix}_tlmleo_breakdown_<scenario>.png",
                f"{out_prefix}_design_scatter.png",
            ]
        }
    }

if __name__ == "__main__":
    args = sys.argv[1:]
    if len(args) == 0:
        print("Usage: python sl_opt_analysis_template.py <csv1> <csv2> ...")
        sys.exit(0)
    csvs = args
    names = [os.path.splitext(os.path.basename(p))[0] for p in csvs]
    print(f"Running analysis for: {list(zip(csvs, names))}")
    result = run_analysis(csvs, names, out_prefix="analysis")
    print("Done. Generated:", result)
