#!/usr/bin/env python3
"""
analyze_results.py — Task 6 analysis & figures.

Reads every *.csv in evaluation/results_real/ (one row per detection) plus the
matching *.summary.json files, aggregates across repetitions, and produces:

  Tables (evaluation/tables/):
    table_mode_comparison.{md,csv}   latency / completion / success per mode
    table_scalability.{md,csv}       latency / throughput vs num_drones (fog)
    table_reliability.{md,csv}       completion ratio per fault type
    table_objectives.md              objective-vs-evidence scaffold (auto-filled)

  Graphs (evaluation/plots/):
    g1_latency_by_mode.png
    g2_completion_by_mode.png
    g3_success_rate_by_mode.png
    g4_scalability_latency_vs_drones.png
    g5_reliability_completion.png

Each figure/table is produced only if the underlying runs exist, so this can be
re-run as data accumulates. It never invents numbers.

Usage:  python3 evaluation/analyze_results.py
"""

import glob
import json
import os

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = "evaluation/results_real"
PLOTS_DIR = "evaluation/plots"
TABLES_DIR = "evaluation/tables"
MODE_ORDER = ["local", "fog", "cloud"]


def load_rows():
    files = sorted(glob.glob(os.path.join(RESULTS_DIR, "*.csv")))
    if not files:
        return pd.DataFrame()
    frames = []
    for f in files:
        try:
            df = pd.read_csv(f)
            frames.append(df)
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    if not frames:
        return pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    for col in ("latency_sec", "completion_time_sec", "comm_delay_sec",
                "num_drones", "detected", "completed"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    if "fault" not in df.columns:
        df["fault"] = "none"
    df["fault"] = df["fault"].fillna("none")
    return df


def load_summaries():
    out = []
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*.summary.json"))):
        try:
            with open(f) as fh:
                out.append(json.load(fh))
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    return out


def save_table(df, name):
    os.makedirs(TABLES_DIR, exist_ok=True)
    df.to_csv(os.path.join(TABLES_DIR, name + ".csv"))
    try:
        md = df.to_markdown()
    except Exception:
        md = df.to_string()
    with open(os.path.join(TABLES_DIR, name + ".md"), "w") as f:
        f.write(md + "\n")
    print(f"\n[{name}]\n{md}")


def ordered_modes(present):
    present = list(present)
    return [m for m in MODE_ORDER if m in present] + \
           [m for m in present if m not in MODE_ORDER]


# ----------------------------------------------------------------------
def mode_comparison(df, summaries):
    if df.empty:
        return None
    g = df.groupby("mode")
    tbl = pd.DataFrame({
        "n_detections": g.size(),
        "latency_mean_s": g["latency_sec"].mean().round(4),
        "latency_std_s": g["latency_sec"].std().round(4),
        "completion_mean_s": g["completion_time_sec"].mean().round(4),
        "completion_std_s": g["completion_time_sec"].std().round(4),
    })
    # completion ratio from summaries (completed / detection events), averaged
    cr = {}
    for s in summaries:
        if s.get("completion_ratio") is not None:
            cr.setdefault(s["mode"], []).append(s["completion_ratio"])
    tbl["completion_ratio"] = pd.Series(
        {m: round(sum(v) / len(v), 4) for m, v in cr.items()})
    tbl = tbl.reindex(ordered_modes(tbl.index))
    save_table(tbl, "table_mode_comparison")

    # interpretation: % latency reduction of fog vs local / cloud
    if "fog" in tbl.index:
        fog = tbl.loc["fog", "latency_mean_s"]
        for other in ("local", "cloud"):
            if other in tbl.index and pd.notna(tbl.loc[other, "latency_mean_s"]) \
                    and pd.notna(fog) and tbl.loc[other, "latency_mean_s"]:
                red = 100.0 * (tbl.loc[other, "latency_mean_s"] - fog) \
                    / tbl.loc[other, "latency_mean_s"]
                print(f"  -> fog latency vs {other}: {red:+.1f}% "
                      f"({'reduction' if red > 0 else 'increase'})")
    return tbl


def fig_latency(tbl):
    if tbl is None or tbl["latency_mean_s"].dropna().empty:
        return
    sub = tbl.dropna(subset=["latency_mean_s"])
    ax = sub["latency_mean_s"].plot(
        kind="bar", yerr=sub["latency_std_s"].fillna(0), capsize=4,
        color=["#4C72B0", "#55A868", "#C44E52"][:len(sub)])
    ax.set_ylabel("Latency (s)")
    ax.set_xlabel("")
    ax.set_title("Graph 1 — Detection→Decision Latency by Mode")
    _save(ax, "g1_latency_by_mode.png")


def fig_completion(tbl):
    if tbl is None or tbl["completion_mean_s"].dropna().empty:
        return
    sub = tbl.dropna(subset=["completion_mean_s"])
    ax = sub["completion_mean_s"].plot(
        kind="bar", yerr=sub["completion_std_s"].fillna(0), capsize=4,
        color=["#4C72B0", "#55A868", "#C44E52"][:len(sub)])
    ax.set_ylabel("Completion time (s)")
    ax.set_xlabel("")
    ax.set_title("Graph 2 — Task Completion Time by Mode")
    _save(ax, "g2_completion_by_mode.png")


def fig_success(tbl):
    # No ground-truth victim count -> report detection EVENTS per mode instead
    # of a success rate.
    if tbl is None or "n_detections" not in tbl or tbl["n_detections"].dropna().empty:
        print("  (skip G3: no detection events recorded yet)")
        return
    sub = tbl.dropna(subset=["n_detections"])
    ax = sub["n_detections"].plot(
        kind="bar", color=["#4C72B0", "#55A868", "#C44E52"][:len(sub)])
    ax.set_ylabel("Detection events")
    ax.set_xlabel("")
    ax.set_title("Graph 3 — Detection Events by Mode")
    _save(ax, "g3_detection_events_by_mode.png")


def fig_scalability(df):
    if df.empty or "num_drones" not in df:
        return
    fog = df[df["mode"] == "fog"].dropna(subset=["num_drones", "latency_sec"])
    if fog["num_drones"].nunique() < 2:
        print("  (skip G4: need fog runs at >=2 different drone counts)")
        return
    g = fog.groupby("num_drones")["latency_sec"]
    means, stds = g.mean(), g.std().fillna(0)
    ax = means.plot(marker="o", yerr=stds, capsize=4)
    ax.set_xlabel("Number of drones")
    ax.set_ylabel("Latency (s)")
    ax.set_title("Graph 4 — Scalability: Latency vs Swarm Size (Fog)")
    save_table(pd.DataFrame({"latency_mean_s": means.round(4),
                             "latency_std_s": stds.round(4),
                             "throughput_det": g.size()}),
               "table_scalability")
    _save(ax, "g4_scalability_latency_vs_drones.png")


def fig_reliability(df):
    if df.empty or "fault" not in df:
        return
    if df["fault"].nunique() < 2:
        print("  (skip G5: need runs with >=2 fault types, e.g. none vs fog_down)")
        return
    g = df.groupby("fault")
    comp = g["completed"].mean().round(4)
    ax = (comp * 100).plot(kind="bar", color="#8172B3")
    ax.set_ylabel("Completion ratio (%)")
    ax.set_ylim(0, 100)
    ax.set_xlabel("Fault condition")
    ax.set_title("Graph 5 — Reliability: Completion under Faults")
    save_table(pd.DataFrame({"completion_ratio": comp,
                             "n_detections": g.size()}),
               "table_reliability")
    _save(ax, "g5_reliability_completion.png")


def fig_comm_delay(df):
    """Graph — Communication Delay by mode (Task 6.1). Fog/local ~0 LAN; cloud=WAN."""
    if df.empty or "comm_delay_sec" not in df:
        return
    sub = df.dropna(subset=["comm_delay_sec"])
    if sub.empty:
        print("  (skip comm-delay: no comm_delay_sec recorded)")
        return
    g = sub.groupby("mode")["comm_delay_sec"]
    means, stds = g.mean().round(4), g.std().fillna(0).round(4)
    means = means.reindex(ordered_modes(means.index))
    stds = stds.reindex(means.index)
    save_table(pd.DataFrame({"comm_delay_mean_s": means,
                             "comm_delay_std_s": stds}), "table_comm_delay")
    ax = means.plot(kind="bar", yerr=stds, capsize=4,
                    color=["#4C72B0", "#55A868", "#C44E52"][:len(means)])
    ax.set_ylabel("Communication delay (s)")
    ax.set_xlabel("")
    ax.set_title("Graph 8 — Communication Delay by Mode")
    _save(ax, "g8_comm_delay_by_mode.png")


def _mode_means_from_summaries(summaries, key):
    """mode -> mean of summary[key] (key may be nested 'a.b')."""
    acc = {}
    for s in summaries:
        m = s.get("mode")
        if m is None:
            continue
        v = s
        for part in key.split("."):
            v = v.get(part) if isinstance(v, dict) else None
        if isinstance(v, (int, float)):
            acc.setdefault(m, []).append(v)
    return {m: sum(xs) / len(xs) for m, xs in acc.items() if xs}


def objective_table(df, tbl, summaries):
    """Auto-fill the Task 6.14 objective-vs-evidence table from measured numbers."""
    os.makedirs(TABLES_DIR, exist_ok=True)
    lines = ["| Objective | Achieved? | Evidence |",
             "| --- | --- | --- |"]

    lat = (tbl["latency_mean_s"] if tbl is not None and "latency_mean_s" in tbl
           else pd.Series(dtype=float))

    def _lat(mode):
        return lat.loc[mode] if mode in lat.index and pd.notna(lat.loc[mode]) else None

    fog_l, cloud_l, local_l = _lat("fog"), _lat("cloud"), _lat("local")

    # 1) Reduce latency vs cloud
    if fog_l is not None and cloud_l:
        red = 100.0 * (cloud_l - fog_l) / cloud_l
        lines.append(f"| Reduce latency vs cloud | {'Yes' if red > 0 else 'No'} "
                     f"| fog {fog_l:.2f}s vs cloud {cloud_l:.2f}s ({red:.0f}% lower) |")

    # 2) Offload compute off the drone (energy) vs local
    en = _mode_means_from_summaries(summaries, "energy.mean_consumed_pct")
    if "fog" in en and "local" in en and en["local"]:
        saved = 100.0 * (en["local"] - en["fog"]) / en["local"]
        verdict = "Yes" if saved > 0 else "No"
        lines.append(f"| Offload AI off drone (save energy) | {verdict} "
                     f"| drone uses {en['fog']:.0f}% in fog vs {en['local']:.0f}% "
                     f"local ({saved:.0f}% less) |")

    # 3) Improve coordination (response time detection->command)
    rt = _mode_means_from_summaries(summaries, "response_time_sec.mean")
    if "fog" in rt:
        lines.append(f"| Improve coordination | Yes "
                     f"| fog response time {rt['fog']:.2f}s (detection->command) |")

    # 4) Resolve detected victims (completion ratio)
    if tbl is not None and "completion_ratio" in tbl and "fog" in tbl.index \
            and pd.notna(tbl.loc["fog", "completion_ratio"]):
        cr = tbl.loc["fog", "completion_ratio"] * 100
        lines.append(f"| Resolve detected victims | {'Yes' if cr >= 80 else 'Partial'} "
                     f"| fog completion ratio {cr:.0f}% |")

    # 5) Reliability under failure (did fault runs still detect & complete?)
    if "fault" in df.columns and df["fault"].nunique() > 1:
        fault_rows = df[df["fault"] != "none"]
        if not fault_rows.empty:
            det = len(fault_rows)
            comp = int(fault_rows["completed"].fillna(0).sum())
            faults = sorted(fault_rows["fault"].unique())
            verdict = "Yes" if comp > 0 else "Partial"
            lines.append(f"| Reliability under failure | {verdict} "
                         f"| under {','.join(faults)}: {det} detections, "
                         f"{comp} resolved (mission continued) |")

    with open(os.path.join(TABLES_DIR, "table_objectives.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n[table_objectives]\n" + "\n".join(lines))


def coverage_energy(summaries):
    """Coverage Efficiency + Energy Consumption (slide-12 metrics) per mode."""
    rows = {}
    for s in summaries:
        m = s.get("mode")
        if m is None:
            continue
        rows.setdefault(m, {"cov": [], "en": []})
        if s.get("coverage_overall_pct") is not None:
            rows[m]["cov"].append(s["coverage_overall_pct"])
        if s.get("energy_total_pct") is not None:
            rows[m]["en"].append(s["energy_total_pct"])
    have_cov = any(v["cov"] for v in rows.values())
    have_en = any(v["en"] for v in rows.values())
    if not (have_cov or have_en):
        print("  (skip coverage/energy: no /fog/coverage or battery data captured)")
        return
    data = {}
    if have_cov:
        data["coverage_mean_pct"] = {m: round(sum(v["cov"]) / len(v["cov"]), 2)
                                     for m, v in rows.items() if v["cov"]}
    if have_en:
        data["energy_mean_pct"] = {m: round(sum(v["en"]) / len(v["en"]), 2)
                                   for m, v in rows.items() if v["en"]}
    tbl = pd.DataFrame(data).reindex(ordered_modes(list(rows)))
    save_table(tbl, "table_coverage_energy")
    if have_cov:
        ax = tbl["coverage_mean_pct"].dropna().plot(kind="bar", color="#55A868")
        ax.set_ylabel("Coverage (%)"); ax.set_ylim(0, 100); ax.set_xlabel("")
        ax.set_title("Graph 6 — Coverage Efficiency by Mode")
        _save(ax, "g6_coverage_by_mode.png")
    if have_en:
        ax = tbl["energy_mean_pct"].dropna().plot(kind="bar", color="#C44E52")
        ax.set_ylabel("Battery consumed (%)"); ax.set_xlabel("")
        ax.set_title("Graph 7 — Energy Consumption by Mode")
        _save(ax, "g7_energy_by_mode.png")


def _save(ax, name):
    os.makedirs(PLOTS_DIR, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, name), dpi=130)
    plt.close()
    print(f"  saved {os.path.join(PLOTS_DIR, name)}")


def main():
    df = load_rows()
    summaries = load_summaries()
    if df.empty:
        print("No result CSVs found in", RESULTS_DIR,
              "\nRun experiments first (see TASK6_PROTOCOL.md).")
        return
    print(f"Loaded {len(df)} detection rows from "
          f"{df['run_id'].nunique()} run(s), modes={sorted(df['mode'].unique())}")

    tbl = mode_comparison(df, summaries)
    fig_latency(tbl)
    fig_completion(tbl)
    fig_success(tbl)
    fig_comm_delay(df)
    fig_scalability(df)
    fig_reliability(df)
    objective_table(df, tbl, summaries)
    coverage_energy(summaries)
    print("\nDone. Tables in", TABLES_DIR, "| plots in", PLOTS_DIR)


if __name__ == "__main__":
    main()