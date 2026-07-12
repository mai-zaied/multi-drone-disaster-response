#!/usr/bin/env python3
"""
analyze_results.py — Task 6 analysis & figures (scenario comparison).

The three Task 6 processing paths are treated as the three SCENARIOS:

    fog    -> "Fog (proposed 3-tier: edge -> fog -> cloud)"
    cloud  -> "Cloud fallback (fog unreachable -> offload + detect on cloud)"
    local  -> "Local fallback (fog + cloud unreachable -> detect on drone)"

It reads, from evaluation/results_real/:
    *_<run>.summary.json   per-run aggregates  (PRIMARY source: always present)
    *_<run>.csv            one row per detection (OPTIONAL: distributions, WAN split)
    *_<run>_battery.csv    battery time series  (OPTIONAL: from battery_logger.py)

and writes tables to evaluation/tables/ and figures to evaluation/plots/.

Design rules:
  * Summary-driven. The headline time comparisons come straight from the
    summary JSONs, so they work even when per-detection CSVs are absent.
  * Never invents numbers. Any figure/table whose inputs are missing is skipped
    with a printed reason, so the same script is safe to re-run as data grows.
  * Multiple runs of the same scenario are averaged (mean of per-run means).

Usage:  python3 evaluation/analyze_results.py
"""

import glob
import json
import os
from collections import defaultdict

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

RESULTS_DIR = "evaluation/results_real"
PLOTS_DIR = "evaluation/plots"
TABLES_DIR = "evaluation/tables"

MODE_ORDER = ["fog", "cloud", "local"]
MODE_LABEL = {
    "fog":   "Fog\n(proposed)",
    "cloud": "Cloud\n(fallback)",
    "local": "Local\n(fallback)",
}
MODE_TITLE = {
    "fog":   "Fog (proposed 3-tier)",
    "cloud": "Cloud fallback (fog down)",
    "local": "Local fallback (fog+cloud down)",
}
MODE_COLOR = {"fog": "#2E7D32", "cloud": "#C44E52", "local": "#4C72B0"}


# ----------------------------------------------------------------------
# loaders
# ----------------------------------------------------------------------
def load_summaries():
    out = []
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*.summary.json"))):
        try:
            with open(f) as fh:
                out.append(json.load(fh))
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    return out


def load_rows():
    """Per-detection CSV rows (optional). Skips the *_battery.csv files."""
    files = [f for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*.csv")))
             if not f.endswith("_battery.csv")]
    frames = []
    for f in files:
        try:
            frames.append(pd.read_csv(f))
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    if not frames:
        return pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    for col in ("latency_sec", "comm_delay_sec", "completion_time_sec",
                "num_drones", "detected", "completed"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    if "fault" not in df.columns:
        df["fault"] = "none"
    df["fault"] = df["fault"].fillna("none")
    return df


def load_battery():
    """Battery time series (optional), tagged with mode/scenario/run."""
    files = sorted(glob.glob(os.path.join(RESULTS_DIR, "*_battery.csv")))
    frames = []
    for f in files:
        try:
            frames.append(pd.read_csv(f))
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    if not frames:
        return pd.DataFrame()
    df = pd.concat(frames, ignore_index=True)
    for col in ("t_rel_sec", "battery_pct"):
        if col in df.columns:
            df[col] = pd.to_numeric(df[col], errors="coerce")
    return df


# ----------------------------------------------------------------------
# helpers
# ----------------------------------------------------------------------
def ordered_modes(present):
    present = list(present)
    return [m for m in MODE_ORDER if m in present] + \
           [m for m in present if m not in MODE_ORDER]


def _get(d, path):
    """Nested get: _get(s, 'latency_sec.mean'); returns None on any miss."""
    cur = d
    for p in path.split("."):
        if not isinstance(cur, dict):
            return None
        cur = cur.get(p)
    return cur


def mode_means(summaries, path):
    """mode -> mean over runs of summary[path] (numeric, non-null)."""
    acc = defaultdict(list)
    for s in summaries:
        m = s.get("mode")
        v = _get(s, path)
        if m is not None and isinstance(v, (int, float)):
            acc[m].append(v)
    return {m: sum(v) / len(v) for m, v in acc.items() if v}


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


def _save(name):
    os.makedirs(PLOTS_DIR, exist_ok=True)
    plt.tight_layout()
    plt.savefig(os.path.join(PLOTS_DIR, name), dpi=130)
    plt.close()
    print(f"  saved {os.path.join(PLOTS_DIR, name)}")


def _bar_by_mode(values, stds, ylabel, title, fname, ylim=None, pct=False):
    """Generic 'metric per scenario' bar chart. `values` is mode->float."""
    modes = ordered_modes(values.keys())
    if not modes:
        print(f"  (skip {fname}: no data)")
        return
    xs = range(len(modes))
    ys = [values[m] for m in modes]
    es = [stds.get(m, 0.0) if stds else 0.0 for m in modes]
    colors = [MODE_COLOR.get(m, "#888888") for m in modes]
    fig, ax = plt.subplots(figsize=(6.2, 4.0))
    bars = ax.bar(xs, ys, yerr=es, capsize=5, color=colors)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([MODE_LABEL.get(m, m) for m in modes])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ylim:
        ax.set_ylim(*ylim)
    for b, y in zip(bars, ys):
        ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                (f"{y:.1f}%" if pct else f"{y:.3f}"),
                ha="center", va="bottom", fontsize=9)
    _save(fname)


# ----------------------------------------------------------------------
# scenario comparison table + headline time figures (summary-driven)
# ----------------------------------------------------------------------
def scenario_table(summaries):
    rows = {}
    for m in ordered_modes({s.get("mode") for s in summaries if s.get("mode")}):
        rows[MODE_TITLE.get(m, m)] = {
            "latency_mean_s":    mode_means(summaries, "latency_sec.mean").get(m),
            "response_mean_s":   mode_means(summaries, "response_time_sec.mean").get(m),
            "completion_mean_s": mode_means(summaries, "completion_time_sec.mean").get(m),
            "completion_ratio":  mode_means(summaries, "completion_ratio").get(m),
            "energy_mean_pct":   mode_means(summaries, "energy.mean_consumed_pct").get(m),
            "coverage_pct":      mode_means(summaries, "coverage_overall_pct").get(m),
            "detection_events":  mode_means(summaries, "detection_events").get(m),
        }
    tbl = pd.DataFrame(rows).T
    save_table(tbl.round(4), "table_scenario_comparison")

    # latency reduction interpretation (fog vs the two fallbacks)
    lat = mode_means(summaries, "latency_sec.mean")
    if "fog" in lat:
        for other in ("cloud", "local"):
            if other in lat and lat[other]:
                red = 100.0 * (lat[other] - lat["fog"]) / lat[other]
                print(f"  -> fog latency vs {other}: {red:+.1f}% "
                      f"({'lower' if red > 0 else 'higher'})")
    return tbl


def fig_time_metrics(summaries):
    """Headline: latency / response / completion, each as a per-scenario bar,
    plus one grouped 'all three' figure."""
    lat = mode_means(summaries, "latency_sec.mean")
    lat_s = mode_means(summaries, "latency_sec.stdev")
    resp = mode_means(summaries, "response_time_sec.mean")
    resp_s = mode_means(summaries, "response_time_sec.stdev")
    comp = mode_means(summaries, "completion_time_sec.mean")
    comp_s = mode_means(summaries, "completion_time_sec.stdev")

    _bar_by_mode(lat, lat_s, "Latency (s)",
                 "Graph 1 - Detection->Decision Latency by Scenario",
                 "g1_latency_by_scenario.png")
    _bar_by_mode(comp, comp_s, "Completion time (s)",
                 "Graph 2 - Task Completion Time by Scenario",
                 "g2_completion_by_scenario.png")
    _bar_by_mode(resp, resp_s, "Response time (s)",
                 "Graph 3 - Detection->Command Response Time by Scenario",
                 "g3_response_by_scenario.png")

    # grouped: latency vs response vs completion, side by side per scenario
    modes = ordered_modes(set(lat) | set(resp) | set(comp))
    if not modes:
        print("  (skip g4 grouped: no time data)")
        return
    series = [("Latency", lat, "#4C72B0"),
              ("Response", resp, "#DD8452"),
              ("Completion", comp, "#55A868")]
    import numpy as np
    x = np.arange(len(modes))
    w = 0.26
    fig, ax = plt.subplots(figsize=(7.2, 4.2))
    for i, (lab, dat, col) in enumerate(series):
        ys = [dat.get(m, float("nan")) for m in modes]
        ax.bar(x + (i - 1) * w, ys, w, label=lab, color=col)
    ax.set_xticks(x)
    ax.set_xticklabels([MODE_LABEL.get(m, m) for m in modes])
    ax.set_ylabel("Seconds")
    ax.set_title("Graph 4 - Time Metrics by Scenario (latency / response / completion)")
    ax.legend()
    _save("g4_time_metrics_grouped.png")


# ----------------------------------------------------------------------
# energy + coverage (summary-driven)
# ----------------------------------------------------------------------
def fig_energy_coverage(summaries):
    en = mode_means(summaries, "energy.mean_consumed_pct")
    _bar_by_mode(en, None, "Battery consumed (%)",
                 "Graph 5 - Energy Consumption by Scenario",
                 "g5_energy_by_scenario.png", pct=True)
    cov = mode_means(summaries, "coverage_overall_pct")
    _bar_by_mode(cov, None, "Coverage (%)",
                 "Graph 6 - Area Coverage by Scenario",
                 "g6_coverage_by_scenario.png", ylim=(0, 100), pct=True)


# ----------------------------------------------------------------------
# battery over time (battery_logger CSVs)
# ----------------------------------------------------------------------
def fig_battery_over_time(batt):
    if batt.empty:
        print("  (skip g7 battery-over-time: no *_battery.csv. "
              "Run battery_logger.py next to the collector.)")
        return
    fig, ax = plt.subplots(figsize=(7.6, 4.4))
    drew = False
    for m in ordered_modes(batt["mode"].unique()):
        sub = batt[batt["mode"] == m].dropna(subset=["t_rel_sec", "battery_pct"])
        if sub.empty:
            continue
        # bin time into 10 s buckets, mean across drones+runs; band = min..max
        sub = sub.copy()
        sub["bin"] = (sub["t_rel_sec"] // 10) * 10
        g = sub.groupby("bin")["battery_pct"]
        mean, lo, hi = g.mean(), g.min(), g.max()
        col = MODE_COLOR.get(m, "#888888")
        ax.plot(mean.index, mean.values, color=col, lw=2,
                label=MODE_TITLE.get(m, m))
        ax.fill_between(mean.index, lo.values, hi.values, color=col, alpha=0.15)
        drew = True
    if not drew:
        print("  (skip g7 battery-over-time: battery CSVs had no usable rows)")
        plt.close()
        return
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Battery (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Graph 7 - Battery Consumption Over Time by Scenario")
    ax.legend()
    _save("g7_battery_over_time.png")


def battery_summary_table(summaries):
    rows = []
    for s in summaries:
        per = _get(s, "energy.per_drone") or {}
        for d, e in per.items():
            rows.append({"scenario": MODE_TITLE.get(s.get("mode"), s.get("mode")),
                         "run_id": s.get("run_id"), "drone": d,
                         "start_pct": e.get("start_pct"),
                         "end_pct": e.get("end_pct"),
                         "consumed_pct": e.get("consumed_pct")})
    if rows:
        save_table(pd.DataFrame(rows), "table_battery_energy")


# ----------------------------------------------------------------------
# cloud latency composition (needs per-detection CSV) — WAN vs inference
# ----------------------------------------------------------------------
def fig_cloud_breakdown(df):
    if df.empty or "comm_delay_sec" not in df:
        print("  (skip g8 cloud breakdown: no per-detection CSV with comm_delay)")
        return
    cloud = df[df["mode"] == "cloud"].dropna(subset=["latency_sec"])
    if cloud.empty:
        print("  (skip g8 cloud breakdown: no cloud detection rows yet)")
        return
    wan = cloud["comm_delay_sec"].fillna(0).mean()
    total = cloud["latency_sec"].mean()
    infer = max(0.0, total - wan)
    fig, ax = plt.subplots(figsize=(4.6, 4.2))
    ax.bar(["Cloud latency"], [infer], color="#55A868", label="Cloud inference")
    ax.bar(["Cloud latency"], [wan], bottom=[infer], color="#C44E52",
           label="WAN round-trip")
    ax.set_ylabel("Seconds")
    ax.set_title("Graph 8 - Cloud Latency Composition")
    ax.legend()
    _save("g8_cloud_latency_breakdown.png")


# ----------------------------------------------------------------------
# scalability (fog latency vs num_drones, across summaries/runs)
# ----------------------------------------------------------------------
def fig_scalability(summaries):
    pts = defaultdict(list)
    for s in summaries:
        if s.get("mode") != "fog":
            continue
        nd = s.get("num_drones")
        lt = _get(s, "latency_sec.mean")
        if isinstance(nd, (int, float)) and isinstance(lt, (int, float)):
            pts[int(nd)].append(lt)
    if len(pts) < 2:
        print("  (skip g9 scalability: need fog runs at >=2 drone counts)")
        return
    xs = sorted(pts)
    ys = [sum(pts[n]) / len(pts[n]) for n in xs]
    fig, ax = plt.subplots(figsize=(6.0, 4.0))
    ax.plot(xs, ys, marker="o", color="#2E7D32")
    ax.set_xlabel("Number of drones")
    ax.set_ylabel("Fog latency (s)")
    ax.set_title("Graph 9 - Scalability: Fog Latency vs Swarm Size")
    save_table(pd.DataFrame({"num_drones": xs, "latency_mean_s": ys}),
               "table_scalability")
    _save("g9_scalability_latency_vs_drones.png")


# ----------------------------------------------------------------------
# reliability (completion ratio per fault, across summaries)
# ----------------------------------------------------------------------
def fig_reliability(summaries):
    acc = defaultdict(list)
    for s in summaries:
        fault = s.get("fault", "none") or "none"
        cr = s.get("completion_ratio")
        if isinstance(cr, (int, float)):
            acc[fault].append(cr)
    if len(acc) < 2:
        print("  (skip g10 reliability: need >=2 fault types with completion_ratio)")
        return
    faults = sorted(acc)
    ys = [100.0 * sum(acc[f]) / len(acc[f]) for f in faults]
    fig, ax = plt.subplots(figsize=(6.4, 4.0))
    ax.bar(faults, ys, color="#8172B3")
    ax.set_ylabel("Completion ratio (%)")
    ax.set_ylim(0, 100)
    ax.set_xlabel("Fault condition")
    ax.set_title("Graph 10 - Reliability: Completion under Faults")
    save_table(pd.DataFrame({"fault": faults, "completion_ratio_pct": ys}),
               "table_reliability")
    _save("g10_reliability_completion.png")


# ----------------------------------------------------------------------
# NEW figures (Task 6.12 checklist)
# ----------------------------------------------------------------------
def _area_of(s):
    """Scanned area (m^2) for a run, from fog coverage telemetry."""
    return _get(s, "coverage.area_m2")


def _scn(s):
    """Human label combining mode + scenario size (e.g. 'fog / large')."""
    return f"{s.get('mode')}/{s.get('scenario')}"


def fig_battery_comparison(summaries):
    """g11 - mean battery consumed (%) per processing tier."""
    en = mode_means(summaries, "energy.mean_consumed_pct")
    if not en:
        print("  (skip g11 battery comparison: no energy in summaries)")
        return
    modes = ordered_modes(en)
    ys = [en[m] for m in modes]
    fig, ax = plt.subplots(figsize=(6.2, 4.2))
    ax.bar([MODE_LABEL.get(m, m) for m in modes], ys,
           color=[MODE_COLOR.get(m, "#888") for m in modes])
    for i, v in enumerate(ys):
        ax.text(i, v, f"{v:.0f}%", ha="center", va="bottom")
    ax.set_ylabel("Mean battery consumed (%)")
    ax.set_title("Graph 11 - Battery Consumption by Tier")
    ax.set_ylim(0, max(ys) * 1.2)
    save_table(pd.DataFrame({"tier": modes, "battery_consumed_pct": ys}),
               "table_battery_comparison")
    _save("g11_battery_comparison.png")


def fig_area_comparison(summaries):
    """g12 - scanned search area (m^2) per run/scenario."""
    rows = [(_scn(s), _area_of(s)) for s in summaries]
    rows = [(k, v) for k, v in rows if isinstance(v, (int, float))]
    if not rows:
        print("  (skip g12 area comparison: no coverage.area_m2 — redeploy fog_server)")
        return
    # mean area per scenario label
    acc = defaultdict(list)
    for k, v in rows:
        acc[k].append(v)
    labels = sorted(acc)
    ys = [sum(acc[k]) / len(acc[k]) for k in labels]
    fig, ax = plt.subplots(figsize=(max(6.0, 1.3 * len(labels)), 4.2))
    ax.bar(labels, ys, color="#4C72B0")
    for i, v in enumerate(ys):
        ax.text(i, v, f"{v:,.0f}", ha="center", va="bottom", fontsize=8)
    ax.set_ylabel("Search area (m$^2$)")
    ax.set_title("Graph 12 - Search Area by Scenario")
    plt.xticks(rotation=20, ha="right")
    save_table(pd.DataFrame({"scenario": labels, "area_m2": ys}),
               "table_area")
    _save("g12_area_comparison.png")


def fig_coverage_vs_area(summaries):
    """g13 - achieved coverage (%) vs scanned area (m^2)."""
    pts = []
    for s in summaries:
        a = _area_of(s)
        c = s.get("coverage_overall_pct")
        if isinstance(a, (int, float)) and isinstance(c, (int, float)):
            pts.append((a, c, s.get("mode")))
    if len(pts) < 2:
        print("  (skip g13 coverage-vs-area: need >=2 runs with area_m2 + coverage)")
        return
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    for m in ordered_modes({p[2] for p in pts}):
        xs = [a for a, c, mm in pts if mm == m]
        ys = [c for a, c, mm in pts if mm == m]
        ax.scatter(xs, ys, s=60, color=MODE_COLOR.get(m, "#888"),
                   label=MODE_TITLE.get(m, m), edgecolor="white", zorder=3)
    ax.set_xlabel("Search area (m$^2$)")
    ax.set_ylabel("Coverage achieved (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Graph 13 - Coverage vs Search Area")
    ax.legend()
    ax.grid(alpha=0.3)
    _save("g13_coverage_vs_area.png")


def fig_battery_and_area(summaries):
    """g14 - battery consumed vs scanned area (efficiency: %/m^2 context)."""
    pts = []
    for s in summaries:
        a = _area_of(s)
        e = _get(s, "energy.mean_consumed_pct")
        if isinstance(a, (int, float)) and isinstance(e, (int, float)) and a > 0:
            pts.append((a, e, s.get("mode")))
    if len(pts) < 2:
        print("  (skip g14 battery-vs-area: need >=2 runs with area_m2 + energy)")
        return
    fig, ax = plt.subplots(figsize=(6.6, 4.4))
    for m in ordered_modes({p[2] for p in pts}):
        xs = [a for a, e, mm in pts if mm == m]
        ys = [e for a, e, mm in pts if mm == m]
        ax.scatter(xs, ys, s=60, color=MODE_COLOR.get(m, "#888"),
                   label=MODE_TITLE.get(m, m), edgecolor="white", zorder=3)
    ax.set_xlabel("Search area (m$^2$)")
    ax.set_ylabel("Mean battery consumed (%)")
    ax.set_title("Graph 14 - Battery Consumption vs Search Area")
    ax.legend()
    ax.grid(alpha=0.3)
    rows = [{"scenario": _scn(s), "area_m2": _area_of(s),
             "battery_consumed_pct": _get(s, "energy.mean_consumed_pct")}
            for s in summaries
            if isinstance(_area_of(s), (int, float))]
    if rows:
        save_table(pd.DataFrame(rows), "table_battery_area")
    _save("g14_battery_and_area.png")


def fig_failure_recovery(summaries, df):
    """g15 - drone-failure recovery timeline.

    Prefers a per-run recovery_sec (fog can log it); otherwise falls back to
    comparing completion time on failure vs no-fault runs so the figure still
    conveys the recovery cost. Skips cleanly if neither is available.
    """
    # direct recovery time if present in summaries
    rec = []
    for s in summaries:
        r = _get(s, "recovery_sec") or _get(s, "failure.recovery_sec")
        if isinstance(r, (int, float)):
            rec.append((s.get("run_id", "run"), r))
    if rec:
        labels = [r[0] for r in rec]
        ys = [r[1] for r in rec]
        fig, ax = plt.subplots(figsize=(max(6.0, 1.2 * len(labels)), 4.2))
        ax.bar(labels, ys, color="#C44E52")
        for i, v in enumerate(ys):
            ax.text(i, v, f"{v:.1f}s", ha="center", va="bottom")
        ax.set_ylabel("Recovery time (s)")
        ax.set_title("Graph 15 - Drone-Failure Recovery Time")
        plt.xticks(rotation=20, ha="right")
        _save("g15_failure_recovery.png")
        return

    # fallback: completion time, fault vs no-fault
    by_fault = defaultdict(list)
    for s in summaries:
        f = (s.get("fault") or "none")
        ct = _get(s, "completion_time_sec.mean")
        if isinstance(ct, (int, float)):
            by_fault["with fault" if f not in ("none", None) else "no fault"].append(ct)
    if len(by_fault) < 2:
        print("  (skip g15 failure recovery: no recovery_sec, and need "
              "fault + no-fault completion times to compare)")
        return
    labels = ["no fault", "with fault"]
    ys = [sum(by_fault[k]) / len(by_fault[k]) if by_fault.get(k) else 0.0
          for k in labels]
    fig, ax = plt.subplots(figsize=(5.6, 4.2))
    ax.bar(labels, ys, color=["#2E7D32", "#C44E52"])
    for i, v in enumerate(ys):
        ax.text(i, v, f"{v:.1f}s", ha="center", va="bottom")
    ax.set_ylabel("Mean completion time (s)")
    ax.set_title("Graph 15 - Recovery Cost: Completion Time under Failure")
    _save("g15_failure_recovery.png")


def fig_fog_utilisation(summaries):
    """g16 - resource utilisation = which TIERS were active in the normal fog
    run, and each tier's own work. The three tiers do different kinds of work
    (edge captures frames, fog runs detection, cloud archives), so this is an
    activity view, NOT a shared %: a fog run engages all three tiers."""
    # pick normal (no-fault) fog runs, average each tier's work
    frames, infers, arch = [], [], []
    for s in summaries:
        if s.get("mode") != "fog":
            continue
        if (s.get("fault") or "none") not in ("none", None):
            continue
        t = _get(s, "tiers")
        if not isinstance(t, dict):
            continue
        frames.append(_get(t, "edge.frames") or 0)
        infers.append(_get(t, "fog.inferences") or 0)
        arch.append(_get(t, "cloud.archive_events")
                    or _get(t, "cloud.archive_batches") or 0)
    if not frames:
        print("  (skip g16 fog utilisation: no fog run with `tiers` block — "
              "redeploy metrics_collector + fog_server)")
        return
    edge_w = sum(frames) / len(frames)
    fog_w = sum(infers) / len(infers)
    cloud_w = sum(arch) / len(arch)
    tiers = ["Edge\n(capture frames)", "Fog\n(detection+coord)", "Cloud\n(archival)"]
    works = [edge_w, fog_w, cloud_w]
    colors = ["#4C72B0", "#2E7D32", "#C44E52"]
    fig, ax = plt.subplots(figsize=(6.6, 4.4))
    bars = ax.bar(tiers, works, color=colors)
    units = ["frames", "inferences", "events"]
    for b, w, u in zip(bars, works, units):
        active = "ACTIVE" if w > 0 else "off"
        ax.text(b.get_x() + b.get_width() / 2, w,
                f"{active}\n{w:,.0f} {u}", ha="center", va="bottom", fontsize=9)
    ax.set_ylabel("Work performed (own unit per tier)")
    ax.set_ylim(0, max(works) * 1.25 if max(works) > 0 else 1)
    ax.set_title("Graph 16 - Resource Utilisation: All Three Tiers Active (Normal Fog)")
    save_table(pd.DataFrame({"tier": ["edge", "fog", "cloud"],
                             "work": works, "unit": units}),
               "table_fog_utilisation")
    _save("g16_fog_utilisation.png")


def utilisation_table(summaries):
    """Per-run tier activity + detection share (%) across all runs."""
    rows = []
    for s in summaries:
        up = (_get(s, "detection_share_pct.total")
              or _get(s, "utilisation_pct.total") or {})
        active = _get(s, "tiers_active") or []
        t = _get(s, "tiers") or {}
        rows.append({
            "scenario": _scn(s), "run_id": s.get("run_id"),
            "fault": s.get("fault", "none"),
            "tiers_active": "+".join(active) if active else "-",
            "edge_frames": _get(t, "edge.frames"),
            "fog_infer": _get(t, "fog.inferences"),
            "cloud_archive_events": _get(t, "cloud.archive_events"),
            "detection_share_fog_pct": up.get("fog"),
            "detection_share_cloud_pct": up.get("cloud"),
            "detection_share_edge_pct": up.get("edge"),
        })
    if rows:
        save_table(pd.DataFrame(rows), "table_utilisation")


# ----------------------------------------------------------------------
# objectives scaffold (Task 6.14)
# ----------------------------------------------------------------------
def objective_table(summaries):
    lat = mode_means(summaries, "latency_sec.mean")
    en = mode_means(summaries, "energy.mean_consumed_pct")
    rt = mode_means(summaries, "response_time_sec.mean")
    cr = mode_means(summaries, "completion_ratio")
    lines = ["| Objective | Achieved? | Evidence |", "| --- | --- | --- |"]

    if "fog" in lat and lat.get("cloud"):
        red = 100.0 * (lat["cloud"] - lat["fog"]) / lat["cloud"]
        lines.append(f"| Reduce latency vs cloud | {'Yes' if red > 0 else 'No'} "
                     f"| fog {lat['fog']:.2f}s vs cloud {lat['cloud']:.2f}s "
                     f"({red:.0f}% lower) |")
    if "fog" in en and en.get("local"):
        saved = 100.0 * (en["local"] - en["fog"]) / en["local"]
        lines.append(f"| Offload AI off drone (save energy) | "
                     f"{'Yes' if saved > 0 else 'No'} | fog {en['fog']:.0f}% vs "
                     f"local {en['local']:.0f}% ({saved:.0f}% less) |")
    if "fog" in rt:
        lines.append(f"| Improve coordination | Yes | fog response "
                     f"{rt['fog']:.2f}s (detection->command) |")
    if "fog" in cr:
        v = cr["fog"] * 100
        lines.append(f"| Resolve detected victims | {'Yes' if v >= 80 else 'Partial'} "
                     f"| fog completion ratio {v:.0f}% |")

    os.makedirs(TABLES_DIR, exist_ok=True)
    with open(os.path.join(TABLES_DIR, "table_objectives.md"), "w") as f:
        f.write("\n".join(lines) + "\n")
    print("\n[table_objectives]\n" + "\n".join(lines))


# ----------------------------------------------------------------------
def main():
    summaries = load_summaries()
    if not summaries:
        print("No *.summary.json found in", RESULTS_DIR,
              "\nRun experiments first (see RUN_GUIDE_full.md).")
        return
    df = load_rows()
    batt = load_battery()
    modes = sorted({s.get("mode") for s in summaries if s.get("mode")})
    print(f"Loaded {len(summaries)} summary file(s); scenarios present: {modes}")
    if not df.empty:
        print(f"  + {len(df)} per-detection rows (optional enrichments enabled)")
    if not batt.empty:
        print(f"  + {len(batt)} battery samples (battery-over-time enabled)")

    scenario_table(summaries)
    fig_time_metrics(summaries)
    fig_energy_coverage(summaries)
    fig_battery_over_time(batt)
    battery_summary_table(summaries)
    fig_cloud_breakdown(df)
    fig_scalability(summaries)
    fig_reliability(summaries)
    # --- Task 6.12 checklist additions ---
    fig_battery_comparison(summaries)     # g11
    fig_area_comparison(summaries)        # g12
    fig_coverage_vs_area(summaries)       # g13
    fig_battery_and_area(summaries)       # g14
    fig_failure_recovery(summaries, df)   # g15
    fig_fog_utilisation(summaries)        # g16
    utilisation_table(summaries)
    objective_table(summaries)
    print("\nDone. Tables in", TABLES_DIR, "| plots in", PLOTS_DIR)


if __name__ == "__main__":
    main()