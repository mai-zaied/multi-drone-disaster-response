#!/usr/bin/env python3

"""

Generate evaluation figures and summary tables from experiment result files.

Usage:

    python3 analyze_results.py [results_dir] [output_dir]

Default paths:

    results_dir = evaluation/results_real

    output_dir  = evaluation

"""

import glob
import json
import math
import os
import sys
from collections import defaultdict

import pandas as pd
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# ----------------------------------------------------------------------
# configuration
# ----------------------------------------------------------------------
RESULTS_DIR = sys.argv[1] if len(sys.argv) > 1 else "evaluation/results_real"
OUT_DIR = sys.argv[2] if len(sys.argv) > 2 else "evaluation"
PLOTS_DIR = os.path.join(OUT_DIR, "plots")
TABLES_DIR = os.path.join(OUT_DIR, "tables")

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
DRONE_COLOR = {"drone0": "#4C72B0", "drone1": "#DD8452", "drone2": "#C44E52"}
TIER_COLOR = {"local": "#4C72B0", "fog": "#2E7D32", "cloud": "#C44E52"}

TIER_SCENARIO = "medium"        # tier comparison uses the common-area scenario


# ----------------------------------------------------------------------
# small helpers
# ----------------------------------------------------------------------
def _get(d, path, default=None):
    cur = d
    for p in path.split("."):
        if not isinstance(cur, dict):
            return default
        cur = cur.get(p)
    return cur if cur is not None else default


def _num(v):
    return v if isinstance(v, (int, float)) and not isinstance(v, bool) else None


def ordered_modes(present):
    present = list(present)
    return [m for m in MODE_ORDER if m in present] + \
           [m for m in present if m not in MODE_ORDER]


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
    plt.savefig(os.path.join(PLOTS_DIR, name), dpi=150)
    plt.close()
    print(f"  saved {os.path.join(PLOTS_DIR, name)}")


# ----------------------------------------------------------------------
# loaders (duplicate-suffix tolerant)
# ----------------------------------------------------------------------
def load_summaries():
    """Every file matching *summary*.json, including 'x.summary (1).json'."""
    out = []
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*summary*.json"))):
        try:
            with open(f) as fh:
                s = json.load(fh)
            s["_file"] = os.path.basename(f)
            out.append(s)
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    return out


def load_battery_files():
    """Battery CSVs kept SEPARATE per file (needed for duplicate pairing)."""
    out = []
    for f in sorted(glob.glob(os.path.join(RESULTS_DIR, "*_battery*.csv"))):
        try:
            df = pd.read_csv(f)
            for col in ("t_rel_sec", "battery_pct"):
                df[col] = pd.to_numeric(df[col], errors="coerce")
            df = df.dropna(subset=["t_rel_sec", "battery_pct"])
            out.append((os.path.basename(f), df))
        except Exception as e:
            print(f"  ! skipped {f}: {e}")
    return out


# ----------------------------------------------------------------------
# run selection
# ----------------------------------------------------------------------
def completeness_score(s):
    score = 0
    if (_num(s.get("detection_events")) or 0) > 0:
        score += 4
    if (_get(s, "latency_sec.n", 0) or 0) > 0:
        score += 2
    if (_get(s, "response_time_sec.n", 0) or 0) > 0:
        score += 1
    comp = _num(_get(s, "completion_time_sec.mean"))
    if comp is not None and comp > 0.05:
        score += 2
    return score


def select_runs(summaries):
    """Partition the summaries into figure families. Returns a dict:
         tier   : {mode: summary}      one best run per tier, medium scenario
         area   : [summaries]          fog / no-fault runs with area_m2
         util   : summary or None      fog / no-fault run with `tiers` block
         fail   : summary or None      fault == drone_down run
       plus a text report of every decision taken.
    """
    report = ["# Run-selection report", ""]

    # --- tier comparison: dedupe (mode, scenario, run_id) by score ---------
    groups = defaultdict(list)
    for s in summaries:
        groups[(s.get("mode"), s.get("scenario"), s.get("run_id"))].append(s)

    tier = {}
    report.append("## Tier comparison (scenario = %s)" % TIER_SCENARIO)
    for (mode, scen, rid), cands in sorted(groups.items(), key=lambda k: str(k[0])):
        if scen != TIER_SCENARIO:
            continue
        if (cands[0].get("fault") or "none") == "drone_down":
            continue  # failure run handled separately
        best = max(cands, key=lambda s: (completeness_score(s),
                                         _num(s.get("detection_events")) or 0))
        if len(cands) > 1:
            for c in sorted(cands, key=completeness_score, reverse=True):
                tag = "SELECTED" if c is best else "rejected"
                report.append(
                    f"- {mode}/{rid} `{c['_file']}` -> **{tag}** "
                    f"(score {completeness_score(c)}: events="
                    f"{c.get('detection_events')}, completion_mean="
                    f"{_get(c, 'completion_time_sec.mean')})")
        else:
            report.append(f"- {mode}/{rid} `{best['_file']}` -> **SELECTED** "
                          f"(only candidate)")
        prev = tier.get(mode)
        if prev is None or completeness_score(best) > completeness_score(prev):
            tier[mode] = best

    # --- area scaling: fog, no fault, has coverage.area_m2 -----------------
    area, seen = [], {}
    for s in summaries:
        if s.get("mode") != "fog" or (s.get("fault") or "none") != "none":
            continue
        a = _num(_get(s, "coverage.area_m2"))
        if a is None or a <= 0:
            continue
        key = (s.get("run_id"), a)
        if key in seen:  # exact duplicate file: keep latest coverage snapshot
            if (_get(s, "coverage.ts", 0) or 0) > (_get(seen[key], "coverage.ts", 0) or 0):
                area[area.index(seen[key])] = s
                seen[key] = s
            continue
        seen[key] = s
        area.append(s)
    area.sort(key=lambda s: _get(s, "coverage.area_m2"))
    report.append("\n## Area scaling (fog / no-fault runs with coverage.area_m2)")
    for s in area:
        report.append(f"- `{s['_file']}` area={_get(s, 'coverage.area_m2'):,.0f} m2 "
                      f"dur={s.get('duration_sec')}s cov={s.get('coverage_overall_pct')}%")

    # --- utilisation: fog / no fault / has tiers, prefer TIER_SCENARIO -----
    util_cands = [s for s in summaries
                  if s.get("mode") == "fog"
                  and (s.get("fault") or "none") == "none"
                  and isinstance(s.get("tiers"), dict)]
    util = None
    if util_cands:
        util = sorted(util_cands,
                      key=lambda s: (s.get("scenario") != TIER_SCENARIO,
                                     -(_get(s, "tiers.fog.inferences", 0) or 0)))[0]
    report.append("\n## Utilisation (normal fog)")
    report.append(f"- {'`' + util['_file'] + '` SELECTED' if util else 'none available'}")

    # --- failure run --------------------------------------------------------
    fail = None
    for s in summaries:
        if "drone" in (s.get("fault") or ""):
            fail = s
    report.append("\n## Failure recovery")
    report.append(f"- {'`' + fail['_file'] + '` SELECTED' if fail else 'none available'}")

    return {"tier": tier, "area": area, "util": util, "fail": fail,
            "report": "\n".join(report)}


def pair_battery(summary, battery_files):
    """Battery CSV for a summary. Key match on (mode, scenario, run_id);
    ties broken by comparing battery at mission end vs summary end_pct."""
    if summary is None:
        return None
    key = (summary.get("mode"), summary.get("scenario"), summary.get("run_id"))
    cands = []
    for name, df in battery_files:
        if df.empty:
            continue
        k = (str(df["mode"].iloc[0]), str(df["scenario"].iloc[0]),
             str(df["run_id"].iloc[0]))
        if k == key:
            cands.append((name, df))
    if not cands:
        return None
    if len(cands) == 1:
        return cands[0]
    end_pct = {d: _num(e.get("end_pct"))
               for d, e in (_get(summary, "energy.per_drone") or {}).items()}
    dur = _num(summary.get("duration_sec")) or 1e9

    def err(item):
        _, df = item
        diffs = []
        for d, target in end_pct.items():
            if target is None:
                continue
            sub = df[(df["drone"] == d) & (df["t_rel_sec"] <= dur)]
            if sub.empty:
                sub = df[df["drone"] == d]
            if sub.empty:
                continue
            diffs.append(abs(sub["battery_pct"].iloc[-1] - target))
        return sum(diffs) / len(diffs) if diffs else 1e9

    best = min(cands, key=err)
    return best


# ----------------------------------------------------------------------
# generic tier bar chart
# ----------------------------------------------------------------------
def _tier_bar(tier, path_mean, path_std, path_n, ylabel, title, fname,
              ylim=None, pct=False, note=None):
    modes = ordered_modes(tier.keys())
    vals, errs, ns = [], [], []
    for m in modes:
        vals.append(_num(_get(tier[m], path_mean)))
        errs.append((_num(_get(tier[m], path_std)) or 0.0) if path_std else 0.0)
        ns.append((_get(tier[m], path_n, 0) or 0) if path_n else 0)
    keep = [i for i, v in enumerate(vals) if v is not None]
    if not keep:
        print(f"  (skip {fname}: no data)")
        return
    modes = [modes[i] for i in keep]
    vals = [vals[i] for i in keep]
    errs = [errs[i] for i in keep]
    ns = [ns[i] for i in keep]
    fig, ax = plt.subplots(figsize=(6.4, 4.4))
    bars = ax.bar(range(len(modes)), vals, yerr=errs, capsize=5,
                  color=[MODE_COLOR.get(m, "#888") for m in modes],
                  edgecolor="white")
    ax.set_xticks(range(len(modes)))
    ax.set_xticklabels([MODE_LABEL.get(m, m) for m in modes])
    ax.set_ylabel(ylabel)
    ax.set_title(title)
    if ylim:
        ax.set_ylim(*ylim)
    for b, v, e, n in zip(bars, vals, errs, ns):
        txt = f"{v:.1f}%" if pct else f"{v:.2f} s"
        if n:
            txt += f"\n(n={n})"
        ax.text(b.get_x() + b.get_width() / 2, b.get_height() + e,
                txt, ha="center", va="bottom", fontsize=9)
    if note:
        ax.annotate(note, xy=(0.5, -0.22), xycoords="axes fraction",
                    ha="center", fontsize=8, color="#444")
        plt.subplots_adjust(bottom=0.28)
    ax.grid(axis="y", alpha=0.25)
    _save(fname)


# ----------------------------------------------------------------------
# fig01 — battery consumption comparison
# ----------------------------------------------------------------------
def fig01_battery_comparison(tier):
    modes = [m for m in ordered_modes(tier.keys())
             if _num(_get(tier[m], "energy.mean_consumed_pct")) is not None]
    if not modes:
        print("  (skip fig01: no energy)")
        return
    fig, ax = plt.subplots(figsize=(6.8, 4.6))
    for i, m in enumerate(modes):
        s = tier[m]
        mean = _get(s, "energy.mean_consumed_pct")
        ax.bar(i, mean, color=MODE_COLOR[m], edgecolor="white", zorder=2)
        per = _get(s, "energy.per_drone") or {}
        for d, e in per.items():
            c = _num(e.get("consumed_pct"))
            if c is not None:
                ax.scatter(i, c, s=28, color="#222", zorder=3)
        dur = _num(s.get("duration_sec")) or 0
        rate = 60.0 * mean / dur if dur else 0
        ax.text(i, mean + 1.2, f"{mean:.1f}%\n({rate:.1f} %/min)",
                ha="center", va="bottom", fontsize=9)
    ax.scatter([], [], s=28, color="#222", label="per-drone")
    ax.set_xticks(range(len(modes)))
    ax.set_xticklabels([MODE_LABEL[m] for m in modes])
    ax.set_ylabel("Battery consumed per drone (%)")
    ax.set_ylim(0, max(_get(tier[m], "energy.mean_consumed_pct") for m in modes) * 1.35)
    ax.set_title("Battery Consumption by Processing Tier (medium area)")
    ax.legend(loc="upper left", fontsize=8)
    ax.grid(axis="y", alpha=0.25)
    _save("fig01_battery_comparison.png")


# ----------------------------------------------------------------------
# fig02/03/04 — area scaling family
# ----------------------------------------------------------------------
def _area_points(area_runs):
    pts = []
    for s in area_runs:
        pts.append({
            "area": _get(s, "coverage.area_m2"),
            "dur": _num(s.get("duration_sec")),
            "cov": _num(s.get("coverage_overall_pct")),
            "energy": _num(_get(s, "energy.mean_consumed_pct")),
            "label": f"{s.get('scenario')}\n{_get(s, 'coverage.area_m2'):,.0f} m$^2$",
            "run": s.get("run_id"),
        })
    return pts


def fig02_search_area(area_runs):
    pts = _area_points(area_runs)
    pts = [p for p in pts if p["dur"]]
    if len(pts) < 2:
        print("  (skip fig02: need >=2 fog runs with area_m2)")
        return
    fig, ax = plt.subplots(figsize=(7.2, 4.6))
    xs = range(len(pts))
    bars = ax.bar(xs, [p["dur"] for p in pts], color="#2E7D32",
                  edgecolor="white", zorder=2)
    for b, p in zip(bars, pts):
        rate = p["area"] * (p["cov"] or 100) / 100.0 / p["dur"]
        ax.text(b.get_x() + b.get_width() / 2, b.get_height(),
                f"{p['dur']:.0f} s\n({rate:.0f} m$^2$/s)",
                ha="center", va="bottom", fontsize=9)
    ax.set_xticks(list(xs))
    ax.set_xticklabels([p["label"] for p in pts], fontsize=9)
    ax.set_ylabel("Mission duration (s)")
    ax.set_ylim(0, max(p["dur"] for p in pts) * 1.22)
    ax.set_title("Search-Area Scaling: Mission Duration per Area (fog, 3 drones)\n"
                 "labels: duration (effective covered-area rate)")
    ax.grid(axis="y", alpha=0.25)
    _save("fig02_search_area_comparison.png")


def fig03_coverage_vs_area(area_runs):
    pts = [p for p in _area_points(area_runs) if p["cov"] is not None]
    if len(pts) < 2:
        print("  (skip fig03: need >=2 fog runs with coverage + area)")
        return
    fig, ax = plt.subplots(figsize=(6.8, 4.6))
    xs = [p["area"] for p in pts]
    ys = [p["cov"] for p in pts]
    ax.plot(xs, ys, "-", color="#2E7D32", lw=1.6, zorder=2)
    ax.scatter(xs, ys, s=70, color="#2E7D32", edgecolor="white", zorder=3)
    for p in pts:
        ax.annotate(f"{p['cov']:.1f}%", (p["area"], p["cov"]),
                    textcoords="offset points", xytext=(0, 9),
                    ha="center", fontsize=9)
    ax.set_xlabel("Search area (m$^2$)")
    ax.set_ylabel("Coverage achieved (%)")
    ax.set_ylim(0, 105)
    ax.set_title("Coverage vs Search Area (fog, 3 drones)")
    ax.grid(alpha=0.3)
    _save("fig03_coverage_vs_area.png")


def fig04_battery_and_area(area_runs):
    pts = [p for p in _area_points(area_runs) if p["energy"] is not None]
    if len(pts) < 2:
        print("  (skip fig04: need >=2 fog runs with energy + area)")
        return
    xs = [p["area"] for p in pts]
    ys = [p["energy"] for p in pts]
    # least-squares line (no numpy dependency needed beyond basics)
    n = len(xs)
    mx, my = sum(xs) / n, sum(ys) / n
    denom = sum((x - mx) ** 2 for x in xs) or 1.0
    slope = sum((x - mx) * (y - my) for x, y in zip(xs, ys)) / denom
    icept = my - slope * mx
    fig, ax = plt.subplots(figsize=(6.8, 4.6))
    lx = [min(xs) * 0.95, max(xs) * 1.03]
    ax.plot(lx, [slope * x + icept for x in lx], "--", color="#999", lw=1.2,
            label=f"linear fit: {slope * 1000:.2f}% per 1000 m$^2$", zorder=1)
    ax.plot(xs, ys, "-", color="#2E7D32", lw=1.4, zorder=2)
    ax.scatter(xs, ys, s=70, color="#2E7D32", edgecolor="white", zorder=3)
    for p in pts:
        ax.annotate(f"{p['energy']:.1f}%", (p["area"], p["energy"]),
                    textcoords="offset points", xytext=(0, 9),
                    ha="center", fontsize=9)
    ax.set_xlabel("Search area (m$^2$)")
    ax.set_ylabel("Mean battery consumed per drone (%)")
    ax.set_title("Battery Consumption vs Search Area (fog, 3 drones)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.3)
    _save("fig04_battery_vs_area.png")


# ----------------------------------------------------------------------
# fig05 — drone failure recovery timeline
# ----------------------------------------------------------------------
def fig05_failure_timeline(fail, battery_files):
    if fail is None:
        print("  (skip fig05: no drone_down run)")
        return
    pair = pair_battery(fail, battery_files)
    if pair is None:
        print("  (skip fig05: no battery CSV for the failure run)")
        return
    _, df = pair
    per_end = {d: _num(e.get("end_pct"))
               for d, e in (_get(fail, "energy.per_drone") or {}).items()}
    failed = min(per_end, key=lambda d: per_end.get(d, 1e9)) if per_end else None
    # fault time = failed drone's last state transition (into its terminal state)
    fault_t = None
    if failed is not None:
        sub = df[df["drone"] == failed].reset_index(drop=True)
        if not sub.empty:
            changes = sub[sub["state"] != sub["state"].shift()]
            if len(changes):
                fault_t = float(changes["t_rel_sec"].iloc[-1])

    dur = _num(fail.get("duration_sec"))
    fig, ax = plt.subplots(figsize=(9.2, 5.0))
    for d in sorted(df["drone"].unique()):
        sub = df[df["drone"] == d].sort_values("t_rel_sec")
        col = DRONE_COLOR.get(d, "#888")
        if d == failed and fault_t is not None:
            ok = sub[sub["t_rel_sec"] <= fault_t]
            down = sub[sub["t_rel_sec"] >= fault_t]
            ax.plot(ok["t_rel_sec"], ok["battery_pct"], color=col, lw=2,
                    label=f"{d} (failed)")
            ax.plot(down["t_rel_sec"], down["battery_pct"], color=col, lw=1.4,
                    ls="--", alpha=0.55)
        else:
            ax.plot(sub["t_rel_sec"], sub["battery_pct"], color=col, lw=2,
                    label=f"{d} (survivor)" if d != failed else d)
    if fault_t is not None:
        ax.axvline(fault_t, color="#C44E52", ls=":", lw=2)
        ax.text(fault_t + 8, 96, f"{failed} failure injected\n t = {fault_t:.0f} s",
                fontsize=9, color="#C44E52", va="top")
        if dur:
            ax.axvspan(fault_t, dur, color="#2E7D32", alpha=0.06)
    if dur:
        ax.axvline(dur, color="#555", ls="--", lw=1.2)
        ax.text(dur - 8, 96, f"mission complete\n t = {dur:.0f} s",
                fontsize=9, color="#333", va="top", ha="right")
    covs = _get(fail, "coverage.per_drone") or {}
    overall = _num(fail.get("coverage_overall_pct"))
    lines = [f"final coverage: overall {overall:.1f}%"] if overall else []
    for d in sorted(covs):
        tag = " (failed)" if d == failed else ""
        lines.append(f"  {d}: {covs[d]:.1f}%{tag}")
    lines.append("fog watchdog reassigned the lost")
    lines.append("partition to the surviving drones")
    ax.text(0.985, 0.55, "\n".join(lines), transform=ax.transAxes, fontsize=8.5,
            va="top", ha="right",
            bbox=dict(boxstyle="round", fc="white", ec="#999", alpha=0.9))
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Battery (%)")
    ax.set_ylim(0, 102)
    ax.set_title("Drone-Failure Recovery Timeline (fog, medium area, drone_down fault)")
    ax.legend(loc="lower left", fontsize=9)
    ax.grid(alpha=0.25)
    _save("fig05_failure_recovery_timeline.png")


# ----------------------------------------------------------------------
# fig06 — resource utilisation in normal fog operation
# ----------------------------------------------------------------------
def fig06_fog_utilisation(util):
    if util is None:
        print("  (skip fig06: no fog run with `tiers` block)")
        return
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(10.4, 4.6))

    # (a) per-drone processing tasks by tier
    per = _get(util, "utilisation.per_drone") or {}
    drones = sorted(per.keys())
    bottoms = [0.0] * len(drones)
    for t in ("local", "fog", "cloud"):
        vals = [per[d].get(t, 0) or 0 for d in drones]
        ax1.bar(range(len(drones)), vals, bottom=bottoms,
                color=TIER_COLOR[t], label=f"{t} tasks", edgecolor="white")
        bottoms = [b + v for b, v in zip(bottoms, vals)]
    for i, d in enumerate(drones):
        ax1.text(i, bottoms[i], f"{int(bottoms[i])}", ha="center",
                 va="bottom", fontsize=9)
    ax1.set_xticks(range(len(drones)))
    ax1.set_xticklabels(drones)
    ax1.set_ylabel("Processing tasks logged")
    ax1.set_title("(a) Per-drone processing tasks by tier")
    ax1.legend(fontsize=8)
    ax1.grid(axis="y", alpha=0.25)

    # (b) tier pipeline activity (each tier's own unit of work)
    t = util.get("tiers", {})
    works = [(_get(t, "edge.frames", 0) or 0, "Edge\n(capture)", "frames", "#4C72B0"),
             (_get(t, "fog.inferences", 0) or 0, "Fog\n(detect+coord)", "inferences", "#2E7D32"),
             (_get(t, "cloud.archive_events", 0) or 0, "Cloud\n(archival)", "events", "#C44E52")]
    ys = [w[0] for w in works]
    bars = ax2.bar([w[1] for w in works], ys, color=[w[3] for w in works],
                   edgecolor="white")
    for b, (v, _, u, _c) in zip(bars, works):
        active = "ACTIVE" if v > 0 else "off"
        ax2.text(b.get_x() + b.get_width() / 2, v, f"{active}\n{v:,.0f} {u}",
                 ha="center", va="bottom", fontsize=9)
    ax2.set_ylabel("Work performed (own unit per tier)")
    ax2.set_ylim(0, max(ys) * 1.25 if max(ys) else 1)
    ax2.set_title("(b) Three-tier pipeline activity")
    ax2.grid(axis="y", alpha=0.25)

    fig.suptitle("Resource Utilisation — Normal Fog Operation "
                 f"({util.get('scenario')} area, run {util.get('run_id')})")
    plt.tight_layout()
    _save("fig06_fog_utilisation.png")


# ----------------------------------------------------------------------
# fig07 — battery vs time (tier comparison, clipped to mission duration)
# ----------------------------------------------------------------------
def fig07_battery_vs_time(tier, battery_files):
    fig, ax = plt.subplots(figsize=(8.4, 4.8))
    drew = False
    for m in ordered_modes(tier.keys()):
        s = tier[m]
        pair = pair_battery(s, battery_files)
        if pair is None:
            print(f"  (fig07: no battery CSV for {m} run — line skipped)")
            continue
        _, df = pair
        dur = _num(s.get("duration_sec")) or df["t_rel_sec"].max()
        sub = df[df["t_rel_sec"] <= dur].copy()
        if sub.empty:
            continue
        sub["bin"] = (sub["t_rel_sec"] // 5) * 5
        g = sub.groupby("bin")["battery_pct"]
        mean, lo, hi = g.mean(), g.min(), g.max()
        col = MODE_COLOR.get(m, "#888")
        ax.plot(mean.index, mean.values, color=col, lw=2.2,
                label=MODE_TITLE.get(m, m))
        ax.fill_between(mean.index, lo.values, hi.values, color=col, alpha=0.14)
        ax.scatter([mean.index[-1]], [mean.values[-1]], color=col, s=35, zorder=3)
        ax.annotate(f"{mean.values[-1]:.0f}% @ {mean.index[-1]:.0f}s",
                    (mean.index[-1], mean.values[-1]),
                    textcoords="offset points", xytext=(6, -4), fontsize=8,
                    color=col)
        drew = True
    if not drew:
        print("  (skip fig07: no usable battery CSVs)")
        plt.close()
        return
    ax.set_xlabel("Mission time (s)")
    ax.set_ylabel("Mean battery across swarm (%)")
    ax.set_ylim(0, 100)
    ax.set_title("Battery vs Time by Processing Tier (medium area; band = min-max "
                 "across drones,\nseries clipped at mission completion)")
    ax.legend(fontsize=9)
    ax.grid(alpha=0.25)
    _save("fig07_battery_vs_time.png")


# ----------------------------------------------------------------------
# fig08-11 — remaining tier comparisons
# ----------------------------------------------------------------------
def fig08_completion(tier):
    _tier_bar(tier, "completion_time_sec.mean", "completion_time_sec.stdev",
              "completion_time_sec.n",
              "Task completion time (s)",
              "Task Completion Time by Tier (assignment -> victim resolved)",
              "fig08_completion_comparison.png",
              note="Completion time is dominated by drone travel distance to the "
                   "victim, not by the processing tier;\nwith n<=4 events per run "
                   "the fog-cloud gap is within run-to-run variation.")


def fig09_coverage(tier):
    _tier_bar(tier, "coverage_overall_pct", None, None,
              "Area coverage (%)",
              "Area Coverage by Tier (medium area)",
              "fig09_coverage_comparison.png", ylim=(0, 110), pct=True,
              note="Near-identical coverage across tiers is expected: the search "
                   "planner is tier-independent,\nso detection latency — not "
                   "search competence — is what differentiates the tiers.")


def fig10_response(tier):
    _tier_bar(tier, "response_time_sec.mean", "response_time_sec.stdev",
              "response_time_sec.n",
              "Response time (s)",
              "Detection -> Command Response Time by Tier",
              "fig10_response_comparison.png")


def fig11_latency(tier):
    _tier_bar(tier, "latency_sec.mean", "latency_sec.stdev", "latency_sec.n",
              "Detection latency (s)",
              "Frame -> Confirmed-Detection Latency by Tier",
              "fig11_latency_comparison.png")
    lat = {m: _num(_get(s, "latency_sec.mean")) for m, s in tier.items()}
    if lat.get("fog"):
        for other in ("cloud", "local"):
            if lat.get(other):
                red = 100.0 * (lat[other] - lat["fog"]) / lat[other]
                print(f"  -> fog latency vs {other}: {red:+.1f}% "
                      f"({lat[other] / lat['fog']:.1f}x faster)")


# ----------------------------------------------------------------------
# tables
# ----------------------------------------------------------------------
def tier_table(tier):
    rows = {}
    for m in ordered_modes(tier.keys()):
        s = tier[m]
        dur = _num(s.get("duration_sec"))
        en = _num(_get(s, "energy.mean_consumed_pct"))
        rows[MODE_TITLE.get(m, m)] = {
            "source_file": s["_file"],
            "duration_s": dur,
            "detection_events": s.get("detection_events"),
            "latency_mean_s": _get(s, "latency_sec.mean"),
            "response_mean_s": _get(s, "response_time_sec.mean"),
            "completion_mean_s": _get(s, "completion_time_sec.mean"),
            "coverage_pct": s.get("coverage_overall_pct"),
            "energy_mean_pct": en,
            "energy_rate_pct_per_min": (60 * en / dur) if en and dur else None,
            "completion_ratio": s.get("completion_ratio"),
        }
    save_table(pd.DataFrame(rows).T, "table_tier_comparison")


def area_table(area_runs):
    rows = [{"file": s["_file"], "scenario": s.get("scenario"),
             "area_m2": _get(s, "coverage.area_m2"),
             "duration_s": s.get("duration_sec"),
             "coverage_pct": s.get("coverage_overall_pct"),
             "energy_mean_pct": _get(s, "energy.mean_consumed_pct")}
            for s in area_runs]
    if rows:
        save_table(pd.DataFrame(rows).set_index("file"), "table_area_scaling")


# ----------------------------------------------------------------------
def main():
    summaries = load_summaries()
    if not summaries:
        print("No *summary*.json found in", RESULTS_DIR)
        return
    battery_files = load_battery_files()
    print(f"Loaded {len(summaries)} summary file(s), "
          f"{len(battery_files)} battery file(s) from {RESULTS_DIR}")

    sel = select_runs(summaries)
    os.makedirs(TABLES_DIR, exist_ok=True)
    with open(os.path.join(TABLES_DIR, "run_selection_report.md"), "w") as f:
        f.write(sel["report"] + "\n")
    print("\n" + sel["report"])

    tier, area, util, fail = sel["tier"], sel["area"], sel["util"], sel["fail"]

    print("\nGenerating figures...")
    fig01_battery_comparison(tier)
    fig02_search_area(area)
    fig03_coverage_vs_area(area)
    fig04_battery_and_area(area)
    fig05_failure_timeline(fail, battery_files)
    fig06_fog_utilisation(util)
    fig07_battery_vs_time(tier, battery_files)
    fig08_completion(tier)
    fig09_coverage(tier)
    fig10_response(tier)
    fig11_latency(tier)

    tier_table(tier)
    area_table(area)
    print("\nDone. Plots in", PLOTS_DIR, "| tables in", TABLES_DIR)


if __name__ == "__main__":
    main()
