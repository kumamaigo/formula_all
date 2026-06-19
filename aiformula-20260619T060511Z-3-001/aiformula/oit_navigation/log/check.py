#!/usr/bin/env python3

import pandas as pd
import numpy as np

def compute_metrics(x, y, z):
    coords = np.vstack([x, y, z]).T
    diffs = np.diff(coords, axis=0)
    dists = np.linalg.norm(diffs, axis=1)
    total_distance = np.sum(dists)
    displacement = np.linalg.norm(coords[-1] - coords[0])
    drift_ratio = (displacement / total_distance * 100) if total_distance > 0 else np.nan
    return total_distance, displacement, drift_ratio

def analyze_odometry_all():
    all_results = []
    prefixes = ["gyro", "sub", "fusion"]

    for run in range(1, 11):  # 1～10回分の走行
        df = pd.read_csv(f"multi_odom_{run}.csv")
        for prefix in prefixes:
            total, disp, ratio = compute_metrics(df[f"{prefix}_x"], df[f"{prefix}_y"], df[f"{prefix}_z"])
            all_results.append({
                "Run": run,
                "Topic": prefix,
                "Total distance [m]": total,
                "Net displacement [m]": disp,
                "Drift ratio [%]": ratio
            })

    results_df = pd.DataFrame(all_results)

    # 全走行まとめ
    summary = []
    for prefix in prefixes:
        df_topic = results_df[results_df["Topic"] == prefix]
        total_dist = df_topic["Total distance [m]"].sum()
        mean_disp = df_topic["Net displacement [m]"].mean()
        mean_ratio = df_topic["Drift ratio [%]"].mean()
        summary.append({
            "Topic": prefix,
            "Total distance (all runs) [m]": total_dist,
            "Mean displacement [m]": mean_disp,
            "Mean drift ratio [%]": mean_ratio
        })
    summary_df = pd.DataFrame(summary)

    print("===== Each Run Results =====")
    print(results_df.round(3))
    print("\n===== Summary Across All Runs =====")
    print(summary_df.round(3))

    results_df.to_csv("odom_eval_each_run.csv", index=False)
    summary_df.to_csv("odom_eval_summary.csv", index=False)

    return results_df, summary_df

if __name__ == "__main__":
    analyze_odometry_all()
