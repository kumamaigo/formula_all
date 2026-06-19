#!/usr/bin/env python3

import pandas as pd
import matplotlib.pyplot as plt

# CSVファイル読み込み
df = pd.read_csv("sub_odom.csv")

# 列名の確認（必要ならprintで表示）
# print(df.columns)

plt.figure(figsize=(7,7))

# 各軌跡を色分けしてプロット
#plt.plot(df["gyro_y"],   df["gyro_x"],   label="Gyro",   color="red")
#plt.plot(df["sub_y"],    df["sub_x"],    label="Sub",    color="blue")
#plt.plot(df["fusion_y"], df["fusion_x"], label="Fusion", color="green")
plt.plot(df["y"], df["x"], label="Fusion", color="red")

# 軸ラベル・タイトル
plt.ylabel("X [m]")
plt.xlabel("Y [m]")
plt.title("Trajectory Comparison")

# スケールを揃える＆グリッド表示
plt.axis("equal")
plt.grid(True)
plt.legend()
plt.tight_layout()
plt.show()

