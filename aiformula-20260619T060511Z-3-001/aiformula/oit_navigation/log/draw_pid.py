import pandas as pd
import matplotlib.pyplot as plt

df = pd.read_csv("/home/aiformula/workspace/ros2_ws/src/aiformula/oit_navigation/log/pid_eval_20251007_215403.csv")

plt.figure()
plt.plot(df["time"], df["cmd_linear"], label="cmd_vel linear")
plt.plot(df["time"], df["odom_linear"], label="odom linear")
plt.xlabel("Time [s]")
plt.ylabel("Linear velocity [m/s]")
plt.legend()
plt.grid()
plt.show()

# 平均誤差など
print("平均線速度誤差:", abs(df["error_linear"]).mean())
print("平均角速度誤差:", abs(df["error_angular"]).mean())
