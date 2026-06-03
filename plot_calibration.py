# plot_calibration.py

import pandas as pd
import matplotlib.pyplot as plt
import glob
import os

# Prend le CSV de calibration le plus récent
files = glob.glob(os.path.expanduser("~/Desktop/calibration_*.csv"))
if not files:
    print("No calibration CSV found on Desktop")
    exit()

csv_file = max(files, key=os.path.getmtime)
print(f"Loading: {csv_file}")

df = pd.read_csv(csv_file)

fig, axes = plt.subplots(3, 1, figsize=(10, 10))
fig.suptitle("WASP Calibration Analysis", fontsize=14, fontweight='bold')

# RPM moteur vs RPM spool
axes[0].plot(df['target_rpm'], df['motor_rpm'], 'bo-', label='Motor RPM (measured)')
axes[0].plot(df['target_rpm'], df['spool_rpm'], 'go-', label='Spool RPM (Hall)')
axes[0].plot(df['target_rpm'], df['target_rpm'] * 2.57, 'r--', label='Spool RPM (theoretical ×2.57)')
axes[0].set_xlabel("Target RPM")
axes[0].set_ylabel("RPM")
axes[0].legend()
axes[0].grid(True, alpha=0.3)
axes[0].set_title("Motor vs Spool RPM")

# Ratio mesuré
axes[1].plot(df['target_rpm'], df['ratio'], 'mo-', label='Measured ratio')
axes[1].axhline(y=2.57, color='red', linestyle='--', label='Theoretical (2.57)')
avg_ratio = df['ratio'][df['ratio'] > 0].mean()
axes[1].axhline(y=avg_ratio, color='green', linestyle='--', label=f'Average ({avg_ratio:.3f})')
axes[1].set_xlabel("Target RPM")
axes[1].set_ylabel("Gear ratio")
axes[1].legend()
axes[1].grid(True, alpha=0.3)
axes[1].set_title("Gear Ratio Measured vs Theoretical")

# Erreur relative
df['error_pct'] = (df['ratio'] - 2.57) / 2.57 * 100
axes[2].bar(df['target_rpm'], df['error_pct'], color='coral')
axes[2].axhline(y=0, color='black', linewidth=0.8)
axes[2].set_xlabel("Target RPM")
axes[2].set_ylabel("Error (%)")
axes[2].grid(True, alpha=0.3)
axes[2].set_title("Error vs Theoretical Ratio (2.57)")

plt.tight_layout()
plt.savefig(os.path.expanduser("~/Desktop/calibration_analysis.png"), dpi=150, bbox_inches='tight')
print(f"\nSuggested GEAR_RATIO_FAST = {avg_ratio:.3f}")
plt.show()