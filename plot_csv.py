# plot_csv.py

import pandas as pd
import matplotlib.pyplot as plt

csv_file = "/Users/Erol/Desktop/levee_charg1Me.csv"

df = pd.read_csv(csv_file)
df['t'] = df['timestamp'] - df['timestamp'].iloc[0]

fig, axes = plt.subplots(6, 1, figsize=(12, 16), sharex=True)
fig.suptitle("WASP — Levée de charge", fontsize=14, fontweight='bold')

# Cable length + z_ref
axes[0].plot(df['t'], df['cable_length'], color='crimson',  linewidth=1.5, label='cable_length')
axes[0].plot(df['t'], df['z_ref'],        color='black',    linewidth=1.2, linestyle='--', label='z_ref')
axes[0].set_ylabel("Length (m)", fontsize=9)
axes[0].legend(fontsize=8, loc='upper right')

# Motor RPM
axes[1].plot(df['t'], df['motor_rpm'], color='steelblue', linewidth=1.5)
axes[1].set_ylabel("Motor RPM", fontsize=9)

# Spool RPM
axes[2].scatter(df['t'], df['spool_rpm'], color='seagreen', s=2)
axes[2].set_ylabel("Spool RPM", fontsize=9)

# Total turns motor
axes[3].plot(df['t'], df['total_turns_motor'], color='darkorange', linewidth=1.5)
axes[3].set_ylabel("Turns motor", fontsize=9)

# Total turns spool
axes[4].plot(df['t'], df['total_turns_spool'], color='purple', linewidth=1.5)
axes[4].set_ylabel("Turns spool", fontsize=9)

# Current
axes[5].plot(df['t'], df['current'], color='goldenrod', linewidth=1.5)
axes[5].set_ylabel("Current (A)", fontsize=9)

for ax in axes:
    ax.grid(True, alpha=0.3)
    ax.spines['top'].set_visible(False)
    ax.spines['right'].set_visible(False)
    ax.tick_params(labelbottom=True)
    ax.set_xlabel("Time (s)", fontsize=8)
    if 'switch_active' in df.columns:
        switch_changes = df[df['switch_active'] != df['switch_active'].shift()]
        for _, row in switch_changes.iterrows():
            ax.axvline(x=row['t'], color='red', linewidth=1.2, linestyle='--', alpha=0.7)

plt.tight_layout()
plt.savefig("/Users/Erol/Desktop/wasp_analysis.png", dpi=150, bbox_inches='tight')
plt.show()