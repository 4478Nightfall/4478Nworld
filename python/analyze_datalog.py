"""Analyze robot datalog data and generate visualizations."""
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
from scipy import stats

# Load the data
fileName = 'datalog.csv'  # CSV is in parent directory when running from python/
try:
    df = pd.read_csv(fileName)
except FileNotFoundError:
    print(
        f"File '{fileName}' not found. Make sure it's in the same directory as this script.")
    exit(1)

print("Columns:", list(df.columns))
print("\nFirst rows:")
print(df.head())

# Birdseye view path plot using odometry
wheelDiameter = 3.25  # inches
trackWidth = 13.0    # inches


def rotToDist(rot, diameter):
    return np.pi * diameter * rot


leftStart = df['left_deg'].iloc[0]
leftEnd = df['left_deg'].iloc[-1]
rightStart = df['right_deg'].iloc[0]
rightEnd = df['right_deg'].iloc[-1]

leftRot = leftEnd - leftStart
rightRot = rightEnd - rightStart
avgRot = (leftRot + rightRot) / 2

print("\nLeft encoder start:", leftStart, "end:", leftEnd)
print("Right encoder start:", rightStart, "end:", rightEnd)
print("Total left rotations:", leftRot)
print("Total right rotations:", rightRot)
print("Average rotations:", avgRot)
print("wheelDiameter:", wheelDiameter)
print("Estimated distance traveled (inches):",
      np.pi * wheelDiameter * avgRot)

# 2D Odometry (for reference, using only left/right wheels)
x, y, theta = 0.0, 0.0, 0.0
positions2d = [(x, y)]

leftPrev = df['left_deg'].iloc[0]
rightPrev = df['right_deg'].iloc[0]

for i in range(1, len(df)):
    leftCurr = df['left_deg'].iloc[i]
    rightCurr = df['right_deg'].iloc[i]
    dLeft = rotToDist(leftCurr - leftPrev, wheelDiameter)
    dRight = rotToDist(rightCurr - rightPrev, wheelDiameter)
    leftPrev, rightPrev = leftCurr, rightCurr

    dCenter = (dLeft + dRight) / 2.0
    dTheta = (dRight - dLeft) / trackWidth

    theta += dTheta
    x += dCenter * np.cos(theta)
    y += dCenter * np.sin(theta)
    positions2d.append((x, y))

positions2d = np.array(positions2d)

print("X range:", positions2d[:, 0].min(), "to", positions2d[:, 0].max())
print("Y range:", positions2d[:, 1].min(), "to", positions2d[:, 1].max())

# Detect anomalies in 2D path (sudden jumps in position)
distances = np.sqrt(np.diff(positions2d[:, 0])**2 + np.diff(positions2d[:, 1])**2)
anomaly_threshold_path = np.mean(distances) + 3 * np.std(distances)
path_anomalies = np.where(distances > anomaly_threshold_path)[0]

plt.figure(figsize=(8, 8))
plt.plot(positions2d[:, 0], positions2d[:, 1], 'b-', label='Robot Path (2D)', alpha=0.7)
if len(path_anomalies) > 0:
    plt.scatter(positions2d[path_anomalies, 0], positions2d[path_anomalies, 1], 
                c='red', s=100, marker='X', label=f'Anomalies ({len(path_anomalies)})', zorder=5)
plt.xlabel('X Position (inches)')
plt.ylabel('Y Position (inches)')
plt.title('Robot Route (Birdseye View, 2D)')
plt.axis('equal')
plt.legend()
plt.grid(True, alpha=0.3)
plt.show(block=False)


# Plot motor temperatures over time
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(df['timestamp'], df['left_temp'], label='Left Motors Temp', alpha=0.7)
ax.plot(df['timestamp'], df['right_temp'], label='Right Motors Temp', alpha=0.7)

# Detect anomalies (temperatures above mean + 3 std)
temp_anomalies_left = np.where(df['left_temp'] > df['left_temp'].mean() + 3*df['left_temp'].std())[0]
temp_anomalies_right = np.where(df['right_temp'] > df['right_temp'].mean() + 3*df['right_temp'].std())[0]

if len(temp_anomalies_left) > 0:
    ax.scatter(df['timestamp'].iloc[temp_anomalies_left], df['left_temp'].iloc[temp_anomalies_left],
               c='red', s=100, marker='X', label=f'Left Anomalies ({len(temp_anomalies_left)})', zorder=5)
if len(temp_anomalies_right) > 0:
    ax.scatter(df['timestamp'].iloc[temp_anomalies_right], df['right_temp'].iloc[temp_anomalies_right],
               c='darkred', s=100, marker='X', label=f'Right Anomalies ({len(temp_anomalies_right)})', zorder=5)

ax.set_xlabel('Time (ms)')
ax.set_ylabel('Temperature (°C)')
ax.set_title('Motor Temperatures Over Time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show(block=False)

# Plot motor velocities over time
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(df['timestamp'], df['left_vel'], label='Left Motors Vel', alpha=0.7)
ax.plot(df['timestamp'], df['right_vel'], label='Right Motors Vel', alpha=0.7)

# Detect anomalies (Z-score > 3)
z_scores_left = np.abs(stats.zscore(df['left_vel']))
z_scores_right = np.abs(stats.zscore(df['right_vel']))
vel_anomalies_left = np.where(z_scores_left > 3)[0]
vel_anomalies_right = np.where(z_scores_right > 3)[0]

if len(vel_anomalies_left) > 0:
    ax.scatter(df['timestamp'].iloc[vel_anomalies_left], df['left_vel'].iloc[vel_anomalies_left],
               c='red', s=100, marker='X', label=f'Left Anomalies ({len(vel_anomalies_left)})', zorder=5)
if len(vel_anomalies_right) > 0:
    ax.scatter(df['timestamp'].iloc[vel_anomalies_right], df['right_vel'].iloc[vel_anomalies_right],
               c='darkred', s=100, marker='X', label=f'Right Anomalies ({len(vel_anomalies_right)})', zorder=5)

ax.set_xlabel('Time (ms)')
ax.set_ylabel('Velocity (RPM)')
ax.set_title('Motor Velocities Over Time (Averaged)')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show(block=False)

# Plot all 6 individual drive motor velocities overlaid
individual_motor_bases = ['mfl', 'mml', 'mbl', 'mfr', 'mmr', 'mbr']
if all(f'{m}_vel' in df.columns for m in individual_motor_bases):
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Plot all motor velocities
    colors = ['blue', 'cyan', 'teal', 'red', 'orange', 'brown']
    labels = ['Front Left (mfl)', 'Middle Left (mml)', 'Back Left (mbl)', 
              'Front Right (mfr)', 'Middle Right (mmr)', 'Back Right (mbr)']
    
    for idx, motor_base in enumerate(individual_motor_bases):
        motor_vel = df[f'{motor_base}_vel']
        ax.plot(df['timestamp'], motor_vel, label=labels[idx], 
                color=colors[idx], linewidth=2, alpha=0.8)
    
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Velocity (RPM)')
    ax.set_title('All Drive Motor Velocities (Individual Motors Overlaid)')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show(block=False)
else:
    print("\nWarning: Individual motor velocity columns not found. Available columns:", list(df.columns))

# Plot motor positions over time
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(df['timestamp'], df['left_deg'], label='Left Motors Deg', alpha=0.7)
ax.plot(df['timestamp'], df['right_deg'], label='Right Motors Deg', alpha=0.7)

# Detect anomalies (sudden changes in position)
pos_diff_left = np.abs(np.diff(df['left_deg']))
pos_diff_right = np.abs(np.diff(df['right_deg']))
pos_threshold_left = np.mean(pos_diff_left) + 3 * np.std(pos_diff_left)
pos_threshold_right = np.mean(pos_diff_right) + 3 * np.std(pos_diff_right)
pos_anomalies_left = np.where(pos_diff_left > pos_threshold_left)[0]
pos_anomalies_right = np.where(pos_diff_right > pos_threshold_right)[0]

if len(pos_anomalies_left) > 0:
    ax.scatter(df['timestamp'].iloc[pos_anomalies_left+1], df['left_deg'].iloc[pos_anomalies_left+1],
               c='red', s=100, marker='X', label=f'Left Anomalies ({len(pos_anomalies_left)})', zorder=5)
if len(pos_anomalies_right) > 0:
    ax.scatter(df['timestamp'].iloc[pos_anomalies_right+1], df['right_deg'].iloc[pos_anomalies_right+1],
               c='darkred', s=100, marker='X', label=f'Right Anomalies ({len(pos_anomalies_right)})', zorder=5)

ax.set_xlabel('Time (ms)')
ax.set_ylabel('Position (deg)')
ax.set_title('Motor Positions Over Time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show(block=False)

# Plot color sensor data over time - COMMENTED OUT (columns don't exist in current CSV)
# fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(10, 10))
# ax1.plot(df['timestamp'], df['colorHue'], 'r-', label='Hue')
# ax1.set_ylabel('Hue (0-360°)')
# ax1.set_title('Color Sensor Data Over Time')
# ax1.legend()
# ax1.grid(True, alpha=0.3)
# ax2.plot(df['timestamp'], df['colorSat'], 'g-', label='Saturation')
# ax2.set_ylabel('Saturation (0-1)')
# ax2.legend()
# ax2.grid(True, alpha=0.3)
# ax3.plot(df['timestamp'], df['colorProx'], 'b-', label='Proximity')
# ax3.set_xlabel('Time (ms)')
# ax3.set_ylabel('Proximity (0-255)')
# ax3.legend()
# ax3.grid(True, alpha=0.3)
# plt.tight_layout()
# plt.show(block=False)

# Plot all 6 chassis motor encoders together to detect deviations
individual_motor_bases = ['mfl', 'mml', 'mbl', 'mfr', 'mmr', 'mbr']
if all(f'{m}_deg' in df.columns for m in individual_motor_bases):
    fig, ax = plt.subplots(figsize=(14, 8))
    
    # Plot all motors
    colors = ['blue', 'cyan', 'teal', 'red', 'orange', 'brown']
    for idx, motor_base in enumerate(individual_motor_bases):
        motor_pos = df[f'{motor_base}_deg']
        ax.plot(df['timestamp'], motor_pos, label=motor_base, 
                color=colors[idx], linewidth=2, alpha=0.8)
    
    # Calculate average position to detect deviations
    avg_pos = df[[f'{m}_deg' for m in individual_motor_bases]].mean(axis=1)
    ax.plot(df['timestamp'], avg_pos, 'k--', label='Average', 
            linewidth=2, alpha=0.6, zorder=1)
    
    # Detect motors that deviate significantly from average
    max_deviations = []
    problem_motors = []
    
    for idx, motor_base in enumerate(individual_motor_bases):
        motor_pos = df[f'{motor_base}_deg']
        deviation = motor_pos - avg_pos
        max_deviation = deviation.abs().max()
        max_deviations.append((motor_base, max_deviation))
        
        # Highlight points where this motor deviates by more than 5 degrees from average
        significant_deviation = deviation.abs() > 5.0
        if significant_deviation.any():
            dev_indices = df['timestamp'][significant_deviation].index
            ax.scatter(df['timestamp'].iloc[dev_indices], motor_pos.iloc[dev_indices],
                      c='red', s=150, marker='X', alpha=0.6, zorder=10,
                      label=f'{motor_base} Deviations' if idx == 0 else '')
        
        # Mark motors that should be replaced (deviation >= 7 degrees)
        if max_deviation >= 7.0:
            problem_motors.append((motor_base, motor_pos))
            # Add label at the end of the motor's line
            final_pos = motor_pos.iloc[-1]
            final_time = df['timestamp'].iloc[-1]
            ax.annotate(f'⚠️ {motor_base}\n(REPLACE)', 
                       xy=(final_time, final_pos),
                       xytext=(10, 10), textcoords='offset points',
                       bbox=dict(boxstyle='round,pad=0.5', facecolor='red', alpha=0.7),
                       arrowprops=dict(arrowstyle='->', connectionstyle='arc3,rad=0'),
                       fontsize=9, fontweight='bold', color='white')
    
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Position (degrees)')
    ax.set_title('All Chassis Motor Encoders - Deviation Detection')
    ax.legend(loc='best')
    ax.grid(True, alpha=0.3)
    plt.tight_layout()
    plt.show(block=False)
    
    # Print deviation summary with replacement recommendations
    print("\n" + "="*80)
    print("MOTOR DEVIATION SUMMARY - REPLACEMENT RECOMMENDATIONS")
    print("="*80)
    max_deviations_sorted = sorted(max_deviations, key=lambda x: x[1], reverse=True)
    problem_motors = []
    
    for motor, max_dev in max_deviations_sorted:
        status = "✓ OK" if max_dev < 3.0 else "⚠ WARNING" if max_dev < 7.0 else "✗ REPLACE!"
        print(f"{motor:12s}: Max deviation from average: {max_dev:6.2f}° | {status}")
        if max_dev >= 7.0:
            problem_motors.append(motor)
    
    if problem_motors:
        print("\n" + "!"*80)
        print("⚠️  ACTION REQUIRED: Consider replacing these motor(s) for better auton accuracy:")
        for motor in problem_motors:
            print(f"   → {motor} (encoder deviating significantly)")
        print("!"*80)
    else:
        print("\n✓ All motors operating within acceptable range for accurate autons")

# Plot auton selector position
fig, ax = plt.subplots(figsize=(10, 6))
ax.plot(df['timestamp'], df['autonSelector_deg'],
        label='Auton Selector Position', alpha=0.7)

# Detect anomalies (sudden changes in position)
selector_diff = np.abs(np.diff(df['autonSelector_deg']))
selector_threshold = np.mean(selector_diff) + 3 * np.std(selector_diff)
selector_anomalies = np.where(selector_diff > selector_threshold)[0]

if len(selector_anomalies) > 0:
    ax.scatter(df['timestamp'].iloc[selector_anomalies+1], 
               df['autonSelector_deg'].iloc[selector_anomalies+1],
               c='red', s=100, marker='X', label=f'Anomalies ({len(selector_anomalies)})', zorder=5)

ax.set_xlabel('Time (ms)')
ax.set_ylabel('Position (deg)')
ax.set_title('Auton Selector Position Over Time')
ax.legend()
ax.grid(True, alpha=0.3)
plt.tight_layout()
plt.show()

# Print summary statistics
print("\nSummary statistics:")
print(df.describe())

print("\nnp.pi:", np.pi)
print("Total rotation:", df['left_deg'].iloc[-1] - df['left_deg'].iloc[0])
print("Computed distance:", np.pi * wheelDiameter *
      (df['left_deg'].iloc[-1] - df['left_deg'].iloc[0]))

# Analyze individual motor data if they exist (temp, vel, pos)
individual_motor_bases = ['mfl', 'mml', 'mbl', 'mfr', 'mmr', 'mbr']
individual_motors = [f'{m}_deg' for m in individual_motor_bases]

# Check if we have the new comprehensive motor data
has_detailed_motors = all(f'{m}_deg' in df.columns and f'{m}_temp' in df.columns and f'{m}_vel' in df.columns 
                          for m in individual_motor_bases)

if has_detailed_motors:
    print("\n" + "="*80)
    print("COMPREHENSIVE MOTOR ANALYSIS - Temperature, Velocity, Position")
    print("="*80)
    
    # Plot motor temperatures
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for idx, motor_base in enumerate(individual_motor_bases):
        ax = axes[idx]
        motor_temp = df[f'{motor_base}_temp']
        motor_vel = df[f'{motor_base}_vel']
        
        ax2 = ax.twinx()
        ax.plot(df['timestamp'], motor_temp, 'r-', label='Temp (°C)', alpha=0.7)
        ax2.plot(df['timestamp'], motor_vel, 'b-', label='Vel (RPM)', alpha=0.7)
        
        # Detect high temperature anomalies
        temp_anomalies = np.where(motor_temp > motor_temp.mean() + 3*motor_temp.std())[0]
        
        # Detect velocity anomalies
        vel_diff = np.abs(np.diff(motor_vel))
        vel_threshold = np.mean(vel_diff) + 3 * np.std(vel_diff)
        vel_anomalies = np.where(vel_diff > vel_threshold)[0]
        
        if len(temp_anomalies) > 0:
            ax.scatter(df['timestamp'].iloc[temp_anomalies], motor_temp.iloc[temp_anomalies],
                      c='red', s=100, marker='^', label=f'High Temp ({len(temp_anomalies)})', zorder=5)
        if len(vel_anomalies) > 0:
            ax2.scatter(df['timestamp'].iloc[vel_anomalies+1], motor_vel.iloc[vel_anomalies+1],
                       c='orange', s=100, marker='X', label=f'Vel Anom ({len(vel_anomalies)})', zorder=5)
        
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Temperature (°C)', color='r')
        ax2.set_ylabel('Velocity (RPM)', color='b')
        ax.set_title(f'{motor_base}: Temp & Velocity')
        ax.legend(loc='upper left')
        ax2.legend(loc='upper right')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    # Print motor temperature summary
    print("\nMotor Temperature Analysis:")
    print("-" * 80)
    for motor_base in individual_motor_bases:
        motor_temp = df[f'{motor_base}_temp']
        max_temp = motor_temp.max()
        avg_temp = motor_temp.mean()
        status = "✓ OK" if max_temp < 50 else "⚠ WARNING" if max_temp < 60 else "✗ HOT!"
        print(f"{motor_base:12s}: Max: {max_temp:5.1f}°C | Avg: {avg_temp:5.1f}°C | {status}")

elif all(col in df.columns for col in individual_motors):
    print("\n" + "="*80)
    print("INDIVIDUAL MOTOR ANALYSIS - Identifying problematic motors")
    print("="*80)
    
    # Plot all individual motor positions
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    axes = axes.flatten()
    
    for idx, motor in enumerate(individual_motors):
        ax = axes[idx]
        motor_data = df[motor]
        ax.plot(df['timestamp'], motor_data, label=motor, alpha=0.7)
        
        # Detect anomalies for this motor
        motor_diff = np.abs(np.diff(motor_data))
        motor_threshold = np.mean(motor_diff) + 3 * np.std(motor_diff)
        motor_anomalies = np.where(motor_diff > motor_threshold)[0]
        
        if len(motor_anomalies) > 0:
            ax.scatter(df['timestamp'].iloc[motor_anomalies+1], 
                      motor_data.iloc[motor_anomalies+1],
                      c='red', s=100, marker='X', label=f'Anomalies ({len(motor_anomalies)})', zorder=5)
        
        # Calculate motor synchronization error
        if idx < 3:  # Left motors
            avg_left = df['left_deg']
            sync_error = motor_data - avg_left
        else:  # Right motors
            avg_right = df['right_deg']
            sync_error = motor_data - avg_right
        
        max_sync_error = sync_error.max() - sync_error.min()
        ax.set_title(f'{motor}\nMax sync error: {max_sync_error:.2f} deg\nAnomalies: {len(motor_anomalies)}')
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Position (deg)')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    # Print summary of motor issues
    print("\nMotor Synchronization Issues:")
    print("-" * 80)
    for idx, motor in enumerate(individual_motors):
        if idx < 3:  # Left motors
            avg_target = df['left_deg']
        else:  # Right motors
            avg_target = df['right_deg']
        
        motor_data = df[motor]
        sync_error = motor_data - avg_target
        max_error = sync_error.abs().max()
        
        # Check for drift
        final_error = abs(sync_error.iloc[-1])
        drift_rate = sync_error.iloc[-1] / (df['timestamp'].iloc[-1] / 1000.0) if len(df) > 1 else 0
        
        status = "✓ OK" if max_error < 2.0 else "⚠ WARNING" if max_error < 5.0 else "✗ PROBLEM"
        print(f"{motor:12s}: Max error: {max_error:6.2f}° | Final drift: {final_error:6.2f}° | {status}")
    
    print("\n" + "="*80)
    print("Motor Anomaly Detection (sudden jumps > 3 std dev)")
    print("="*80)
    total_anomalies = 0
    for motor in individual_motors:
        motor_data = df[motor]
        motor_diff = np.abs(np.diff(motor_data))
        motor_threshold = np.mean(motor_diff) + 3 * np.std(motor_diff)
        motor_anomalies = np.where(motor_diff > motor_threshold)[0]
        print(f"{motor:12s}: {len(motor_anomalies):3d} anomalies detected")
        total_anomalies += len(motor_anomalies)
    
    if total_anomalies > 0:
        print(f"\n⚠️  WARNING: Total of {total_anomalies} anomalies detected across all motors!")
        print("This may indicate mechanical issues, encoder problems, or control instabilities.")
    else:
        print("\n✓ No motor anomalies detected - all motors operating normally.")

# Analyze IMU and horizontal tracker data
if 'imu_heading' in df.columns:
    print("\n" + "="*80)
    print("IMU & ODOMETRY ANALYSIS")
    print("="*80)
    
    fig, axes = plt.subplots(2, 2, figsize=(12, 10))
    
    # IMU Heading
    ax = axes[0, 0]
    ax.plot(df['timestamp'], df['imu_heading'], label='IMU Heading', alpha=0.7)
    ax.set_xlabel('Time (ms)')
    ax.set_ylabel('Heading (deg)')
    ax.set_title('IMU Heading Over Time')
    ax.legend()
    ax.grid(True, alpha=0.3)
    
    # Horizontal tracker
    if 'hTracker_deg' in df.columns:
        ax = axes[0, 1]
        ax.plot(df['timestamp'], df['hTracker_deg'], label='Horiz Tracker', alpha=0.7)
        
        # Detect anomalies
        htracker_diff = np.abs(np.diff(df['hTracker_deg']))
        htracker_threshold = np.mean(htracker_diff) + 3 * np.std(htracker_diff)
        htracker_anomalies = np.where(htracker_diff > htracker_threshold)[0]
        
        if len(htracker_anomalies) > 0:
            ax.scatter(df['timestamp'].iloc[htracker_anomalies+1], 
                      df['hTracker_deg'].iloc[htracker_anomalies+1],
                      c='red', s=100, marker='X', label=f'Anomalies ({len(htracker_anomalies)})', zorder=5)
        
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Position (deg)')
        ax.set_title('Horizontal Tracker')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # IMU Rotation, Pitch, Roll if available
    if all(col in df.columns for col in ['imu_rotation', 'imu_pitch', 'imu_roll']):
        ax = axes[1, 0]
        ax.plot(df['timestamp'], df['imu_rotation'], label='Rotation', alpha=0.7)
        ax.plot(df['timestamp'], df['imu_pitch'], label='Pitch', alpha=0.7)
        ax.plot(df['timestamp'], df['imu_roll'], label='Roll', alpha=0.7)
        ax.set_xlabel('Time (ms)')
        ax.set_ylabel('Angle (deg)')
        ax.set_title('IMU Rotation, Pitch, Roll')
        ax.legend()
        ax.grid(True, alpha=0.3)
    
    # Heading vs Position comparison
    if 'hTracker_deg' in df.columns:
        ax = axes[1, 1]
        # Calculate heading change from horizontal tracker
        # This assumes the tracker measures lateral displacement which should correlate with heading
        ax.scatter(df['imu_heading'], df['hTracker_deg'], alpha=0.5)
        ax.set_xlabel('IMU Heading (deg)')
        ax.set_ylabel('Horiz Tracker (deg)')
        ax.set_title('Heading Correlation')
        ax.grid(True, alpha=0.3)
    
    plt.tight_layout()
    plt.show(block=False)
    
    # Print IMU analysis
    print("\nIMU Analysis:")
    print("-" * 80)
    heading_change = df['imu_heading'].iloc[-1] - df['imu_heading'].iloc[0]
    print(f"Total heading change: {heading_change:.2f}°")
    
    if 'imu_rotation' in df.columns:
        rotation_change = df['imu_rotation'].iloc[-1] - df['imu_rotation'].iloc[0]
        print(f"Total unbounded rotation: {rotation_change:.2f}°")
        max_pitch = df['imu_pitch'].abs().max()
        max_roll = df['imu_roll'].abs().max()
        print(f"Max pitch deviation: {max_pitch:.2f}°")
        print(f"Max roll deviation: {max_roll:.2f}°")
        if max_pitch > 10 or max_roll > 10:
            print("⚠️  WARNING: Large pitch/roll detected - robot may be tilting!")
    
    if 'hTracker_deg' in df.columns:
        htracker_change = df['hTracker_deg'].iloc[-1] - df['hTracker_deg'].iloc[0]
        print(f"Horizontal tracker movement: {htracker_change:.2f}°")
        htracker_range = df['hTracker_deg'].max() - df['hTracker_deg'].min()
        print(f"Horizontal tracker range: {htracker_range:.2f}°")
