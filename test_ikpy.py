import ikpy.chain
import numpy as np
import matplotlib.pyplot as plt
import imageio
# Suppress warnings
import warnings
warnings.filterwarnings('ignore')

# 1. Load the robot arm from URDF
urdf_file = "src/hexapod_arm_bot_description/urdf/hexapod.urdf.xacro"
# Since ikpy doesn't parse xacro, we might need a raw URDF.
# Let's write a small script that converts first.

import subprocess
import tempfile

def create_raw_urdf(xacro_path):
    print(f"Translating xacro to urdf from: {xacro_path}")
    raw_urdf_path = tempfile.mktemp(suffix=".urdf")
    cmd = f"bash -c 'source /opt/ros/humble/setup.bash && source install/setup.bash && xacro {xacro_path} > {raw_urdf_path}'"
    result = subprocess.run(cmd, shell=True, capture_output=True)
    if result.returncode != 0:
        print("Error compiling xacro:")
        print(result.stderr.decode("utf-8"))
        exit(1)
    return raw_urdf_path

raw_urdf = create_raw_urdf(urdf_file)

# We will focus on one arm, say 'arm_L1_cyl6' (the end of the first left arm)
# ikpy chain needs a base_elements and an active_links mask.
base_element = "base_footprint"
end_effector = "arm_L1_cyl6"

print(f"Loading kinematic chain from {base_element} to {end_effector}...")
try:
    arm_chain = ikpy.chain.Chain.from_urdf_file(raw_urdf, base_elements=[base_element])
except Exception as e:
    print(f"Error loading chain: {e}")
    # Let's try parsing specifically for the joints we want by finding a path
    # Actually ikpy's from_urdf_file extracts the first valid chain it finds.
    # To specify the end effector (often necessary for complex robots):
    print("Falling back to standard parsing without explicit base if needed, or check link names.")
    import os
    os.remove(raw_urdf)
    exit(1)

# Clean up temp file
#os.remove(raw_urdf) 

print("Chain loaded successfully!")
print(f"Number of joints: {len(arm_chain.links)}")
for i, link in enumerate(arm_chain.links):
    print(f"  {i}: {link.name}")

# 2. Define Initial State
# The robot has several joints per arm. 
# arm_chain.links contains base + joints.
initial_angles = [0.0] * len(arm_chain.links)
print("\nInitial State (Joint Angles):")
print([round(j, 3) for j in initial_angles[1:-1]]) # Exclude base/tip static links

# 3. Calculate Initial End-Effector Position (Forward Kinematics)
target_frame_initial = arm_chain.forward_kinematics(initial_angles)
initial_position = target_frame_initial[:3, 3]
print("\nInitial End-Effector Position (x, y, z):")
print([round(p, 3) for p in initial_position])

# 4. Generate a Random Target State
import random
print("\nGenerating a random valid target state...")
# We perturb the initial angles lightly
target_angles_real = []
for i in range(len(arm_chain.links)):
    # Add random noise between -1.2 and 1.2 rad to active joints
    noise = random.uniform(-1.2, 1.2) if i not in [0, len(arm_chain.links)-1] else 0.0
    target_angles_real.append(initial_angles[i] + noise)

target_frame = arm_chain.forward_kinematics(target_angles_real)
target_position = target_frame[:3, 3]
print("Random Target End-Effector Position (x, y, z):")
print([round(p, 3) for p in target_position])

# 5. Solve Inverse Kinematics
print("\nSolving IK for the target position using initial state as seed...")
# ik_solve returns the joint angles
ik_angles = arm_chain.inverse_kinematics(target_position, initial_position=initial_angles)

print("\nIK Solution (Joint Angles):")
print([round(j, 3) for j in ik_angles[1:-1]])

# Verify the solution via FK
solved_frame = arm_chain.forward_kinematics(ik_angles)
solved_position = solved_frame[:3, 3]
solved_rotation = solved_frame[:3, :3]

# Compute Quaternion using scipy (ikpy operates on matrices)
from scipy.spatial.transform import Rotation as R
r = R.from_matrix(solved_rotation)
solved_quaternion = r.as_quat() # returns [x, y, z, w]

error = np.linalg.norm(solved_position - target_position)
print(f"Distance Error to target: {error:.5f} meters")

# 6. Generate GIF Animation and Process Per-Step Results
print("\nGenerating GIF animation and per-step text results...")
num_steps = 20
gif_frames = []
temp_images = []

step_results = []

for step in range(num_steps + 1):
    # Interpolate between initial angles and ik_angles
    t = step / num_steps
    current_angles = [initial_angles[i] * (1-t) + ik_angles[i] * t for i in range(len(ik_angles))]
    
    # Calculate FK for the current interpolated step
    current_frame = arm_chain.forward_kinematics(current_angles)
    curr_pos = current_frame[:3, 3]
    curr_rot = current_frame[:3, :3]
    
    # Quaternion
    curr_r = R.from_matrix(curr_rot)
    curr_quat = curr_r.as_quat()
    
    step_results.append({
        'step': step,
        'position': curr_pos,
        'rotation': curr_rot,
        'quaternion': curr_quat,
        'angles': current_angles
    })
    
    # Create two subplots: Top View (X-Y) and Side View (X-Z)
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(16, 8))
    joints = arm_chain.forward_kinematics(current_angles, full_kinematics=True)
    xs = [j[0, 3] for j in joints]
    ys = [j[1, 3] for j in joints]
    zs = [j[2, 3] for j in joints]
    
    # --- Top View (X-Y) ---
    body_top = plt.Circle((0, 0), 0.2, color='gray', alpha=0.3, label='Body (Top)')
    ax1.add_patch(body_top)
    ax1.plot(xs, ys, "-o", linewidth=3, markersize=8, color='blue', label='Arm L1')
    ax1.plot(xs[0], ys[0], "ks", markersize=10)
    ax1.plot(target_position[0], target_position[1], "r*", markersize=15, label='Target')
    ax1.set_title(f"Top View (X-Y)\nStep {step}/{num_steps}")
    ax1.set_xlabel("X (meters)")
    ax1.set_ylabel("Y (meters)")
    ax1.set_xlim(-2.5, 2.5)
    ax1.set_ylim(-2.5, 2.5)
    ax1.set_aspect('equal')
    ax1.grid(True, linestyle='--', alpha=0.5)
    ax1.legend(loc='upper right')

    # --- Side View (X-Z) ---
    # Body in side view (cylinder side)
    body_side = plt.Rectangle((-0.5, 0.5), 1.0, 0.4, color='gray', alpha=0.3, label='Body (Side)')
    ax2.add_patch(body_side)
    ax2.plot(xs, zs, "-o", linewidth=3, markersize=8, color='green', label='Arm L1')
    ax2.plot(xs[0], zs[0], "ks", markersize=10)
    ax2.plot(target_position[0], target_position[2], "r*", markersize=15, label='Target')
    ax2.set_title(f"Side View (X-Z)\nStep {step}/{num_steps}")
    ax2.set_xlabel("X (meters)")
    ax2.set_ylabel("Z (meters)")
    ax2.set_xlim(-2.5, 2.5)
    ax2.set_ylim(-1.5, 3.5) # Z range might be different
    ax2.set_aspect('equal')
    ax2.grid(True, linestyle='--', alpha=0.5)
    ax2.legend(loc='upper right')
    
    # --- Data Overlay (Matrix/Quat) ---
    rot_str = (f"Rotation Matrix:\n"
               f"[{curr_rot[0,0]:.2f}, {curr_rot[0,1]:.2f}, {curr_rot[0,2]:.2f}]\n"
               f"[{curr_rot[1,0]:.2f}, {curr_rot[1,1]:.2f}, {curr_rot[1,2]:.2f}]\n"
               f"[{curr_rot[2,0]:.2f}, {curr_rot[2,1]:.2f}, {curr_rot[2,2]:.2f}]")
    quat_str = f"Quat: [{curr_quat[0]:.2f}, {curr_quat[1]:.2f}, {curr_quat[2]:.2f}, {curr_quat[3]:.2f}]"
    init_pos_str = f"Initial Pos: [{initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f}]"
    targ_pos_str = f"Target Pos: [{target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}]"

    # Overlay everything in the middle/background or a common area
    plt.figtext(0.5, 0.02, f"{rot_str}    {quat_str}\n{init_pos_str}    {targ_pos_str}", 
                ha='center', fontsize=9, bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))

    plt.tight_layout(rect=[0, 0.1, 1, 1]) # Make room for the text at bottom
    
    img_path = f"temp_frame_{step}.png"
    plt.savefig(img_path)
    plt.close(fig)
    
    temp_images.append(img_path)

# 5.5 Write Results to File
ik_result_file = "ik_results.txt"
with open(ik_result_file, "w") as f:
    f.write("=== Inverse Kinematics Results ===\n\n")
    f.write("Initial Position (x, y, z):\n")
    f.write(f"[{initial_position[0]:.5f}, {initial_position[1]:.5f}, {initial_position[2]:.5f}]\n\n")
    f.write("Target Position (x, y, z):\n")
    f.write(f"[{target_position[0]:.5f}, {target_position[1]:.5f}, {target_position[2]:.5f}]\n\n")
    f.write("Final Solved Position (x, y, z):\n")
    f.write(f"[{solved_position[0]:.5f}, {solved_position[1]:.5f}, {solved_position[2]:.5f}]\n\n")
    f.write(f"Distance Error to Target: {error:.5f} meters\n\n")
    
    f.write("=== Per-Step Trajectory ===\n\n")
    for res in step_results:
        f.write(f"--- Step {res['step']}/{num_steps} ---\n")
        f.write(f"Position: [{res['position'][0]:.5f}, {res['position'][1]:.5f}, {res['position'][2]:.5f}]\n")
        f.write(f"Quaternion [x, y, z, w]: [{res['quaternion'][0]:.5f}, {res['quaternion'][1]:.5f}, {res['quaternion'][2]:.5f}, {res['quaternion'][3]:.5f}]\n")
        f.write("Rotation Matrix:\n")
        for row in res['rotation']:
            f.write(f"  [{row[0]:.5f}, {row[1]:.5f}, {row[2]:.5f}]\n")
        f.write("\n")

output_gif = "ik_animation.gif"
with imageio.get_writer(output_gif, mode='I', duration=0.5) as writer:
    for img_path in temp_images:
        image = imageio.imread(img_path)
        writer.append_data(image)

print(f"Animation saved to {output_gif}")

# Cleanup
import os
for img_path in temp_images:
    os.remove(img_path)
if os.path.exists(raw_urdf):
    os.remove(raw_urdf)
print("Done!")
