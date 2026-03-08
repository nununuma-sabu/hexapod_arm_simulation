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
    
    # --- Integrated 3D View (Manual Projection) ---
    def project_3d(x, y, z):
        # Simple Isometric-like projection
        # u: horizontal axis, v: vertical axis
        u = (y - x) * np.cos(np.radians(30))
        v = z - (x + y) * np.sin(np.radians(30))
        return u, v

    # Calculate all joint positions
    joints = arm_chain.forward_kinematics(current_angles, full_kinematics=True)
    all_xs = [j[0, 3] for j in joints]
    all_ys = [j[1, 3] for j in joints]
    all_zs = [j[2, 3] for j in joints]

    # --- Identify the Arm segments for plotting ---
    arm_start_idx = 0
    body_idx = 0
    for i, link in enumerate(arm_chain.links):
        if "body_link" in link.name:
            body_idx = i
        if "cyl1" in link.name:
            arm_start_idx = i
            break
            
    body_pos = joints[body_idx][:3, 3]
    xs = all_xs[arm_start_idx:]
    ys = all_ys[arm_start_idx:]
    zs = all_zs[arm_start_idx:]
    
    # Other arm base positions and rotations (based on URDF)
    # Each arm starts by pointing outwards from the torso.
    # L1: front-left, L2: mid-left, L3: back-left...
    arms_config = [
        {"name": "arm_L1", "xyz": [0.0, 0.2, -0.3], "is_active": True},
        {"name": "arm_L2", "xyz": [0.0, 0.2, 0.0],  "is_active": False},
        {"name": "arm_L3", "xyz": [0.0, 0.2, 0.3],  "is_active": False},
        {"name": "arm_R1", "xyz": [0.0, -0.2, -0.3], "is_active": False},
        {"name": "arm_R2", "xyz": [0.0, -0.2, 0.0],  "is_active": False},
        {"name": "arm_R3", "xyz": [0.0, -0.2, 0.3],  "is_active": False},
    ]
    
    # Create three subplots: Top, Side, and Integrated 3D
    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(18, 7))
    
    def plot_robot_scene(ax, view_type='3d'):
        # World Axes (RGB: X=Red, Y=Green, Z=Blue)
        scale = 0.4
        if view_type == 'top':
            ax.quiver(0, 0, scale, 0, color='red', angles='xy', scale_units='xy', scale=1, alpha=0.5) # X
            ax.quiver(0, 0, 0, scale, color='green', angles='xy', scale_units='xy', scale=1, alpha=0.5) # Y
        elif view_type == 'side':
            ax.quiver(0, 0, scale, 0, color='red', angles='xy', scale_units='xy', scale=1, alpha=0.5) # X
            ax.quiver(0, 0, 0, scale, color='blue', angles='xy', scale_units='xy', scale=1, alpha=0.5) # Z
        elif view_type == '3d_iso':
            ux, vx = project_3d(scale, 0, 0); uy, vy = project_3d(0, scale, 0); uz, vz = project_3d(0, 0, scale)
            ax.plot([0, ux], [0, vx], color='red', linewidth=3, alpha=0.6)
            ax.plot([0, uy], [0, vy], color='green', linewidth=3, alpha=0.6)
            ax.plot([0, uz], [0, vz], color='blue', linewidth=3, alpha=0.6)

        # Draw Body (Grey Cylinder)
        if view_type == 'top':
            rect = plt.Rectangle((body_pos[0]-0.5, body_pos[1]-0.2), 1.0, 0.4, color='gray', alpha=0.7)
            ax.add_patch(rect)
        elif view_type == 'side':
            rect = plt.Rectangle((body_pos[0]-0.5, body_pos[2]-0.2), 1.0, 0.4, color='gray', alpha=0.7)
            ax.add_patch(rect)
        elif view_type == '3d_iso':
            for bx in [body_pos[0]-0.5, body_pos[0]+0.5]:
                phis = np.linspace(0, 2*np.pi, 20)
                ub, vb = zip(*[project_3d(bx, body_pos[1]+0.2*np.cos(p), body_pos[2]+0.2*np.sin(p)) for p in phis])
                ax.plot(ub, vb, color='gray', alpha=0.4, linewidth=1.5)
            for ang in [0, np.pi/2, np.pi, 3*np.pi/2]:
                ub1, vb1 = project_3d(body_pos[0]-0.5, body_pos[1]+0.2*np.cos(ang), body_pos[2]+0.2*np.sin(ang))
                ub2, vb2 = project_3d(body_pos[0]+0.5, body_pos[1]+0.2*np.cos(ang), body_pos[2]+0.2*np.sin(ang))
                ax.plot([ub1, ub2], [vb1, vb2], color='gray', alpha=0.4, linewidth=1.5)

        # Plot All 6 Arms
        for arm in arms_config:
            if arm['is_active']:
                # Active arm L1 (using current FK results)
                la_xs, la_ys, la_zs = xs, ys, zs
                alpha = 0.9
            else:
                # Other arms: draw as simple straight lines from their base to show hexapod shape
                # base_joint orientation in URDF is +/- pi/2 around X
                side = 1.0 if "L" in arm['name'] else -1.0
                full_len = 1.5
                b_x, b_y, b_z = body_pos[0]+arm['xyz'][0], body_pos[1]+arm['xyz'][1], body_pos[2]+arm['xyz'][2]
                la_xs = [b_x, b_x]; la_ys = [b_y, b_y + side*full_len]; la_zs = [b_z, b_z]
                alpha = 0.25 # Faded for inactive arms

            if view_type == 'top':
                ax.plot(la_xs, la_ys, color='red', linewidth=3, alpha=alpha)
                ax.plot(la_xs, la_ys, "bo", markersize=3, alpha=alpha)
                ax.plot(la_xs[0], la_ys[0], "ks", markersize=5, alpha=alpha)
            elif view_type == 'side':
                ax.plot(la_xs, la_zs, color='red', linewidth=3, alpha=alpha)
                ax.plot(la_xs, la_zs, "bo", markersize=3, alpha=alpha)
                ax.plot(la_xs[0], la_zs[0], "ks", markersize=5, alpha=alpha)
            elif view_type == '3d_iso':
                ua, va = zip(*[project_3d(x,y,z) for x,y,z in zip(la_xs, la_ys, la_zs)])
                ax.plot(ua, va, color='red', linewidth=3, alpha=alpha)
                ax.plot(ua, va, "bo", markersize=4, alpha=alpha)
                ub, vb = project_3d(la_xs[0], la_ys[0], la_zs[0])
                ax.plot(ub, vb, "ks", markersize=6, alpha=alpha)

    # Render Scenes
    plot_robot_scene(ax1, 'top')
    ax1.plot(target_position[0], target_position[1], "r*", markersize=14, label='Target')
    ax1.set_title("Top View (X-Y)")
    ax1.set_xlim(-2.0, 2.0); ax1.set_ylim(-2.0, 2.0); ax1.set_aspect('equal'); ax1.grid(True, alpha=0.2)

    plot_robot_scene(ax2, 'side')
    ax2.axhline(0, color='brown', linewidth=1, alpha=0.3)
    ax2.plot(target_position[0], target_position[2], "r*", markersize=14)
    ax2.set_title("Side View (X-Z)")
    ax2.set_xlim(-2.0, 2.0); ax2.set_ylim(-0.2, 2.0); ax2.set_aspect('equal'); ax2.grid(True, alpha=0.2)

    plot_robot_scene(ax3, '3d_iso')
    ut, vt = project_3d(target_position[0], target_position[1], target_position[2])
    ax3.plot(ut, vt, "r*", markersize=16)
    ax3.set_title(f"Integrated 3D (Isometric)\nStep {step}/{num_steps}")
    ax3.set_xlim(-3.0, 3.0); ax3.set_ylim(-3.0, 3.0); ax3.set_aspect('equal'); ax3.axis('off')

    # --- Data Overlay (Matrix/Quat) ---

    # --- Data Overlay (Matrix/Quat) ---
    rot_str = (f"Rotation Matrix:\n"
               f"[{curr_rot[0,0]:.2f}, {curr_rot[0,1]:.2f}, {curr_rot[0,2]:.2f}]\n"
               f"[{curr_rot[1,0]:.2f}, {curr_rot[1,1]:.2f}, {curr_rot[1,2]:.2f}]\n"
               f"[{curr_rot[2,0]:.2f}, {curr_rot[2,1]:.2f}, {curr_rot[2,2]:.2f}]")
    quat_str = f"Quat: [{curr_quat[0]:.2f}, {curr_quat[1]:.2f}, {curr_quat[2]:.2f}, {curr_quat[3]:.2f}]"
    pos_info = f"Init Pos: [{initial_position[0]:.2f}, {initial_position[1]:.2f}, {initial_position[2]:.2f}] | Targ Pos: [{target_position[0]:.2f}, {target_position[1]:.2f}, {target_position[2]:.2f}]"

    plt.figtext(0.5, 0.02, f"{rot_str}    {quat_str}\n{pos_info}", 
                ha='center', fontsize=10, bbox=dict(boxstyle='round', facecolor='white', alpha=0.9))
    
    # Disclaimer
    plt.figtext(0.98, 0.02, "*Illustrative purposes only", fontsize=9, ha='right', va='bottom', alpha=0.7)

    plt.tight_layout(rect=[0, 0.12, 1, 0.95]) 
    
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
with imageio.get_writer(output_gif, mode='I', duration=0.5, loop=0) as writer:
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
