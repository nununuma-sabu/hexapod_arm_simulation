import ikpy.chain
import numpy as np
import subprocess
import tempfile
import os

def create_raw_urdf(xacro_path):
    raw_urdf_path = tempfile.mktemp(suffix=".urdf")
    # Use full path to be safe
    abs_xacro = os.path.abspath(xacro_path)
    cmd = f"bash -c 'source /opt/ros/humble/setup.bash && xacro {abs_xacro} > {raw_urdf_path}'"
    subprocess.run(cmd, shell=True)
    return raw_urdf_path

urdf_file = "src/hexapod_arm_bot_description/urdf/hexapod.urdf.xacro"
raw_urdf = create_raw_urdf(urdf_file)

print(f"Loading chain from {raw_urdf}...")
chain = ikpy.chain.Chain.from_urdf_file(raw_urdf, base_elements=["base_footprint"])

print("\nLink positions at 0 angles:")
joints = chain.forward_kinematics([0.0]*len(chain.links), full_kinematics=True)
for i, link in enumerate(chain.links):
    pos = joints[i][:3, 3]
    print(f"{i}: {link.name} at {pos}")

os.remove(raw_urdf)
