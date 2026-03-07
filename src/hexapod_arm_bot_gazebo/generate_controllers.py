import yaml
import os

def generate_yaml():
    arms = ["arm_L1", "arm_L2", "arm_L3", "arm_R1", "arm_R2", "arm_R3"]
    gimbals = ["g1", "g2", "g3", "g4", "g5"]
    axes = ["roll", "pitch", "yaw"]

    joints = []
    for arm in arms:
        for g in gimbals:
            for axis in axes:
                joints.append(f"{arm}_{g}_{axis}_joint")

    config = {
        "controller_manager": {
            "ros__parameters": {
                "update_rate": 100,
                "joint_state_broadcaster": {
                    "type": "joint_state_broadcaster/JointStateBroadcaster"
                },
                "hexapod_arm_controller": {
                    "type": "joint_trajectory_controller/JointTrajectoryController"
                }
            }
        },
        "hexapod_arm_controller": {
            "ros__parameters": {
                "joints": joints,
                "command_interfaces": ["position"],
                "state_interfaces": ["position", "velocity"],
                "state_publish_rate": 100.0,
                "action_monitor_rate": 20.0,
                "allow_partial_joints_goal": True
            }
        }
    }

    # Write to config directory
    os.makedirs("/home/numa/robot/src/hexapod_arm_bot_gazebo/config", exist_ok=True)
    with open("/home/numa/robot/src/hexapod_arm_bot_gazebo/config/ros2_controllers.yaml", "w") as f:
        yaml.dump(config, f, default_flow_style=False, sort_keys=False)
    
    print("Generated ros2_controllers.yaml")

if __name__ == "__main__":
    generate_yaml()
