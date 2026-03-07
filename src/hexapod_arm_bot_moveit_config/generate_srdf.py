import xml.etree.ElementTree as ET
import os

def generate_srdf():
    robot = ET.Element('robot', name="hexapod_arm_bot")
    
    arms = ["arm_L1", "arm_L2", "arm_L3", "arm_R1", "arm_R2", "arm_R3"]
    
    # groups
    all_arms = ET.SubElement(robot, 'group', name="all_arms")
    for arm in arms:
        group = ET.SubElement(robot, 'group', name=arm)
        ET.SubElement(group, 'chain', base_link="body_link", tip_link=f"{arm}_cyl6")
        ET.SubElement(all_arms, 'group', name=arm)

    # Disable collisions logic
    # We disable intra-arm collisions and base-body collisions to avoid false positives.
    # Inter-arm collisions and arm-to-distant-body collisions are preserved, enabling interference check.
    links = ["body_link"]
    arm_links = {}
    for arm in arms:
        arm_links[arm] = [f"{arm}_cyl1", f"{arm}_cyl2", f"{arm}_cyl3", f"{arm}_cyl4", f"{arm}_cyl5", f"{arm}_cyl6"]
        for g in ["g1", "g2", "g3", "g4", "g5"]:
            arm_links[arm].extend([f"{arm}_{g}_roll_link", f"{arm}_{g}_pitch_link", f"{arm}_{g}_yaw_link"])
        links.extend(arm_links[arm])
        
    for i, l1 in enumerate(links):
        for j in range(i+1, len(links)):
            l2 = links[j]
            same_arm = False
            for arm in arms:
                if l1 in arm_links[arm] and l2 in arm_links[arm]:
                    same_arm = True
                    break
            
            body_adj = False
            if (l1 == "body_link" and l2 in arm_links[l2.split('_')[0] + "_" + l2.split('_')[1]]) and ("cyl1" in l2 or "g1" in l2):
                body_adj = True
            elif (l2 == "body_link" and l1 in arm_links[l1.split('_')[0] + "_" + l1.split('_')[1]]) and ("cyl1" in l1 or "g1" in l1):
                body_adj = True
                
            if same_arm or body_adj:
                ET.SubElement(robot, 'disable_collisions', link1=l1, link2=l2, reason="Adjacent/SameArm")

    # Format and write
    tree = ET.ElementTree(robot)
    ET.indent(tree, space="  ", level=0)
    
    os.makedirs("/home/numa/robot/src/hexapod_arm_bot_moveit_config/config", exist_ok=True)
    tree.write("/home/numa/robot/src/hexapod_arm_bot_moveit_config/config/hexapod_arm_bot.srdf", encoding="utf-8", xml_declaration=True)
    print("Generated SRDF")

if __name__ == "__main__":
    generate_srdf()
