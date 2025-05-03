import xml.etree.ElementTree as ET
from xml.dom import minidom

# === CONFIG ===
scene_path = "$(dirname)/../assets/scene.xml"
output_launch_path = "complete_sim.launch"
default_pkg = "cv_project"

# === Parse scene.xml to count included agents ===
tree = ET.parse(scene_path)
root = tree.getroot()

includes = [
    inc for inc in root.findall(".//include")
    if inc.get("file", "").startswith("robot_")
]

agent_ids = sorted([int(inc.get("file")[6:-4]) for inc in includes])  # from "robot_X.xml"
agent_num = max(agent_ids) + 1

# === Start building launch XML ===
launch = ET.Element("launch")

# Launch args
for name, default in [
    ("output", "screen"),
    ("xml_file", scene_path),
    ("param", "$(dirname)/../assets/param.yaml"),
    ("package", default_pkg),
]:
    arg = ET.SubElement(launch, "arg", name=name, default=default)

# Simulator node
sim_node = ET.SubElement(launch, "node", {
    "name": "simulator",
    "pkg": "$(arg package)",
    "type": "simulator",
    "output": "$(arg output)"
})
ET.SubElement(sim_node, "param", name="xml_file", value="$(arg xml_file)")
ET.SubElement(sim_node, "param", name="param", value="$(arg param)")

# Per-agent groups
for i in agent_ids:
    group = ET.SubElement(launch, "group", ns=f"robot_{i}")

    setpoint = ET.SubElement(group, "node", {
        "name": "trajectory_generator",
        "pkg": "$(arg package)",
        "type": "trajectory_generator",
        "output": "screen"
    })
    ET.SubElement(setpoint, "param", name="agent_id", value=str(i))
    ET.SubElement(setpoint, "param", name="param", value="$(arg param)")
    
# === Pretty-print to file ===
def pretty_print(elem):
    rough = ET.tostring(elem, "utf-8")
    reparsed = minidom.parseString(rough)
    return reparsed.toprettyxml(indent="  ")

with open(output_launch_path, "w") as f:
    f.write(pretty_print(launch))

print(f"✅ Launch file generated at: {output_launch_path}")
