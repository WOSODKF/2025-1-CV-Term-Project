import xml.etree.ElementTree as ET
from xml.dom import minidom

# === CONFIG ===
scene_path = "../assets/scene.xml"
output_launch_path = "complete_sim.launch"
default_pkg = "cv_project"
yolo_model_path = "../yolo_model/runs/segment/train/weights/best.pt"

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
    ("xml_file", "$(dirname)/" + scene_path),
    ("param", "$(dirname)/../assets/param.yaml"),
    ("model", "$(dirname)/" + yolo_model_path),
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

# segmentor node
seg_node = ET.SubElement(launch, "node",{
    "name": "segmentor",
    "pkg": "$(arg package)",
    "type": "image_segmentor",
    "output": "$(arg output)"
})
ET.SubElement(seg_node, "param", name="param", value="$(arg param)")
ET.SubElement(seg_node, "param", name="agent_num", value=str(agent_num))
ET.SubElement(seg_node, "param", name="model_dir", value="$(arg model)")

# mesh estimator node
mesh_node = ET.SubElement(launch, "node",{
    "name": "mesh_estimator",
    "pkg": "$(arg package)",
    "type": "mesh_estimator",
    "output": "$(arg output)"
})
ET.SubElement(mesh_node, "param", name="param", value="$(arg param)")
ET.SubElement(mesh_node, "param", name="agent_num", value=str(agent_num))

# param estimator node
param_node = ET.SubElement(launch, "node",{
    "name": "param_estimator",
    "pkg": "$(arg package)",
    "type": "param_estimator",
    "output": "$(arg output)"
})
ET.SubElement(param_node, "param", name="param", value="$(arg param)")
ET.SubElement(param_node, "param", name="agent_num", value=str(agent_num))

# mesh visualizer node
vis_node = ET.SubElement(launch, "node", {
    "name": "mesh_visualizer",
    "pkg": "$(arg package)",
    "type": "mesh_visualizer",
    "output": "$(arg output)"
})
ET.SubElement(vis_node, "param", name="param", value="$(arg param)")
ET.SubElement(vis_node, "param", name="frame_id", value="world")


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
