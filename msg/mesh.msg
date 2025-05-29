std_msgs/Header header

bool initial

uint32 rows           # number of rows (e.g., 8)
uint32 cols           # number of cols (e.g., 8)
geometry_msgs/Point[] positions  # row-major: positions[i * cols + j]