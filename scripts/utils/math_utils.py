import numpy as np
import math

def skew(v:np.ndarray):
    return np.array([[0, -v(2), v(1)],
                    [v(2), 0, -v(0)],
                    [-v(1), v(0), 0]])

def unskew(R:np.ndarray):
    # assert R.ndim == 2 and np.shape(R) == (3, 3), "Invalid input for unskew (not 3x3 matrix)"
    # assert np.all(np.abs(R + R.transpose()) < 1e-4), "Invalid input for unskew (not skew-symmetric)"
    
    v = np.array([-R[1,2], R[0,2], -R[0,1]])
    
    return v

def rot_z(z, in_degrees=True):
    if in_degrees:
        c = math.pi/180
    else:
        c = 1.0
    return np.array([[math.cos(c * z), -math.sin(c * z), 0],
                    [math.sin(c * z), math.cos(c * z), 0],
                    [0, 0, 1]])
    
def rot_y(y, in_degrees=True):
    if in_degrees:
        c = math.pi/180
    else:
        c = 1.0
    return np.array([[math.cos(c * y), 0, math.sin(c * y)],
                      [0, 1, 0],
                      [-math.sin(c * y), 0, math.cos(c * y)]])
    
def rot_x(x, in_degrees=True):
    if in_degrees:
        c = math.pi/180
    else:
        c = 1.0
    return np.array([[1, 0, 0],
                      [0, math.cos(c * x), -math.sin(c * x)],
                      [0, math.sin(c* x), math.cos(c * x)]])

def zyx_euler_to_mat(zyx_euler:np.ndarray, in_degrees=True):
    # assert zyx_euler.ndim == 1 and np.shape(zyx_euler) == 3, "Invalid input for euler_to_mat"
    z = zyx_euler[0]
    y = zyx_euler[1]
    x = zyx_euler[2]
    
    return rot_z(z,in_degrees) @ rot_y(y, in_degrees) @ rot_x(x, in_degrees)

def zyx_euler_to_quat(zyx_euler, in_degree=True):
    yaw = zyx_euler[0]
    pitch = zyx_euler[1]
    roll = zyx_euler[2]
    
    if in_degree:
        c = math.pi/180
    else:
        c = 1.0
    
    cy = np.cos(c * yaw * 0.5)
    sy = np.sin(c * yaw * 0.5)
    cp = np.cos(c * pitch * 0.5)
    sp = np.sin(c * pitch * 0.5)
    cr = np.cos(c * roll * 0.5)
    sr = np.sin(c * roll * 0.5)

    w = cr * cp * cy + sr * sp * sy
    x = sr * cp * cy - cr * sp * sy
    y = cr * sp * cy + sr * cp * sy
    z = cr * cp * sy - sr * sp * cy

    return np.array([w, x, y, z])

def quat_to_mat(quat):
    w = quat[0]
    x = quat[1]
    y = quat[2]
    z = quat[3]
    
    norm = np.sqrt(w**2 + x**2 + y**2 + z**2)
    w, x, y, z = w / norm, x / norm, y / norm, z / norm

    rot = np.array([
        [1 - 2*(y**2 + z**2),     2*(x*y - z*w),       2*(x*z + y*w)],
        [2*(x*y + z*w),           1 - 2*(x**2 + z**2), 2*(y*z - x*w)],
        [2*(x*z - y*w),           2*(y*z + x*w),       1 - 2*(x**2 + y**2)]
    ])
    
    return rot

import numpy as np

def mat_to_euler_zyx(R:np.ndarray, in_degrees=True):
    if abs(R[2, 0]) < 1.0:  # Normal case
        pitch = np.arcsin(-R[2, 0])
        yaw = np.arctan2(R[1, 0], R[0, 0])
        roll = np.arctan2(R[2, 1], R[2, 2])
    else:  # Gimbal lock case
        pitch = np.pi/2 if R[2, 0] <= -1.0 else -np.pi/2
        yaw = np.arctan2(-R[0, 1], R[1, 1])
        roll = 0.0  # not uniquely defined
        
    if in_degrees:
        c = 180/math.pi
    else:
        c = 1.0

    return c * np.array([yaw, pitch, roll])

def mat_to_quat(R):
    trace = np.trace(R)
    if trace > 0:
        S = np.sqrt(trace + 1.0) * 2  # S = 4 * w
        w = 0.25 * S
        x = (R[2, 1] - R[1, 2]) / S
        y = (R[0, 2] - R[2, 0]) / S
        z = (R[1, 0] - R[0, 1]) / S
    elif (R[0, 0] > R[1, 1]) and (R[0, 0] > R[2, 2]):
        S = np.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
        w = (R[2, 1] - R[1, 2]) / S
        x = 0.25 * S
        y = (R[0, 1] + R[1, 0]) / S
        z = (R[0, 2] + R[2, 0]) / S
    elif R[1, 1] > R[2, 2]:
        S = np.sqrt(1.0 + R[1, 1] - R[0, 0] - R[2, 2]) * 2
        w = (R[0, 2] - R[2, 0]) / S
        x = (R[0, 1] + R[1, 0]) / S
        y = 0.25 * S
        z = (R[1, 2] + R[2, 1]) / S
    else:
        S = np.sqrt(1.0 + R[2, 2] - R[0, 0] - R[1, 1]) * 2
        w = (R[1, 0] - R[0, 1]) / S
        x = (R[0, 2] + R[2, 0]) / S
        y = (R[1, 2] + R[2, 1]) / S
        z = 0.25 * S

    return np.array([w, x, y, z])