import numpy as np
from isaacsim.core.utils.rotations import euler_angles_to_quat
# from isaacsim.oceansim.utils.config import *

CAM_CONFIG_FILE_PATH = "/render/sea_vague_50/render_param_0.yaml"

ROS_TAG = True
# ROS_TAG = False

SONAR_VIEW = not ROS_TAG
CAM_VIEW = not ROS_TAG

DEFAULT_SONAR = False
DEFAULT_CAM = True
DEFAULT_DVL = False
DEFAULT_IMU = True

# PHYSICS_DT = 1/60
PHYSICS_DT = 1/100
RENDERING_DT = 1/100
IMU_frequency = 100 # simulation frequency

Retrieve_image_frequency = 10


# INITIAL_T = np.array([-0.0, 2, 1])
INITIAL_T = np.array([-12, 17, 1.2])
ROB_MASS = 10 # kg
Angular_damping = 3
Linear_damping = 3

Sonar_trans = np.array([0.3, 0.2, 0.0])
Sonar_orientation=euler_angles_to_quat(np.array([0.0, 90.0, 0.0]), degrees=True)

Cam_trans = np.array([0.3, 0, 0.0])
Cam_orientation=euler_angles_to_quat(np.array([0.0, 90.0, 0.0]), degrees=True)
Cam2_trans = Sonar_trans
Cam2_orientation=Sonar_orientation

DVL_trans = np.array([0,0,-0.05])
DVL_orientation=euler_angles_to_quat(np.array([0.0, 0.0, 0.0]), degrees=True) # np.array([1.0, 0.0, 0.0, 0.0])
IMU_trans = np.array([0, 0, 0])
IMU_orientation = np.array([1, 0, 0, 0])

Cam_image_width = 1920
Cam_image_height = 1080
FX = 960.0
FY = 960.0
CX = 960.0
CY = 540.0


pixel_size_microns = 3.0  # 单位：微米 (microns)
# 将像素尺寸从微米(μm)转换为毫米(mm)
pixel_size_mm = pixel_size_microns * 1e-3

# 计算传感器物理尺寸 (Aperture), 单位: 毫米(mm)
# 公式: 传感器尺寸 = 像素数量 * 单个像素尺寸
Horizontal_aperture_mm = Cam_image_width * pixel_size_mm
Vertical_aperture_mm = Cam_image_height * pixel_size_mm

# 计算物理焦距, 单位: 毫米(mm)
# 公式: 物理焦距 = 像素焦距 * 单个像素尺寸
focal_length_x_mm = FX * pixel_size_mm
focal_length_y_mm = FY * pixel_size_mm
# 取平均值作为最终焦距
Focal_length_mm = (focal_length_x_mm + focal_length_y_mm) / 2 

CAM_RANGE = 20