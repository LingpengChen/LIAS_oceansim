import numpy as np

# --- 1. 输入参数 ---
# 这是你已知的内参矩阵和分辨率
camera_matrix = np.array([
            [320, 0.0, 320], 
            [0.0, 320, 240], 
            [0.0, 0.0, 1.0]
        ])
width, height = 640, 480

# 这是你需要查找或估计的关键参数
pixel_size_microns = 3.0  # 单位：微米 (microns)

# --- 2. 计算物理参数 ---
# 从内参矩阵中提取 fx, fy (单位: 像素)
fx = camera_matrix[0, 0]
fy = camera_matrix[1, 1]

# 将像素尺寸从微米(μm)转换为毫米(mm)
pixel_size_mm = pixel_size_microns * 1e-3

# 计算传感器物理尺寸 (Aperture), 单位: 毫米(mm)
# 公式: 传感器尺寸 = 像素数量 * 单个像素尺寸
horizontal_aperture_mm = width * pixel_size_mm
vertical_aperture_mm = height * pixel_size_mm

# 计算物理焦距, 单位: 毫米(mm)
# 公式: 物理焦距 = 像素焦距 * 单个像素尺寸
focal_length_x_mm = fx * pixel_size_mm
focal_length_y_mm = fy * pixel_size_mm
# 取平均值作为最终焦距
focal_length_mm = (focal_length_x_mm + focal_length_y_mm) / 2

# --- 3. 计算视场角 (FOV) ---
# 计算水平和垂直视场角
# 公式: FOV = 2 * arctan(sensor_size / (2 * focal_length))
hfov = 2 * np.arctan2(horizontal_aperture_mm, 2 * focal_length_mm)
vfov = 2 * np.arctan2(vertical_aperture_mm, 2 * focal_length_mm)

# Horizontal Fov is 1.5707963267948966
# Vertical Fov is 0.9817477042468103


# 将弧度转换为角度
hfov_deg = np.degrees(hfov)
vfov_deg = np.degrees(vfov)

print(f"计算得到的物理焦距: {focal_length_mm:.2f} mm")
print(f"计算得到的传感器尺寸 (H x V): {horizontal_aperture_mm:.2f} mm x {vertical_aperture_mm:.2f} mm")
print(f"水平视场角 (HFOV): {hfov_deg:.2f}° {np.deg2rad(hfov_deg):.2f}")
print(f"垂直视场角 (VFOV): {vfov_deg:.2f}° {np.deg2rad(vfov_deg):.2f}")

# from omni.isaac.core.objects import Camera
# from omni.isaac.core.prims import XFormPrim
# import numpy as np

# # 假设你的场景中已经有一个相机，其路径为 /World/Camera
# camera_path = "/World/Camera"

# # --- 使用上面计算出的值 ---
# # Isaac Sim API 需要的单位是米(m)，所以我们需要转换
# # focal_length_mm 来自上一步的计算结果
# # horizontal_aperture_mm, vertical_aperture_mm 同上

# # 1. 设置相机的光学属性 (内参)
# sim_camera = Camera(prim_path=camera_path, resolution=(width, height))
# sim_camera.set_focal_length(focal_length_mm / 1000.0) # mm -> m  2.87/1000
# sim_camera.set_horizontal_aperture(horizontal_aperture_mm / 1000.0) # mm -> m
# # 垂直孔径通常会自动根据分辨率的宽高比计算，但也可以手动设置
# # sim_camera.set_vertical_aperture(vertical_aperture_mm / 1000.0) # mm -> m

# # 设置其他可选参数
# sim_camera.set_clipping_range(0.05, 1.0e5) # 裁剪范围，单位：米
# sim_camera.set_focus_distance(3.0) # 对焦距离，单位：米
# sim_camera.set_lens_aperture(0.0) # f-stop值，设为0.0可禁用景深效果，使所有物体清晰

# # 应用设置
# sim_camera.initialize()
# print("相机光学参数（内参）已设置。")

print(np.tan(0.9817/2)) # 0.5344804696153075*960*2