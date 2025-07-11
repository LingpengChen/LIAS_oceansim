# Omniverse import
import numpy as np
from pxr import Gf, PhysxSchema

# Isaac sim import
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_path

from isaacsim.oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
from isaacsim.oceansim.sensors.UW_Camera import UW_Camera
from isaacsim.oceansim.sensors.DVLsensor import DVLsensor
from isaacsim.sensors.physics import IMUSensor
from isaacsim.oceansim.utils.assets_utils import get_oceansim_assets_path

import omni
import omni.replicator.core as rep
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.utils.render_product import set_camera_prim_path
from isaacsim.core.utils.rotations import euler_angles_to_quat
from isaacsim.oceansim.utils.config import *

import warp as wp


if ROS_TAG:
    import rospy
    from std_msgs.msg import String, Header
    from sensor_msgs.msg import Image, Imu
    from geometry_msgs.msg import PoseStamped, Point, Quaternion, Vector3 # <--- 添加 PoseStamped 和其他几何消息
    import tf2_ros
    from geometry_msgs.msg import TransformStamped
    from sensor_msgs.msg import CameraInfo # <--- 添加 CameraInfo


save_img_dir = ""

class MHL_Sensor_Example_Scenario():
    def __init__(self):
        self._rob = None
        self._sonar = None
        self._cam = None
        self._cam2 = None
        self._DVL = None
        self._IMU = None
        self._baro = None

        self._ctrl_mode = None

        self._running_scenario = False
        self._time = 0.0

        if ROS_TAG:

            rospy.init_node("isaacsim_traditional_rospy_publisher", disable_signals=True)
            print("INFO: Traditional rospy node initialized.")

            camera_base_topic = "/isaacsim/camera" 
            camera2_base_topic = "/isaacsim/camera2" 

            # 使用这个基命名空间来创建publishers
            self.image_pub = rospy.Publisher(camera_base_topic + "/image_raw", Image, queue_size=1)
            self.depth_pub = rospy.Publisher(camera_base_topic + "/depth/image_raw", Image, queue_size=1) # 深度图通常有自己的子命名空间
            self.camera_info_pub = rospy.Publisher(camera_base_topic+ "/camera_info", CameraInfo, queue_size=1)
            
            self.image_pub2 = rospy.Publisher(camera2_base_topic + "/image_raw", Image, queue_size=1)
            self.depth_pub2 = rospy.Publisher(camera2_base_topic + "/depth/image_raw", Image, queue_size=1) # 深度图通常有自己的子命名空间
            
            self.sonar_pub = rospy.Publisher('/isaacsim/sonar_rect_image', Image, queue_size=1)
            self.dvl_pub = rospy.Publisher('/isaacsim/DVL', Image, queue_size=1)
            self.imu_pub = rospy.Publisher("/isaacsim/imu/data", Imu, queue_size=10)
            self.pose_gt_pub = rospy.Publisher("/isaacsim/ground_truth/pose", PoseStamped, queue_size=10)

           
            # 定义机器人主坐标系和其他坐标系ID
            self._imu_frame_id = "imu_link"      # IMU/机器人本体坐标系
            self._world_frame_id = "world"       # 世界坐标系
            self._camera_frame_id = "camera_link"
            self._camera2_frame_id = "camera2_link"
            self._sonar_frame_id = "sonar_link"
            self._dvl_frame_id = "dvl_link"

            # 创建一个列表来存储所有的静态变换
            static_transforms = []

            # self.string_pub = rospy.Publisher("/isaacsim/my_string_topic", String, queue_size=10)
            # --- 新增：初始化 TF 广播器 ---
            self.tf_broadcaster = tf2_ros.TransformBroadcaster()
            self.static_tf_broadcaster = tf2_ros.StaticTransformBroadcaster()
            
            # --- 新增：在这里定义传感器的相对位置和名称 ---
            # 定义传感器信息 (名称，平移向量)
            sensor_setups = [
                (self._camera_frame_id, (Cam_trans, Cam_orientation)),
                (self._camera2_frame_id, (Cam2_trans, Cam2_orientation)),
                (self._sonar_frame_id,  (Sonar_trans, Sonar_orientation)),
                (self._dvl_frame_id,    (DVL_trans, DVL_orientation))
            ]

            # 2. 修改循环以处理新的数据结构
            static_transforms = []
            for child_frame, (translation, orientation) in sensor_setups:
                t = TransformStamped()

                t.header.stamp = rospy.Time.now()
                t.header.frame_id = self._imu_frame_id  # 所有传感器都相对于 imu_link
                t.child_frame_id = child_frame

                # 填充平移 (Translation)
                t.transform.translation.x = translation[0]
                t.transform.translation.y = translation[1]
                t.transform.translation.z = translation[2]

                # 填充旋转 (Orientation)
                # 假设 orientation 是一个包含 [x, y, z, w] 的列表或Numpy数组
                t.transform.rotation.w = orientation[0]
                t.transform.rotation.x = orientation[1]
                t.transform.rotation.y = orientation[2]
                t.transform.rotation.z = orientation[3]

                static_transforms.append(t)

            # `static_transforms` 列表现在包含了带有正确平移和旋转的变换
            # 接下来的发布代码保持不变
            self.static_tf_broadcaster.sendTransform(static_transforms)
            
            print(f"INFO: Published static transforms for camera, sonar, and DVL relative to '{self._imu_frame_id}'.")


    ##################################################################################
    ##################################################################################
    ##################################################################################
    ##################################################################################
    
    def setup_scenario(self, rob, sonar: ImagingSonarSensor, cam: UW_Camera, cam2: UW_Camera,  DVL: DVLsensor, baro, ctrl_mode, use_IMU=True):
        self._rob = rob
        self._sonar = sonar
        self._cam = cam
        self._cam2 = cam2
        self._DVL = DVL
        self._baro = baro
        self._ctrl_mode = ctrl_mode
        if self._sonar is not None:
            self._sonar.sonar_initialize(viewport=SONAR_VIEW, include_unlabelled=True)
        if self._cam is not None:
            # self._cam.initialize()
            uw_cam_yaml_path = get_oceansim_assets_path() + CAM_CONFIG_FILE_PATH
            # uw_cam_yaml_path = get_oceansim_assets_path() + "/render/render_shenzhen_clear/render_param_0.yaml"
            self._cam.initialize(UW_yaml_path=uw_cam_yaml_path, viewport=CAM_VIEW)
        if self._cam2 is not None:
            # self._cam.initialize()
            uw_cam_yaml_path = get_oceansim_assets_path() + CAM_CONFIG_FILE_PATH
            # uw_cam_yaml_path = get_oceansim_assets_path() + "/render/render_shenzhen_clear/render_param_0.yaml"
            self._cam2.initialize(UW_yaml_path=uw_cam_yaml_path, viewport=CAM_VIEW)
        if self._DVL is not None:
            self._DVL_reading = [0.0, 0.0, 0.0]
            self._DVL.set_freq(10)
        if self._baro is not None:
            self._baro_reading = 101325.0 # atmospheric pressure (Pa)
        
        if use_IMU:
            robot_prim_path = "/World/rob"
            from isaacsim.sensors.physics import IMUSensor
            # imu_prim = robot_prim_path + "/Imu"  # /World/rob/Imu
            imu_prim = robot_prim_path + "/Imu"  # /World/rob/Imu
            self._IMU = IMUSensor(
                prim_path=imu_prim,
                name="imu",
                frequency=IMU_frequency,
                translation=IMU_trans,
                orientation=IMU_orientation,
                linear_acceleration_filter_size = 10,
                angular_velocity_filter_size = 10,
                orientation_filter_size = 10
            )
            
            self._IMU.initialize()
        
        # Apply the physx force schema if manual control
        if ctrl_mode == "Manual control":
            from ...utils.keyboard_cmd import keyboard_cmd

            self._rob_forceAPI = PhysxSchema.PhysxForceAPI.Apply(self._rob)
            force_value = 2.0
            torque_value = 1.0
            self._force_cmd = keyboard_cmd(base_command=np.array([0.0, 0.0, 0.0]),
                                      input_keyboard_mapping={
                                        # forward command
                                        "W": [force_value, 0.0, 0.0],
                                        # backward command
                                        "S": [-force_value, 0.0, 0.0],
                                        # leftward command
                                        "A": [0.0, force_value, 0.0],
                                        # rightward command
                                        "D": [0.0, -force_value, 0.0],
                                         # rise command
                                        "UP": [0.0, 0.0, force_value],
                                        # sink command
                                        "DOWN": [0.0, 0.0, -force_value],
                                      })
            self._torque_cmd = keyboard_cmd(base_command=np.array([0.0, 0.0, 0.0]),
                                      input_keyboard_mapping={
                                        # yaw command (left)
                                        "J": [0.0, 0.0, torque_value],
                                        # yaw command (right)
                                        "L": [0.0, 0.0, -torque_value],
                                        # pitch command (up)
                                        "I": [0.0, -torque_value, 0.0],
                                        # pitch command (down)
                                        "K": [0.0, torque_value, 0.0],
                                        # row command (left)
                                        "LEFT": [-torque_value, 0.0, 0.0],
                                        # row command (negative)
                                        "RIGHT": [torque_value, 0.0, 0.0],
                                      })
            
        self._running_scenario = True

    # This function will only be called if ctrl_mode==waypoints and waypoints files are changed
    def setup_waypoints(self, waypoint_path, default_waypoint_path):
        def read_data_from_file(file_path):
            # Initialize an empty list to store the floats
            data = []
            
            # Open the file in read mode
            with open(file_path, 'r') as file:
                # Read each line in the file
                for line in file:
                    # Strip any leading/trailing whitespace and split the line by spaces
                    float_strings = line.strip().split()
                    
                    # Convert the list of strings to a list of floats
                    floats = [float(x) for x in float_strings]
                    
                    # Append the list of floats to the data list
                    data.append(floats)
            
            return data
        try:
            self.waypoints = read_data_from_file(waypoint_path)
            print('Waypoints loaded successfully.')
            print(f'Waypoint[0]: {self.waypoints[0]}')
        except:
            self.waypoints = read_data_from_file(default_waypoint_path)
            print('Fail to load this waypoints. Back to default waypoints.')

        
    def teardown_scenario(self):

        # Because these two sensors create annotator cache in GPU,
        # close() will detach annotator from render product and clear the cache.
        if self._sonar is not None:
            self._sonar.close()
        if self._cam is not None:
            self._cam.close()
        if self._cam2 is not None:
            self._cam2.close()

        # clear the keyboard subscription
        if self._ctrl_mode=="Manual control":
            self._force_cmd.cleanup()
            self._torque_cmd.cleanup()

        self._rob = None
        self._sonar = None
        self._cam = None
        self._cam2 = None
        self._DVL = None
        self._IMU = None
        self._baro = None
        self._running_scenario = False
        self._time = 0.0


    # "rgb8": 表示数据是R、G、B顺序排列的，每个通道是8位无符号整数。
    # "bgr8": OpenCV的默认格式，B、G、R顺序。
    # "mono8": 8位灰度图。
    # "mono16": 16位灰度图。
    # "32FC1": 32位浮点数的单通道图（常用于深度图）
    
    def update_scenario(self, step: float):

        if not self._running_scenario:
            return
        
        self._time += step
        
        if self._IMU is not None:
            imu_data = self._IMU.get_current_frame()
            timestamp = imu_data['time']
            world_pose_data = self._IMU.get_world_pose()
            
            # IMU value {'time': 0, 'physics_step': 0, 'lin_acc': array([0., 0., 0.], dtype=float32), 'ang_vel': array([0., 0., 0.], dtype=float32), 'orientation': array([1., 0., 0., 0.], dtype=float32)}
            # world_pose (array([-2.096689,  0.      , -0.8     ], dtype=float32), array([1., 0., 0., 0.], dtype=float32))
            if ROS_TAG:
                ros_timestamp = rospy.Time.from_sec(timestamp)
                # --- 2. 封装并发布 sensor_msgs/Imu ---
                imu_msg = Imu()
                
                # 填充 Header
                imu_msg.header.stamp = ros_timestamp
                imu_msg.header.frame_id = self._imu_frame_id
                
                # 填充 Orientation (注意 Isaac Sim 是 [w,x,y,z], ROS 是 x,y,z,w 字段)
                imu_msg.orientation.w = imu_data['orientation'][0]
                imu_msg.orientation.x = imu_data['orientation'][1]
                imu_msg.orientation.y = imu_data['orientation'][2]
                imu_msg.orientation.z = imu_data['orientation'][3]
                
                # 填充 Angular Velocity
                imu_msg.angular_velocity.x = imu_data['ang_vel'][0]
                imu_msg.angular_velocity.y = imu_data['ang_vel'][1]
                imu_msg.angular_velocity.z = imu_data['ang_vel'][2]
                
                # 填充 Linear Acceleration
                imu_msg.linear_acceleration.x = imu_data['lin_acc'][0]
                imu_msg.linear_acceleration.y = imu_data['lin_acc'][1]
                imu_msg.linear_acceleration.z = imu_data['lin_acc'][2]
                
                # 填充协方差矩阵 (如果不知道，可以先用0填充)
                # 对角线上的非零值表示该轴有测量值，-1表示无测量值
                imu_msg.orientation_covariance = [0.01, 0, 0, 0, 0.01, 0, 0, 0, 0.01] # 示例值
                imu_msg.angular_velocity_covariance = [0.005, 0, 0, 0, 0.005, 0, 0, 0, 0.005] # 示例值
                imu_msg.linear_acceleration_covariance = [0.02, 0, 0, 0, 0.02, 0, 0, 0, 0.02] # 示例值
                
                # 发布IMU消息
                self.imu_pub.publish(imu_msg)
                
                
                # # --- 3. 封装并发布 geometry_msgs/PoseStamped ---
                # pose_msg = PoseStamped()

                # # 填充 Header
                # pose_msg.header.stamp = ros_timestamp
                # pose_msg.header.frame_id = self._world_frame_id # 真值位姿是相对于世界坐标系的
                
                # # 填充 Position
                # pose_msg.pose.position.x = world_pose_data[0][0]
                # pose_msg.pose.position.y = world_pose_data[0][1]
                # pose_msg.pose.position.z = world_pose_data[0][2]
                
                # # 填充 Orientation (同样注意 [w,x,y,z] -> x,y,z,w 字段)
                # pose_msg.pose.orientation.w = world_pose_data[1][0]
                # pose_msg.pose.orientation.x = world_pose_data[1][1]
                # pose_msg.pose.orientation.y = world_pose_data[1][2]
                # pose_msg.pose.orientation.z = world_pose_data[1][3]
                
                # # 发布Pose GT消息
                # self.pose_gt_pub.publish(pose_msg)
                # --- 新增：发布动态 TF (world -> imu_link) ---
                t = TransformStamped()
                t.header.stamp = ros_timestamp
                t.header.frame_id = self._world_frame_id
                t.child_frame_id = self._imu_frame_id

                # 从 world_pose_data 填充变换信息
                t.transform.translation.x = world_pose_data[0][0] - INITIAL_T[0]
                t.transform.translation.y = world_pose_data[0][1] - INITIAL_T[1]
                t.transform.translation.z = world_pose_data[0][2] - INITIAL_T[2] + 0.5
                
                # Isaac Sim: [w, x, y, z], ROS: [x, y, z, w]
                t.transform.rotation.x = world_pose_data[1][1] 
                t.transform.rotation.y = world_pose_data[1][2] 
                t.transform.rotation.z = world_pose_data[1][3] 
                t.transform.rotation.w = world_pose_data[1][0]
                
                self.tf_broadcaster.sendTransform(t)
                
        if self._time % ( 1/(RENDERING_DT*Retrieve_image_frequency) * step ) != 0: # 10 Hz for image
            return
        
        
        if self._sonar is not None:
            sonar_map_numpy = self._sonar.make_sonar_data()
            # print(sonar_map_numpy.shape, sonar_map_numpy.dtype) # (960, 240, 4) uint8
            # print(np.sum(sonar_map_numpy[:,:,0])-np.sum(sonar_map_numpy[:,:,1]))
            # print(np.sum(sonar_map_numpy[:,:,0])-np.sum(sonar_map_numpy[:,:,2]))
            # print(np.sum(sonar_map_numpy[:,:,0])-np.sum(sonar_map_numpy[:,:,3]))
            # print()
            # [ImagingSonar] Render query res: 4000 x 800. Binning res: 960 x 240

            if sonar_map_numpy is not None and ROS_TAG:
                sonar_msg = Image()
                h_d, w_d = sonar_map_numpy.shape # (960, 240) uint8
                sonar_msg.header = Header(stamp=ros_timestamp, frame_id=self._sonar_frame_id) # 使用相同的时间戳和frame_id
                sonar_msg.height = h_d
                sonar_msg.width = w_d
                sonar_msg.encoding = "mono8"
                sonar_msg.step = w_d
               
                sonar_msg.data = sonar_map_numpy.tobytes()
                self.sonar_pub.publish(sonar_msg)
                
        if self._cam is not None:
            print(timestamp)
            
            uw_image_rgb, depth_map_numpy = self._cam.render()
            uw_image_rgb2, depth_map_numpy2 = self._cam2.render()

            if uw_image_rgb is not None and depth_map_numpy is not None and ROS_TAG:
                
                # 发布 uw_image_rgb
                (height, width, channel) = uw_image_rgb.shape
                # print(uw_image_rgb.shape) # (1080, 1920, 3)
                rgb_msg = Image()
                rgb_msg.header = Header(stamp=ros_timestamp, frame_id=self._camera_frame_id)

                rgb_msg.height = height
                rgb_msg.width = width
                rgb_msg.encoding = "rgb8"  # 指明是RGB格式，每个通道8位
                rgb_msg.is_bigendian = 0
                rgb_msg.step = width * 3  # 每行的字节数 = 宽度 * 通道数 * 每个通道的字节数
                rgb_msg.data = uw_image_rgb.tobytes() # 将numpy数组平铺成字节流

                self.image_pub.publish(rgb_msg)
                
                # ##########################################################
                # # 发布 depth_map_numpy
                depth_msg = Image()
                h_d, w_d = depth_map_numpy.shape
                # print(h_d, w_d, depth_map_numpy.dtype) # 1080 1920 float32
                depth_msg.header = Header(stamp=ros_timestamp, frame_id=self._camera_frame_id) # 使用相同的时间戳和frame_id
                depth_msg.height = h_d
                depth_msg.width = w_d
                
                depth_msg.encoding = "32FC1"
                depth_msg.step = w_d * 4 # 每个像素4个字节
                # # 【关键】设置正确的深度图编码格式
                # # 深度图通常是32位浮点数，单位是米
                # if depth_map_numpy.dtype == np.float32:
                #     depth_msg.encoding = "32FC1"
                #     depth_msg.step = w_d * 4 # 每个像素4个字节
                # elif depth_map_numpy.dtype == np.uint16:
                #     depth_msg.encoding = "16UC1" # 如果深度图是16位无符号整数（如毫米为单位）
                #     depth_msg.step = w_d * 2 # 每个像素2个字节
                # else:
                #     # 添加对其他可能格式的处理或报错
                #     rospy.logwarn_throttle(1.0, f"Unsupported depth format: {depth_map_numpy.dtype}")

                depth_msg.data = depth_map_numpy.tobytes()
                self.depth_pub.publish(depth_msg)
                
                ##########################################################
                ##########################################################
                
                cam_info_msg = CameraInfo()
                cam_info_msg.header.stamp = ros_timestamp
                cam_info_msg.header.frame_id = self._camera_frame_id
                cam_info_msg.height = height
                cam_info_msg.width = width
                
                # 从你的参数中提取fx, fy, cx, cy
                fx = FX
                fy = FY
                cx = CX
                cy = CY

                cam_info_msg.distortion_model = "plumb_bob"
                cam_info_msg.D = [0.0, 0.0, 0.0, 0.0, 0.0]
                cam_info_msg.K = [fx, 0.0, cx, 0.0, fy, cy, 0.0, 0.0, 1.0]
                cam_info_msg.R = [1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0]
                cam_info_msg.P = [fx, 0.0, cx, 0.0, 0.0, fy, cy, 0.0, 0.0, 0.0, 1.0, 0.0]

                self.camera_info_pub.publish(cam_info_msg)
                
            if uw_image_rgb2 is not None and depth_map_numpy2 is not None and ROS_TAG:
                
                # 发布 uw_image_rgb
                (height, width, channel) = uw_image_rgb2.shape
                rgb_msg = Image()
                rgb_msg.header = Header(stamp=ros_timestamp, frame_id=self._camera2_frame_id)

                rgb_msg.height = height
                rgb_msg.width = width
                rgb_msg.encoding = "rgb8"  # 指明是RGB格式，每个通道8位
                rgb_msg.is_bigendian = 0
                rgb_msg.step = width * 3  # 每行的字节数 = 宽度 * 通道数 * 每个通道的字节数
                rgb_msg.data = uw_image_rgb2.tobytes() # 将numpy数组平铺成字节流

                self.image_pub2.publish(rgb_msg)
                
                ##########################################################
                depth_msg = Image()
                h_d, w_d = depth_map_numpy2.shape
                depth_msg.header = Header(stamp=ros_timestamp, frame_id=self._camera2_frame_id) # 使用相同的时间戳和frame_id
                depth_msg.height = h_d
                depth_msg.width = w_d
                
                depth_msg.encoding = "32FC1"
                depth_msg.step = w_d * 4 # 每个像素4个字节

                depth_msg.data = depth_map_numpy2.tobytes()
                self.depth_pub2.publish(depth_msg)
                
   
        if self._DVL is not None:
            # self._DVL_reading = self._DVL.get_linear_vel()
            dvl_vel = self._DVL.get_linear_vel_fd(step)
            dvl_depth = self._DVL.get_depth_fd(step)
            if isinstance(dvl_vel, np.ndarray): # nan of numpy array
                print(dvl_vel, dvl_depth)
            
        
            
        if self._baro is not None:
            self._baro_reading = self._baro.get_pressure()
            
        

        if self._ctrl_mode=="Manual control":
            force_cmd = Gf.Vec3f(*self._force_cmd._base_command)
            torque_cmd = Gf.Vec3f(*self._torque_cmd._base_command)
            self._rob_forceAPI.CreateForceAttr().Set(force_cmd)
            self._rob_forceAPI.CreateTorqueAttr().Set(torque_cmd)
        elif self._ctrl_mode=="Waypoints":
            if len(self.waypoints) > 0:
                waypoints = self.waypoints[0]
                self._rob.GetAttribute('xformOp:translate').Set(Gf.Vec3f(waypoints[0], waypoints[1], waypoints[2]))
                self._rob.GetAttribute('xformOp:orient').Set(Gf.Quatd(waypoints[3], waypoints[4], waypoints[5], waypoints[6]))
                self.waypoints.pop(0)
            else:
                print('Waypoints finished')                
        elif self._ctrl_mode=="Straight line":
            SingleRigidPrim(prim_path=get_prim_path(self._rob)).set_linear_velocity(np.array([0.5,0,0])) 




        

        


