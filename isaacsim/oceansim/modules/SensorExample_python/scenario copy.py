# Omniverse import
import numpy as np
from pxr import Gf, PhysxSchema

# Isaac sim import
from isaacsim.core.prims import SingleRigidPrim
from isaacsim.core.utils.prims import get_prim_path

from isaacsim.oceansim.sensors.ImagingSonarSensor import ImagingSonarSensor
from isaacsim.oceansim.sensors.UW_Camera import UW_Camera
from isaacsim.oceansim.utils.assets_utils import get_oceansim_assets_path

import omni
import omni.replicator.core as rep
from omni.kit.viewport.utility import get_active_viewport
from isaacsim.core.utils.render_product import set_camera_prim_path
import warp as wp


height = 1920
width = 1080
# 创建我们自己的图像
my_custom_image = np.zeros((height, width, 3), dtype=np.uint8)

# 画一个左红右蓝的图像
my_custom_image[:, :width // 2] = [255, 0, 0]  # Red
my_custom_image[:, width // 2:] = [0, 0, 255]  # Blue

class MHL_Sensor_Example_Scenario():
    def __init__(self):
        self._rob = None
        self._sonar = None
        self._cam = None
        self._DVL = None
        self._baro = None

        self._ctrl_mode = None

        self._running_scenario = False
        self._time = 0.0

    def setup_scenario(self, rob, sonar: ImagingSonarSensor, cam: UW_Camera, DVL, baro, ctrl_mode):
        self._rob = rob
        self._sonar = sonar
        self._cam = cam
        self._DVL = DVL
        self._baro = baro
        self._ctrl_mode = ctrl_mode
        if self._sonar is not None:
            self._sonar.sonar_initialize(include_unlabelled=True)
        if self._cam is not None:
            # self._cam.initialize()
            # uw_cam_yaml_path = get_oceansim_assets_path() + "/render_shenzhen_vague/render_param_0.yaml"
            uw_cam_yaml_path = get_oceansim_assets_path() + "/render_shenzhen_clear/render_param_0.yaml"
            self._cam.initialize(UW_yaml_path=uw_cam_yaml_path)
        if self._DVL is not None:
            self._DVL_reading = [0.0, 0.0, 0.0]
        if self._baro is not None:
            self._baro_reading = 101325.0 # atmospheric pressure (Pa)
        
        
        # Apply the physx force schema if manual control
        if ctrl_mode == "Manual control":
            from ...utils.keyboard_cmd import keyboard_cmd

            self._rob_forceAPI = PhysxSchema.PhysxForceAPI.Apply(self._rob)
            force_value = 2.0
            torque_value = 2.0
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
        
        
        
        ########################################################################3
        CAMERA_STAGE_PATH = "/World/rob/UW_camera"
        # grab our render product and directly set the camera prim
        # 1. 定义你想要的分辨率
        desired_width = 1920
        desired_height = 1080

        # 2. 获取当前视口并设置它的分辨率
        viewport = get_active_viewport()
        viewport.set_texture_resolution((desired_width, desired_height))
        
        render_product_path = viewport.get_render_product_path()
        set_camera_prim_path(render_product_path, CAMERA_STAGE_PATH)

        # GPU Noise Kernel for illustrative purposes, input is rgba, outputs rgb
        def generate_and_replace_image(data_in: np.ndarray, my_custom_image):
            """
            这个函数接受输入数据，但忽略其内容，只使用其形状。
            然后创建一个全新的图像并返回它。
            """
            # 从输入数据中获取正确的尺寸，自动解决分辨率问题
            height, width, _ = data_in.shape
            
            print(f"INFO: Replacing renderer output with custom {width}x{height} image.")

            # # 创建我们自己的图像
            # my_custom_image = np.zeros((height, width, 3), dtype=np.uint8)
            
            # # 画一个左红右蓝的图像
            # my_custom_image[:, :width // 2] = [255, 0, 0]  # Red
            # my_custom_image[:, width // 2:] = [0, 0, 255]  # Blue
            # global my_custom_image
            
            # 返回这个全新的图像
            return my_custom_image

        # 1. 【核心】创建一个可调用的数据提供者类
        
        def get_image_from_another_source():
            # 这里可以是任何复杂的逻辑
            # 为了演示，我们返回一个固定的图像
            print("INFO: 'get_image_from_another_source' is being called...")
            image = np.full((480, 640, 3), [100, 200, 50], dtype=np.uint8) # 一个橄榄绿的图像
            return image
        
        class MyDataProvider:
            """
            这个类是一个数据容器和提供者。
            它的实例可以像函数一样被调用，并返回它持有的数据。
            """
            def __init__(self):
                """
                在构造函数中，我们调用其他函数来获取并存储图像数据。
                这就解决了“无法访问”的问题。
                """
                self.image_data = get_image_from_another_source()

            def __call__(self):
                """
                这使得类的实例可以被调用。
                当Replicator调用它时，它只返回已存储的数据。
                """
                return self.image_data
        
        my_data_provider_instance = MyDataProvider()

        # --- 2. 注册一个新的Annotator，使用我们的替换函数 ---
        rep.annotators.register(
            # 给它一个描述性的新名字
            name="custom_image_replacer",
            annotator=rep.annotators.augment_compose(
                # 源头仍然是渲染器的RGB输出，它为我们提供 data_in 和正确的尺寸
                source_annotator=rep.annotators.get("rgb", device="cpu"), # 注意：使用CPU函数，源最好也在CPU上
                
                augmentations=[
                    rep.annotators.Augmentation.from_function(
                        generate_and_replace_image,
                    ),
                ],
            ),
        )

        # --- 3. 创建Writer，让它使用我们新的Annotator ---
        rep.writers.register_node_writer(
            name=f"CustomROS1PublishImage",
            node_type_id="isaacsim.ros1.bridge.ROS1PublishImage",
            annotators=[
                # ↓↓↓ 让Writer使用我们新的 "custom_image_replacer" ↓↓↓
                "custom_image_replacer",
                
                # 仍然需要时间戳
                omni.syntheticdata.SyntheticData.NodeConnectionTemplate(
                    "IsaacReadSimulationTime", attributes_mapping={"outputs:simulationTime": "inputs:timeStamp"}
                ),
            ],
            category="custom",
        )
        # Register writer for Replicator telemetry tracking
        rep.WriterRegistry._default_writers.append(
            "CustomROS1PublishImage"
        ) if "CustomROS1PublishImage" not in rep.WriterRegistry._default_writers else None

        # Create the new writer and attach to our render product
        writer = rep.writers.get(f"CustomROS1PublishImage")
        writer.initialize(topicName="rgb_augmented", frameId="sim_camera")
        writer.attach([render_product_path])
        
        ########################################################################3
        
        
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

        # clear the keyboard subscription
        if self._ctrl_mode=="Manual control":
            self._force_cmd.cleanup()
            self._torque_cmd.cleanup()

        self._rob = None
        self._sonar = None
        self._cam = None
        self._DVL = None
        self._baro = None
        self._running_scenario = False
        self._time = 0.0


    def update_scenario(self, step: float):

        if not self._running_scenario:
            return
        
        self._time += step
        
        if self._sonar is not None:
            self._sonar.make_sonar_data()
        if self._cam is not None:
            self._cam.render()
        if self._DVL is not None:
            self._DVL_reading = self._DVL.get_linear_vel()
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




        

        


