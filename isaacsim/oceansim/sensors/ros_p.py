
        ########################################################################3
        
        # import warp as wp

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
        def generate_and_replace_image(data_in: np.ndarray):
            """
            这个函数接受输入数据，但忽略其内容，只使用其形状。
            然后创建一个全新的图像并返回它。
            """
            # 从输入数据中获取正确的尺寸，自动解决分辨率问题
            height, width, _ = data_in.shape
            
            print(f"INFO: Replacing renderer output with custom {width}x{height} image.")

            # 创建我们自己的图像
            my_custom_image = np.zeros((height, width, 3), dtype=np.uint8)
            
            # 画一个左红右蓝的图像
            my_custom_image[:, :width // 2] = [255, 0, 0]  # Red
            my_custom_image[:, width // 2:] = [0, 0, 255]  # Blue
            
            # 返回这个全新的图像
            return my_custom_image


        # --- 2. 注册一个新的Annotator，使用我们的替换函数 ---
        rep.annotators.register(
            # 给它一个描述性的新名字
            name="custom_image_replacer",
            annotator=rep.annotators.augment_compose(
                # 源头仍然是渲染器的RGB输出，它为我们提供 data_in 和正确的尺寸
                source_annotator=rep.annotators.get("rgb", device="cpu"), # 注意：使用CPU函数，源最好也在CPU上
                
                augmentations=[
                    # 使用我们的新函数作为增强步骤
                    rep.annotators.Augmentation.from_function(
                        generate_and_replace_image
                        # self.render_ros
                        # 注意：这里不需要 sigma, seed, data_out_shape 等Warp专用参数
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
    