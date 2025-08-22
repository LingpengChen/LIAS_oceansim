# Copyright (c) 2022-2023, NVIDIA CORPORATION. All rights reserved.
#
# NVIDIA CORPORATION and its licensors retain all intellectual property
# and proprietary rights in and to this software, related documentation
# and any modifications thereto. Any use, reproduction, disclosure or
# distribution of this software and related documentation without an express
# license agreement from NVIDIA CORPORATION is strictly prohibited.
#

import asyncio
import gc
import weakref
import os
import subprocess
import sys

# Setup ROS environment for rospy support
def setup_ros_environment():
    # Add common ROS Python paths to sys.path
    ros_python_paths = [
        '/opt/ros/noetic/lib/python3/dist-packages',
        '/opt/ros/noetic/lib/python3.8/dist-packages',  # for Python 3.8
        '/usr/lib/python3/dist-packages',  # system packages
        '/usr/lib/python3.8/dist-packages',  # system Python 3.8
    ]
    
    paths_added = 0
    for path in ros_python_paths:
        if os.path.exists(path) and path not in sys.path:
            sys.path.insert(0, path)
            paths_added += 1
            print(f"Added to Python path: {path}")
    
    print(f"Added {paths_added} paths to sys.path")
    return True
       

# Setup ROS before importing anything else
setup_ros_environment()

# Test rospy import
try:
    import rospy
    print("✅ ROS environment loaded successfully, rospy available")
    print(f"ROS_MASTER_URI: {os.environ.get('ROS_MASTER_URI', 'Not set')}")
    print(f"ROS_PACKAGE_PATH: {os.environ.get('ROS_PACKAGE_PATH', 'Not set')}")
    print(f"rospy location: {rospy.__file__}")
except ImportError as e:
    print(f"❌ Failed to import rospy: {e}")
    print(f"Current Python path includes:")
    for i, path in enumerate(sys.path[:5]):  # Show first 5 paths
        print(f"  {i}: {path}")
    print("...")
    
    # Check if ROS package directories exist
    ros_dirs = ['/opt/ros/noetic/lib/python3/dist-packages', '/usr/lib/python3/dist-packages']
    for ros_dir in ros_dirs:
        if os.path.exists(ros_dir):
            print(f"📁 {ros_dir} exists")
            rospy_path = os.path.join(ros_dir, 'rospy')
            if os.path.exists(rospy_path):
                print(f"  ✅ rospy found at {rospy_path}")
            else:
                print(f"  ❌ rospy not found in {ros_dir}")
        else:
            print(f"📁 {ros_dir} does not exist")

import omni
import omni.kit.commands
import omni.physx as _physx
import omni.timeline
import omni.ui as ui
import omni.usd
from omni.isaac.ui.element_wrappers import ScrollingWindow
from omni.isaac.ui.menu import MenuItemDescription, make_menu_item_description
from omni.kit.menu.utils import add_menu_items, remove_menu_items
from omni.usd import StageEventType

from .global_variables import EXTENSION_TITLE
from .ui_builder import UIBuilder

"""
This file serves as a basic template for the standard boilerplate operations
that make a UI-based extension appear on the toolbar.

This implementation is meant to cover most use-cases without modification.
Various callbacks are hooked up to a seperate class UIBuilder in .ui_builder.py
Most users will be able to make their desired UI extension by interacting solely with
UIBuilder.

This class sets up standard useful callback functions in UIBuilder:
    on_menu_callback: Called when extension is opened
    on_timeline_event: Called when timeline is stopped, paused, or played
    on_physics_step: Called on every physics step
    on_stage_event: Called when stage is opened or closed
    cleanup: Called when resources such as physics subscriptions should be cleaned up
    build_ui: User function that creates the UI they want.
"""


class Extension(omni.ext.IExt):
    def on_startup(self, ext_id: str):
        """Initialize extension and UI elements"""

        self.ext_id = ext_id
        self._usd_context = omni.usd.get_context()

        # Build Window
        self._window = ScrollingWindow(
            title=EXTENSION_TITLE, width=600, height=500, visible=False, dockPreference=ui.DockPreference.LEFT_BOTTOM
        )
        self._window.set_visibility_changed_fn(self._on_window)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.register_action(
            ext_id,
            f"CreateUIExtension:{EXTENSION_TITLE}",
            self._menu_callback,
            description=f"Add {EXTENSION_TITLE} Extension to UI toolbar",
        )


        self._menu_items = [
            MenuItemDescription(
                name="Examples",
                onclick_action=(ext_id, f"CreateUIExtension:{EXTENSION_TITLE}"),
                sub_menu=[
                    make_menu_item_description(
                        ext_id, "Sensor Example", lambda a=weakref.proxy(self): a._menu_callback()
                    )
                ],
            )
        ]

        add_menu_items(self._menu_items, "OceanSim")

        # Filled in with User Functions
        self.ui_builder = UIBuilder()

        # Events
        self._usd_context = omni.usd.get_context()
        self._physxIFace = _physx.acquire_physx_interface()
        self._physx_subscription = None
        self._stage_event_sub = None
        self._timeline = omni.timeline.get_timeline_interface()

    def on_shutdown(self):
        self._models = {}
        remove_menu_items(self._menu_items, EXTENSION_TITLE)

        action_registry = omni.kit.actions.core.get_action_registry()
        action_registry.deregister_action(self.ext_id, f"CreateUIExtension:{EXTENSION_TITLE}")

        if self._window:
            self._window = None
        self.ui_builder.cleanup()
        gc.collect()

    def _on_window(self, visible):
        if self._window.visible:
            # Subscribe to Stage and Timeline Events
            self._usd_context = omni.usd.get_context()
            events = self._usd_context.get_stage_event_stream()
            self._stage_event_sub = events.create_subscription_to_pop(self._on_stage_event)
            stream = self._timeline.get_timeline_event_stream()
            self._timeline_event_sub = stream.create_subscription_to_pop(self._on_timeline_event)

            self._build_ui()
        else:
            self._usd_context = None
            self._stage_event_sub = None
            self._timeline_event_sub = None
            self.ui_builder.cleanup()

    def _build_ui(self):
        with self._window.frame:
            with ui.VStack(spacing=5, height=0):
                self._build_extension_ui()

        async def dock_window():
            await omni.kit.app.get_app().next_update_async()

            def dock(space, name, location, pos=0.5):
                window = omni.ui.Workspace.get_window(name)
                if window and space:
                    window.dock_in(space, location, pos)
                return window

            tgt = ui.Workspace.get_window("Viewport")
            dock(tgt, EXTENSION_TITLE, omni.ui.DockPosition.LEFT, 0.33)
            await omni.kit.app.get_app().next_update_async()

        self._task = asyncio.ensure_future(dock_window())

    #################################################################
    # Functions below this point call user functions
    #################################################################

    def _menu_callback(self):
        self._window.visible = not self._window.visible
        self.ui_builder.on_menu_callback()

    def _on_timeline_event(self, event):
        if event.type == int(omni.timeline.TimelineEventType.PLAY):
            if not self._physx_subscription:
                self._physx_subscription = self._physxIFace.subscribe_physics_step_events(self._on_physics_step)
        elif event.type == int(omni.timeline.TimelineEventType.STOP):
            self._physx_subscription = None

        self.ui_builder.on_timeline_event(event)

    def _on_physics_step(self, step):
        self.ui_builder.on_physics_step(step)

    def _on_stage_event(self, event):
        if event.type == int(StageEventType.OPENED) or event.type == int(StageEventType.CLOSED):
            # stage was opened or closed, cleanup
            self._physx_subscription = None
            self.ui_builder.cleanup()

        self.ui_builder.on_stage_event(event)

    def _build_extension_ui(self):
        # Call user function for building UI
        self.ui_builder.build_ui()
