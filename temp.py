import os
import subprocess
import sys

# Setup ROS environment for rospy support
def setup_ros_environment():
    if os.path.exists("/opt/ros/noetic/setup.bash"):
        print("✅ ROS environment found, setting up...")
        
        # Get ROS environment variables
        result = subprocess.run(['bash', '-c', 'source /opt/ros/noetic/setup.bash && env'], 
                              capture_output=True, text=True)
        
        if result.returncode == 0:
            # Set environment variables
            env_vars_set = 0
            for line in result.stdout.strip().split('\n'):
                if '=' in line:
                    key, value = line.split('=', 1)
                    os.environ[key] = value
                    env_vars_set += 1
            print(f"Set {env_vars_set} environment variables")
            
            # Check for PYTHONPATH in ROS environment
            pythonpath = os.environ.get('PYTHONPATH', '')
            if pythonpath:
                print(f"ROS PYTHONPATH: {pythonpath}")
                # Add ROS PYTHONPATH entries to sys.path
                for path in pythonpath.split(':'):
                    if path and os.path.exists(path) and path not in sys.path:
                        sys.path.insert(0, path)
                        print(f"Added from PYTHONPATH: {path}")
            
            # Add common ROS Python paths to sys.path
            ros_python_paths = [
                '/opt/ros/noetic/lib/python3/dist-packages',
                '/opt/ros/noetic/lib/python3.8/dist-packages',  # for Python 3.8
                '/opt/ros/noetic/lib/python3.10/dist-packages', # for Python 3.10
                '/usr/lib/python3/dist-packages',  # system packages
                '/usr/lib/python3.8/dist-packages',  # system Python 3.8
                '/usr/lib/python3.10/dist-packages',  # system Python 3.10
            ]
            
            paths_added = 0
            for path in ros_python_paths:
                if os.path.exists(path) and path not in sys.path:
                    sys.path.insert(0, path)
                    paths_added += 1
                    print(f"Added to Python path: {path}")
            
            print(f"Added {paths_added} paths to sys.path")
            return True
        else:
            print(f"❌ Failed to source ROS environment: {result.stderr}")
            return False
    else:
        print("❌ ROS environment not found at /opt/ros/noetic/setup.bash")
        return False

# Setup ROS before importing anything else
# setup_ros_environment()


# Test rospy import

import rospy
print("✅ ROS environment loaded successfully, rospy available")
