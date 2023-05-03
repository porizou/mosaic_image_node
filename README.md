# mosaic_image_node

## Overview

`mosaic_image_node` is a package that uses ROS2 to apply mosaic processing to the face regions in camera images. This node detects faces in the image and applies a mosaic with the specified mosaic size to the faces.

## Dependencies

- ROS2 (Recommended versions: Foxy, Galactic, Rolling)
- OpenCV (Recommended version: 4.x)
- cv_bridge
- sensor_msgs
- std_msgs

## Installation

### Setting up the workspace


```bash
mkdir -p ~/ros2_ws/src
cd ~/ros2_ws/src
```

Cloning the repository

```bash
git clone https://github.com/your_username/mosaic_image_node.git
```

Move to the root of the workspace and build.

```bash
cd ~/ros2_ws
colcon build --symlink-install
```

# Usage

## Parameters

mosaic_size: Specifies the size of the mosaic. The larger the value, the coarser the mosaic. The default value is 10.

## Running

First, set up the environment in the ROS2 workspace.

```bash
source ~/ros2_ws/install/setup.bash
```

Next, run a node that publishes the camera images. For example, you can use the usb_cam node to publish images from a USB camera.

```bash
ros2 run usb_cam usb_cam_node
```

Run the mosaic_image_node.

```bash
ros2 run mosaic_image_node mosaic_node
```

Now, the mosaic-processed images will be published on the /output_mosaic_image topic. To view them, use an appropriate tool such as rqt_image_view.

# License

This package is licensed under the Apache License 2.0.
