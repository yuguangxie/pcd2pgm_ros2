# PCD to Grid Map Conversion and Saving as PGM and YAML Files

This program converts PCD (Point Cloud Data) files into grid map messages in a ROS2 environment and publishes them on a specified ROS2 topic. Additionally, the program supports saving the grid map directly as a PGM image file and a corresponding YAML metadata file.

## Features

- **Point Cloud Processing**: Utilizes the PCL library to process the input PCD file, including pass-through and radius filters.
- **Grid Map Generation**: Converts the filtered point cloud data into a grid map format with customizable resolution and point count thresholds.
- **File Saving**: Supports saving the generated grid map as a PGM image file and a YAML file containing the map metadata.
- **Real-time Publishing**: The generated grid map is published in real-time via a ROS2 topic for other ROS2 nodes to subscribe to.

## Dependencies

- ROS2 (recommended version: Humble)
- PCL library
- OpenCV

## Installation

1. Clone the repository into your ROS2 workspace:

    ```bash
    git clone https://github.com/yuguangxie/pcd2pgm_ros.git
    cd ~/your_ros2_workspace
    colcon build
    ```

2. Build the workspace:

    ```bash
    colcon build
    ```

## Running the Program

Launch the program with the following command:

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

## Configuration Parameters

You can configure the following parameters in `pcd2pgm_launch.py`:

- `file_directory`: Directory where the PCD file is located and where the output files will be saved.
- `file_name`: Name of the PCD file (without the extension).
- `thre_z_min`: Lower threshold for the Z-axis, used to filter out unwanted points like ground.
- `thre_z_max`: Upper threshold for the Z-axis, used to filter out points above a certain height.
- `flag_pass_through`: Whether to enable the pass-through filter, enabled by default.
- `thre_radius`: Radius threshold for the radius filter, controlling point cloud sparsity.
- `map_resolution`: Resolution of the generated grid map (size of each grid cell).
- `thres_point_count`: Point count threshold for the radius filter, determining point cloud density.
- `map_topic_name`: The ROS2 topic name where the grid map will be published.

## Output

The program will generate the following files and save them to the specified directory:

- **PGM File**: The grid map image file (e.g., `map.pgm`).
- **YAML File**: The metadata file for the grid map (e.g., `map.yaml`), containing map resolution, origin, occupancy thresholds, etc.

## Example

Suppose you have a point cloud file named `test.pcd` that you want to convert into a grid map and save. You can configure the parameters in `pcd2pgm_launch.py` as follows:

```python
file_directory: "/home/user/pcd_files/"
file_name: "test"
thre_z_min: 0.1
thre_z_max: 1.5
flag_pass_through: True
thre_radius: 0.5
map_resolution: 0.05
thres_point_count: 2
map_topic_name: "/grid_map"
```

After launching, the program will generate `test.pgm` and `test.yaml` files and publish the grid map on the `/grid_map` topic.