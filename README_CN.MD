# PCD 转换为栅格地图并保存为 PGM 和 YAML 文件

此程序旨在将 PCD（点云数据）文件转换为 ROS2 环境中的栅格地图（Occupancy Grid）消息，并将其发布到指定的 ROS2 话题上。此外，程序还支持将栅格地图直接保存为 PGM 图像文件以及对应的 YAML 元数据文件。

## 功能特性

- **点云数据处理**: 使用 PCL 库对输入的 PCD 文件进行处理，包括穿透滤波器、半径滤波器等。
- **栅格地图生成**: 将经过滤波的点云数据转换为栅格地图格式，并可自定义分辨率和点数阈值。
- **文件保存**: 支持将生成的栅格地图保存为 PGM 格式的图像文件，以及包含地图元数据的 YAML 文件。
- **实时发布**: 生成的栅格地图会通过 ROS2 的话题实时发布，供其他 ROS2 节点订阅使用。

## 依赖

- ROS2 (建议使用 Humble 版本)
- PCL 库
- OpenCV

## 安装步骤

1. 克隆仓库到你的 ROS2 工作区：

    ```bash
    git clone https://github.com/yuguangxie/pcd2pgm_ros.git
    cd ~/your_ros2_workspace
    colcon build
    ```

2. 编译工作区：

    ```bash
    colcon build
    ```

## 运行

使用以下命令启动程序：

```bash
ros2 launch pcd2pgm pcd2pgm_launch.py
```

## 配置参数

在 `pcd2pgm_launch.py` 中，你可以配置以下参数：

- `file_directory`：PCD 文件所在的目录及输出文件保存目录。
- `file_name`：PCD 文件名（不带后缀名）。
- `thre_z_min`：Z 轴下限阈值，用于过滤地面等不需要的点。
- `thre_z_max`：Z 轴上限阈值，用于过滤高于设定高度的点。
- `flag_pass_through`：是否启用穿透过滤器，默认启用。
- `thre_radius`：半径滤波器的半径阈值，控制点云稀疏程度。
- `map_resolution`：生成的栅格地图的分辨率（每个栅格的大小）。
- `thres_point_count`：半径滤波器的点数阈值，决定点云密度。
- `map_topic_name`：发布栅格地图的 ROS2 话题名称。

## 输出

程序会生成以下文件并保存至指定目录：

- **PGM 文件**：保存生成的栅格地图图像（如 `map.pgm`）。
- **YAML 文件**：保存栅格地图的元数据（如 `map.yaml`），包含地图分辨率、原点、占用阈值等信息。

## 示例

假设你有一个名为 `test.pcd` 的点云文件，想将其转换为栅格地图并保存。你可以在 `pcd2pgm_launch.py` 中配置如下参数：

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

启动后，程序会生成 `test.pgm` 和 `test.yaml` 文件，并发布到 `/grid_map` 话题上。
