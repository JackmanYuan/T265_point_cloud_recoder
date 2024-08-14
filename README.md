# Visualize RealSense T265 Depth Data with Open3D

This project demonstrates how to use a RealSense T265 camera to capture stereo images and generate a real-time 3D point cloud. The point cloud is visualized using Open3D, and the application allows users to interactively view and save the point cloud data.

## Features

- **Stereo Vision:** Captures stereo images from the RealSense T265 camera.
- **Real-Time 3D Point Cloud:** Generates and visualizes a 3D point cloud in real-time using Open3D.
- **Interactive Controls:**
  - Press `Space` to save the current point cloud as a `.ply` file.
  - Press `s` to switch to stack mode.
  - Press `o` to switch to overlay mode.
  - Press `Esc` to exit the application.

## Getting Started

### Prerequisites

To run this project, you need the following:

- **Intel RealSense T265 Camera**
- **Intel RealSense SDK:** Ensure that the RealSense SDK (`librealsense`) is installed and your T265 camera is connected.
- **Python 3.6+**
- **Required Python Packages:**
  - `pyrealsense2`
  - `opencv-python`
  - `open3d`
  - `numpy`

### Installation

1. **Install the RealSense SDK:**
   Follow the instructions on the [RealSense SDK installation page](https://github.com/IntelRealSense/librealsense/blob/master/doc/installation.md).

2. **Install Python Packages:**
   Install the required Python packages using `pip`:
   ```bash
   pip install pyrealsense2 opencv-python open3d numpy
   ```

3. **Clone the Repository:**
   If you haven't already, clone this repository to your local machine.

### Running the Application

To run the application, execute the Python script:

```bash
python t265_pointcloud_recorder.py
```

Once the application starts, you'll see a visualization window displaying the 3D point cloud generated from the T265 camera's stereo images.

### Keyboard Controls

- **Space:** Save the current point cloud as a `.ply` file.
- **s:** Switch to stack mode for visualizing the point cloud.
- **o:** Switch to overlay mode for visualizing the point cloud.
- **Esc:** Exit the application.

## How It Works

This application captures stereo images from the RealSense T265 camera, computes the disparity between the images, and reprojects the disparity to generate a 3D point cloud. The point cloud is then visualized using Open3D, allowing users to interactively explore the data.


### Based on RealSense Sample Code

This project is based on the sample code from the Intel RealSense repository, specifically the `t265_stereo.py` example. You can find the original sample code [here](https://github.com/IntelRealSense/librealsense/blob/v2.53.1/wrappers/python/examples/t265_stereo.py).


## License

This project is licensed under the MIT License. See the [LICENSE](LICENSE) file for details.

## Acknowledgments

- **Intel RealSense SDK:** For providing the tools to capture stereo images and generate 3D point clouds.
- **Open3D:** For making 3D visualization in Python simple and powerful.
