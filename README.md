## Robot Package Template

This is a GitHub template. You can make your own copy by clicking the green "Use this template" button.

It is recommended that you keep the repo/package name the same, but if you do change it, ensure you do a "Find all" using your IDE (or the built-in GitHub IDE by hitting the `.` key) and rename all instances of `my_bot` to whatever your project's name is.

Note that each directory currently has at least one file in it to ensure that git tracks the files (and, consequently, that a fresh clone has direcctories present for CMake to find). These example files can be removed if required (and the directories can be removed if `CMakeLists.txt` is adjusted accordingly).


`sudo apt install ros-humble-xacro`

`sudo apt install ros-humble-twist-mux`

`sudo apt install ros-humble-controller-manager`

`sudo apt install ros-humble-ros2-control`

`sudo apt install ros-humble-ros2-controllers`

`sudo apt install ros-humble-navigation2`

`sudo apt install ros-humble-rtabmap-ros`

`sudo apt install ros-humble-librealsense2*`

`sudo apt install ros-humble-realsense2-*`

# Realsense Drivers
Ref: https://github.com/IntelRealSense/librealsense/blob/master/doc/distribution_linux.md#installing-the-packages

`sudo mkdir -p /etc/apt/keyrings`
`curl -sSf https://librealsense.intel.com/Debian/librealsense.pgp | sudo tee /etc/apt/keyrings/librealsense.pgp > /dev/null`
`echo "deb [signed-by=/etc/apt/keyrings/librealsense.pgp] https://librealsense.intel.com/Debian/apt-repo jammy main" | \`
`sudo tee /etc/apt/sources.list.d/librealsense.list` in prompt
`sudo apt-get update`
`sudo apt-get install librealsense2-dkms`
`sudo apt-get install librealsense2-utils`
`sudo apt-get install librealsense2-dev`
`sudo apt-get install librealsense2-dbg`