# Tạo môi trường làm việc cho ROS2
- Cài máy ảo để chạy trên Windows: Oracle VM VirtualBox.
- Cài hệ điều hành
	+ Nếu CPU yếu (nhỏ hơn 2GHz) thì cài Ubuntu 22.04 LTS (hỗ trợ bản ROS2 Humble) => tôi dùng bản này.
	+ Nếu CPU khoẻ (lớn hơn hoặc bằng 2GHz) thì cài Ubuntu 24.04 LTS (hỗ trợ bản ROS2 Jazzy mới nhất).
- Lỗi Terminal không mở được
	+ Bước 1 -> truy cập Settings.
	+ Bước 2 -> chọn Region & Language.
	+ Bước 3 -> thay đổi ngôn ngữ khác và Restart.

- Các lệnh tiếp theo trên Terminal để cài ROS2
	+ `su -` , truy cập root
	+ `usermod -aG sudo <nameuser>` , thêm người dùng với quyền sudo
	+ `reboot` , cập nhật các cài đặt thay đổi
	+ Truy cập https://docs.ros.org/ và làm theo hướng dẫn để cài đặt ROS2 phù hợp

- Cài đặt môi trường Python3 để build packages
	+ `sudo apt-get install python3-pip`
	+ `sudo apt install python3-colcon-common-extensions`

- Cài đặt môi trường luôn chạy source khi mở Terminal
	+ `cd ~/vxtruog_ros2/` -> `colcon build`
	+ `gedit ~/.bashrc`
	+ thêm `source /opt/ros/humble/setup.bash`
	+ thêm `source ~/vxtruog_ros2/install/setup.bash`
	+ thêm `source /usr/share/colcon_argcomplete/hook/colcon-argcomplete.bash`

- Tạo package trong ~/vxtruog_ros2/src/ (chạy python thì tạo my_python_pkg, chạy c++ thì tạo my_cpp_pkg)
	+ `ros2 pkg create my_python_pkg --build-type ament_python --dependencies rclpy`
	+ `ros2 pkg create my_cpp_pkg --build-type ament_cmake --dependencies rclcpp`
	+ `cd ~/vxtruog_ros2/` -> `colcon build`, build tất cả package sau mỗi lần thay đổi.
	+ `colcon build --packages-select <package_name>`, build package chỉ định.

- Cài đặt VS Code -> được tệp .deb -> Open with Software Install -> Install code
- Cài đặt C/C++, CMake trong VS Code


	
