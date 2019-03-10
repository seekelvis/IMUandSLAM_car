# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.5

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Remove some rules from gmake that .SUFFIXES does not remove.
SUFFIXES =

.SUFFIXES: .hpux_make_needs_suffix_list


# Suppress display of executed commands.
$(VERBOSE).SILENT:


# A target that is always out of date.
cmake_force:

.PHONY : cmake_force

#=============================================================================
# Set environment variables for the build.

# The shell in which to execute make rules.
SHELL = /bin/sh

# The CMake executable.
CMAKE_COMMAND = /usr/bin/cmake

# The command to remove a file.
RM = /usr/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/IMU_Trans/catkin_ws/src

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/IMU_Trans/catkin_ws/build

# Include any dependencies generated for this target.
include usb_cam/CMakeFiles/usb_cam_node.dir/depend.make

# Include the progress variables for this target.
include usb_cam/CMakeFiles/usb_cam_node.dir/progress.make

# Include the compile flags for this target's objects.
include usb_cam/CMakeFiles/usb_cam_node.dir/flags.make

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o: usb_cam/CMakeFiles/usb_cam_node.dir/flags.make
usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o: /home/pi/IMU_Trans/catkin_ws/src/usb_cam/nodes/usb_cam_node.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/IMU_Trans/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o"
	cd /home/pi/IMU_Trans/catkin_ws/build/usb_cam && /usr/bin/c++   $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o -c /home/pi/IMU_Trans/catkin_ws/src/usb_cam/nodes/usb_cam_node.cpp

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.i"
	cd /home/pi/IMU_Trans/catkin_ws/build/usb_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/IMU_Trans/catkin_ws/src/usb_cam/nodes/usb_cam_node.cpp > CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.i

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.s"
	cd /home/pi/IMU_Trans/catkin_ws/build/usb_cam && /usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/IMU_Trans/catkin_ws/src/usb_cam/nodes/usb_cam_node.cpp -o CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.s

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.requires:

.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.requires

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.provides: usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.requires
	$(MAKE) -f usb_cam/CMakeFiles/usb_cam_node.dir/build.make usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.provides.build
.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.provides

usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.provides.build: usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o


# Object files for target usb_cam_node
usb_cam_node_OBJECTS = \
"CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o"

# External object files for target usb_cam_node
usb_cam_node_EXTERNAL_OBJECTS =

/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: usb_cam/CMakeFiles/usb_cam_node.dir/build.make
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /home/pi/IMU_Trans/catkin_ws/devel/lib/libusb_cam.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libimage_transport.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libmessage_filters.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libtinyxml2.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libclass_loader.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/libPocoFoundation.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libdl.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libroslib.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/librospack.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libpython2.7.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_program_options.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libtinyxml.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libcamera_info_manager.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libcamera_calibration_parsers.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libroscpp.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_filesystem.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_signals.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/librosconsole.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/librosconsole_log4cxx.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/librosconsole_backend_interface.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/liblog4cxx.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_regex.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libxmlrpcpp.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libroscpp_serialization.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/librostime.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /opt/ros/kinetic/lib/libcpp_common.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_system.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_thread.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_chrono.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_date_time.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libboost_atomic.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libpthread.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: /usr/lib/arm-linux-gnueabihf/libconsole_bridge.so
/home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node: usb_cam/CMakeFiles/usb_cam_node.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/IMU_Trans/catkin_ws/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable /home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node"
	cd /home/pi/IMU_Trans/catkin_ws/build/usb_cam && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/usb_cam_node.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
usb_cam/CMakeFiles/usb_cam_node.dir/build: /home/pi/IMU_Trans/catkin_ws/devel/lib/usb_cam/usb_cam_node

.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/build

usb_cam/CMakeFiles/usb_cam_node.dir/requires: usb_cam/CMakeFiles/usb_cam_node.dir/nodes/usb_cam_node.cpp.o.requires

.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/requires

usb_cam/CMakeFiles/usb_cam_node.dir/clean:
	cd /home/pi/IMU_Trans/catkin_ws/build/usb_cam && $(CMAKE_COMMAND) -P CMakeFiles/usb_cam_node.dir/cmake_clean.cmake
.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/clean

usb_cam/CMakeFiles/usb_cam_node.dir/depend:
	cd /home/pi/IMU_Trans/catkin_ws/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/IMU_Trans/catkin_ws/src /home/pi/IMU_Trans/catkin_ws/src/usb_cam /home/pi/IMU_Trans/catkin_ws/build /home/pi/IMU_Trans/catkin_ws/build/usb_cam /home/pi/IMU_Trans/catkin_ws/build/usb_cam/CMakeFiles/usb_cam_node.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : usb_cam/CMakeFiles/usb_cam_node.dir/depend

