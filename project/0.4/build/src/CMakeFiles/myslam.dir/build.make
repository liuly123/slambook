# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.14

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
CMAKE_COMMAND = /usr/local/bin/cmake

# The command to remove a file.
RM = /usr/local/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/liuly/slambook/project/0.4

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/liuly/slambook/project/0.4/build

# Include any dependencies generated for this target.
include src/CMakeFiles/myslam.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/myslam.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/myslam.dir/flags.make

src/CMakeFiles/myslam.dir/frame.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/frame.cpp.o: ../src/frame.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/myslam.dir/frame.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/frame.cpp.o -c /home/liuly/slambook/project/0.4/src/frame.cpp

src/CMakeFiles/myslam.dir/frame.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/frame.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/frame.cpp > CMakeFiles/myslam.dir/frame.cpp.i

src/CMakeFiles/myslam.dir/frame.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/frame.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/frame.cpp -o CMakeFiles/myslam.dir/frame.cpp.s

src/CMakeFiles/myslam.dir/mappoint.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/mappoint.cpp.o: ../src/mappoint.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/myslam.dir/mappoint.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/mappoint.cpp.o -c /home/liuly/slambook/project/0.4/src/mappoint.cpp

src/CMakeFiles/myslam.dir/mappoint.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/mappoint.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/mappoint.cpp > CMakeFiles/myslam.dir/mappoint.cpp.i

src/CMakeFiles/myslam.dir/mappoint.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/mappoint.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/mappoint.cpp -o CMakeFiles/myslam.dir/mappoint.cpp.s

src/CMakeFiles/myslam.dir/map.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/map.cpp.o: ../src/map.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/myslam.dir/map.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/map.cpp.o -c /home/liuly/slambook/project/0.4/src/map.cpp

src/CMakeFiles/myslam.dir/map.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/map.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/map.cpp > CMakeFiles/myslam.dir/map.cpp.i

src/CMakeFiles/myslam.dir/map.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/map.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/map.cpp -o CMakeFiles/myslam.dir/map.cpp.s

src/CMakeFiles/myslam.dir/camera.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/camera.cpp.o: ../src/camera.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/myslam.dir/camera.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/camera.cpp.o -c /home/liuly/slambook/project/0.4/src/camera.cpp

src/CMakeFiles/myslam.dir/camera.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/camera.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/camera.cpp > CMakeFiles/myslam.dir/camera.cpp.i

src/CMakeFiles/myslam.dir/camera.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/camera.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/camera.cpp -o CMakeFiles/myslam.dir/camera.cpp.s

src/CMakeFiles/myslam.dir/config.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/config.cpp.o: ../src/config.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/myslam.dir/config.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/config.cpp.o -c /home/liuly/slambook/project/0.4/src/config.cpp

src/CMakeFiles/myslam.dir/config.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/config.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/config.cpp > CMakeFiles/myslam.dir/config.cpp.i

src/CMakeFiles/myslam.dir/config.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/config.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/config.cpp -o CMakeFiles/myslam.dir/config.cpp.s

src/CMakeFiles/myslam.dir/g2o_types.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/g2o_types.cpp.o: ../src/g2o_types.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Building CXX object src/CMakeFiles/myslam.dir/g2o_types.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/g2o_types.cpp.o -c /home/liuly/slambook/project/0.4/src/g2o_types.cpp

src/CMakeFiles/myslam.dir/g2o_types.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/g2o_types.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/g2o_types.cpp > CMakeFiles/myslam.dir/g2o_types.cpp.i

src/CMakeFiles/myslam.dir/g2o_types.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/g2o_types.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/g2o_types.cpp -o CMakeFiles/myslam.dir/g2o_types.cpp.s

src/CMakeFiles/myslam.dir/visual_odometry.cpp.o: src/CMakeFiles/myslam.dir/flags.make
src/CMakeFiles/myslam.dir/visual_odometry.cpp.o: ../src/visual_odometry.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_7) "Building CXX object src/CMakeFiles/myslam.dir/visual_odometry.cpp.o"
	cd /home/liuly/slambook/project/0.4/build/src && g++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/myslam.dir/visual_odometry.cpp.o -c /home/liuly/slambook/project/0.4/src/visual_odometry.cpp

src/CMakeFiles/myslam.dir/visual_odometry.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/myslam.dir/visual_odometry.cpp.i"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/liuly/slambook/project/0.4/src/visual_odometry.cpp > CMakeFiles/myslam.dir/visual_odometry.cpp.i

src/CMakeFiles/myslam.dir/visual_odometry.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/myslam.dir/visual_odometry.cpp.s"
	cd /home/liuly/slambook/project/0.4/build/src && g++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/liuly/slambook/project/0.4/src/visual_odometry.cpp -o CMakeFiles/myslam.dir/visual_odometry.cpp.s

# Object files for target myslam
myslam_OBJECTS = \
"CMakeFiles/myslam.dir/frame.cpp.o" \
"CMakeFiles/myslam.dir/mappoint.cpp.o" \
"CMakeFiles/myslam.dir/map.cpp.o" \
"CMakeFiles/myslam.dir/camera.cpp.o" \
"CMakeFiles/myslam.dir/config.cpp.o" \
"CMakeFiles/myslam.dir/g2o_types.cpp.o" \
"CMakeFiles/myslam.dir/visual_odometry.cpp.o"

# External object files for target myslam
myslam_EXTERNAL_OBJECTS =

../lib/libmyslam.so: src/CMakeFiles/myslam.dir/frame.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/mappoint.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/map.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/camera.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/config.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/g2o_types.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/visual_odometry.cpp.o
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/build.make
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stitching3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_superres3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videostab3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_aruco3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bgsegm3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_bioinspired3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ccalib3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_cvv3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dpm3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_face3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_fuzzy3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_hdf3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_img_hash3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_line_descriptor3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_optflow3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_reg3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_rgbd3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_saliency3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_stereo3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_structured_light3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_surface_matching3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_tracking3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xfeatures2d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ximgproc3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xobjdetect3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_xphoto3.so.3.3.1
../lib/libmyslam.so: /home/liuly/slambook/3rdparty/Sophus/build/libSophus.so
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_shape3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_photo3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_datasets3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_plot3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_text3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_dnn3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_ml3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_video3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_calib3d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_features2d3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_highgui3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_videoio3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_viz3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_phase_unwrapping3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_flann3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgcodecs3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_objdetect3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_imgproc3.so.3.3.1
../lib/libmyslam.so: /opt/ros/kinetic/lib/x86_64-linux-gnu/libopencv_core3.so.3.3.1
../lib/libmyslam.so: src/CMakeFiles/myslam.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/liuly/slambook/project/0.4/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_8) "Linking CXX shared library ../../lib/libmyslam.so"
	cd /home/liuly/slambook/project/0.4/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/myslam.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/myslam.dir/build: ../lib/libmyslam.so

.PHONY : src/CMakeFiles/myslam.dir/build

src/CMakeFiles/myslam.dir/clean:
	cd /home/liuly/slambook/project/0.4/build/src && $(CMAKE_COMMAND) -P CMakeFiles/myslam.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/myslam.dir/clean

src/CMakeFiles/myslam.dir/depend:
	cd /home/liuly/slambook/project/0.4/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/liuly/slambook/project/0.4 /home/liuly/slambook/project/0.4/src /home/liuly/slambook/project/0.4/build /home/liuly/slambook/project/0.4/build/src /home/liuly/slambook/project/0.4/build/src/CMakeFiles/myslam.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/myslam.dir/depend

