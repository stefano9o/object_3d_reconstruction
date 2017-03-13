# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 2.8

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

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/parallels/Desktop/Reconstruction3D/Filtering

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/parallels/Desktop/Reconstruction3D/Filtering/build

# Include any dependencies generated for this target.
include CMakeFiles/filtering.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/filtering.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/filtering.dir/flags.make

CMakeFiles/filtering.dir/src/filtroXZ.cpp.o: CMakeFiles/filtering.dir/flags.make
CMakeFiles/filtering.dir/src/filtroXZ.cpp.o: ../src/filtroXZ.cpp
	$(CMAKE_COMMAND) -E cmake_progress_report /home/parallels/Desktop/Reconstruction3D/Filtering/build/CMakeFiles $(CMAKE_PROGRESS_1)
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Building CXX object CMakeFiles/filtering.dir/src/filtroXZ.cpp.o"
	/usr/bin/c++   $(CXX_DEFINES) $(CXX_FLAGS) -o CMakeFiles/filtering.dir/src/filtroXZ.cpp.o -c /home/parallels/Desktop/Reconstruction3D/Filtering/src/filtroXZ.cpp

CMakeFiles/filtering.dir/src/filtroXZ.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/filtering.dir/src/filtroXZ.cpp.i"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -E /home/parallels/Desktop/Reconstruction3D/Filtering/src/filtroXZ.cpp > CMakeFiles/filtering.dir/src/filtroXZ.cpp.i

CMakeFiles/filtering.dir/src/filtroXZ.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/filtering.dir/src/filtroXZ.cpp.s"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_FLAGS) -S /home/parallels/Desktop/Reconstruction3D/Filtering/src/filtroXZ.cpp -o CMakeFiles/filtering.dir/src/filtroXZ.cpp.s

CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.requires:
.PHONY : CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.requires

CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.provides: CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.requires
	$(MAKE) -f CMakeFiles/filtering.dir/build.make CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.provides.build
.PHONY : CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.provides

CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.provides.build: CMakeFiles/filtering.dir/src/filtroXZ.cpp.o

# Object files for target filtering
filtering_OBJECTS = \
"CMakeFiles/filtering.dir/src/filtroXZ.cpp.o"

# External object files for target filtering
filtering_EXTERNAL_OBJECTS =

filtering: CMakeFiles/filtering.dir/src/filtroXZ.cpp.o
filtering: /usr/lib/libboost_system-mt.so
filtering: /usr/lib/libboost_filesystem-mt.so
filtering: /usr/lib/libboost_thread-mt.so
filtering: /usr/lib/libboost_date_time-mt.so
filtering: /usr/lib/libboost_iostreams-mt.so
filtering: /usr/lib/libboost_serialization-mt.so
filtering: /usr/lib/libpcl_common.so
filtering: /usr/lib/libflann_cpp_s.a
filtering: /usr/lib/libpcl_kdtree.so
filtering: /usr/lib/libpcl_octree.so
filtering: /usr/lib/libpcl_search.so
filtering: /usr/lib/libpcl_sample_consensus.so
filtering: /usr/lib/libpcl_filters.so
filtering: /usr/lib/libpcl_features.so
filtering: /usr/lib/libpcl_registration.so
filtering: /usr/lib/libOpenNI.so
filtering: /usr/lib/libvtkCommon.so.5.8.0
filtering: /usr/lib/libvtkRendering.so.5.8.0
filtering: /usr/lib/libvtkHybrid.so.5.8.0
filtering: /usr/lib/libvtkCharts.so.5.8.0
filtering: /usr/lib/libpcl_io.so
filtering: /usr/lib/libpcl_visualization.so
filtering: /usr/lib/libpcl_segmentation.so
filtering: /usr/lib/libpcl_people.so
filtering: /usr/lib/libqhull.so
filtering: /usr/lib/libpcl_surface.so
filtering: /usr/lib/libpcl_recognition.so
filtering: /usr/lib/libpcl_keypoints.so
filtering: /usr/lib/libpcl_outofcore.so
filtering: /usr/lib/libpcl_tracking.so
filtering: /usr/lib/libpcl_apps.so
filtering: /usr/lib/libboost_system-mt.so
filtering: /usr/lib/libboost_filesystem-mt.so
filtering: /usr/lib/libboost_thread-mt.so
filtering: /usr/lib/libboost_date_time-mt.so
filtering: /usr/lib/libboost_iostreams-mt.so
filtering: /usr/lib/libboost_serialization-mt.so
filtering: /usr/lib/libqhull.so
filtering: /usr/lib/libOpenNI.so
filtering: /usr/lib/libflann_cpp_s.a
filtering: /usr/lib/libvtkCommon.so.5.8.0
filtering: /usr/lib/libvtkRendering.so.5.8.0
filtering: /usr/lib/libvtkHybrid.so.5.8.0
filtering: /usr/lib/libvtkCharts.so.5.8.0
filtering: /usr/lib/libpcl_common.so
filtering: /usr/lib/libpcl_kdtree.so
filtering: /usr/lib/libpcl_octree.so
filtering: /usr/lib/libpcl_search.so
filtering: /usr/lib/libpcl_sample_consensus.so
filtering: /usr/lib/libpcl_filters.so
filtering: /usr/lib/libpcl_features.so
filtering: /usr/lib/libpcl_registration.so
filtering: /usr/lib/libpcl_io.so
filtering: /usr/lib/libpcl_visualization.so
filtering: /usr/lib/libpcl_segmentation.so
filtering: /usr/lib/libpcl_people.so
filtering: /usr/lib/libpcl_surface.so
filtering: /usr/lib/libpcl_recognition.so
filtering: /usr/lib/libpcl_keypoints.so
filtering: /usr/lib/libpcl_outofcore.so
filtering: /usr/lib/libpcl_tracking.so
filtering: /usr/lib/libpcl_apps.so
filtering: /usr/lib/libvtkViews.so.5.8.0
filtering: /usr/lib/libvtkInfovis.so.5.8.0
filtering: /usr/lib/libvtkWidgets.so.5.8.0
filtering: /usr/lib/libvtkHybrid.so.5.8.0
filtering: /usr/lib/libvtkParallel.so.5.8.0
filtering: /usr/lib/libvtkVolumeRendering.so.5.8.0
filtering: /usr/lib/libvtkRendering.so.5.8.0
filtering: /usr/lib/libvtkGraphics.so.5.8.0
filtering: /usr/lib/libvtkImaging.so.5.8.0
filtering: /usr/lib/libvtkIO.so.5.8.0
filtering: /usr/lib/libvtkFiltering.so.5.8.0
filtering: /usr/lib/libvtkCommon.so.5.8.0
filtering: /usr/lib/libvtksys.so.5.8.0
filtering: CMakeFiles/filtering.dir/build.make
filtering: CMakeFiles/filtering.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --red --bold "Linking CXX executable filtering"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/filtering.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/filtering.dir/build: filtering
.PHONY : CMakeFiles/filtering.dir/build

CMakeFiles/filtering.dir/requires: CMakeFiles/filtering.dir/src/filtroXZ.cpp.o.requires
.PHONY : CMakeFiles/filtering.dir/requires

CMakeFiles/filtering.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/filtering.dir/cmake_clean.cmake
.PHONY : CMakeFiles/filtering.dir/clean

CMakeFiles/filtering.dir/depend:
	cd /home/parallels/Desktop/Reconstruction3D/Filtering/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/parallels/Desktop/Reconstruction3D/Filtering /home/parallels/Desktop/Reconstruction3D/Filtering /home/parallels/Desktop/Reconstruction3D/Filtering/build /home/parallels/Desktop/Reconstruction3D/Filtering/build /home/parallels/Desktop/Reconstruction3D/Filtering/build/CMakeFiles/filtering.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/filtering.dir/depend
