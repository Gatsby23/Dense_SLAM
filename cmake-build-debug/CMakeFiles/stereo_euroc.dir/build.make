# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.13

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
CMAKE_COMMAND = /home/robotics/Downloads/clion-2018.3.4/bin/cmake/linux/bin/cmake

# The command to remove a file.
RM = /home/robotics/Downloads/clion-2018.3.4/bin/cmake/linux/bin/cmake -E remove -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/robotics/SLAM_Study/Dense_SLAM

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug

# Include any dependencies generated for this target.
include CMakeFiles/stereo_euroc.dir/depend.make

# Include the progress variables for this target.
include CMakeFiles/stereo_euroc.dir/progress.make

# Include the compile flags for this target's objects.
include CMakeFiles/stereo_euroc.dir/flags.make

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o: CMakeFiles/stereo_euroc.dir/flags.make
CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o: ../Examples/Stereo/stereo_euroc.cc
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o"
	/usr/bin/c++  $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o -c /home/robotics/SLAM_Study/Dense_SLAM/Examples/Stereo/stereo_euroc.cc

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/robotics/SLAM_Study/Dense_SLAM/Examples/Stereo/stereo_euroc.cc > CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.i

CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s"
	/usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/robotics/SLAM_Study/Dense_SLAM/Examples/Stereo/stereo_euroc.cc -o CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.s

# Object files for target stereo_euroc
stereo_euroc_OBJECTS = \
"CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o"

# External object files for target stereo_euroc
stereo_euroc_EXTERNAL_OBJECTS =

../Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/Examples/Stereo/stereo_euroc.cc.o
../Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/build.make
../Examples/Stereo/stereo_euroc: ../lib/libORB_SLAM2.so
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_videostab.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_ts.a
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_superres.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_stitching.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_contrib.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_nonfree.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_ocl.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_gpu.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_photo.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_objdetect.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_legacy.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_video.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_ml.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_calib3d.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_features2d.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_highgui.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_imgproc.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_flann.so.2.4.11
../Examples/Stereo/stereo_euroc: /usr/local/opencv2.4.11/lib/libopencv_core.so.2.4.11
../Examples/Stereo/stereo_euroc: /home/robotics/Downloads/Pangolin/build/src/libpangolin.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGLEW.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkDomainsChemistry-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersFlowPaths-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersGeneric-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersHyperTree-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersParallelImaging-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersProgrammable-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersSMP-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersSelection-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersTexture-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersVerdict-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkverdict-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkGeovisCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkproj4-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOAMR-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersAMR-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOEnSight-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOExodus-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOExport-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingGL2PS-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingContextOpenGL-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkgl2ps-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOImport-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOInfovis-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtklibxml2-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOLSDyna-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOMINC-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOMovie-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkoggtheora-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOPLY-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOParallel-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersParallel-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkexoIIc-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIONetCDF-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkNetCDF_cxx-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkNetCDF-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkhdf5_hl-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkhdf5-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkjsoncpp-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOParallelXML-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkParallelCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOSQL-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtksqlite-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOVideo-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingMath-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingMorphological-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingStatistics-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingStencil-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkInteractionImage-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingImage-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingLIC-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOXML-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOGeometry-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOXMLParser-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkexpat-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOLegacy-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingLOD-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingVolumeOpenGL-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingOpenGL-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGLU.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libSM.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libICE.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libX11.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libXext.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libXt.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkViewsContext2D-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkViewsInfovis-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkChartsCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingContext2D-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersImaging-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkInfovisLayout-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkInfovisCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkViewsCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkInteractionWidgets-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersHybrid-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingGeneral-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingSources-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersModeling-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingHybrid-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOImage-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkDICOMParser-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkIOCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkmetaio-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkpng-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtktiff-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkjpeg-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkInteractionStyle-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingAnnotation-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingColor-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingVolume-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingLabel-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingFreeType-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkRenderingCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonColor-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersExtraction-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersStatistics-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingFourier-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkImagingCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkalglib-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersGeometry-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersSources-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersGeneral-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkFiltersCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonExecutionModel-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonComputationalGeometry-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonDataModel-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonMisc-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonTransforms-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonMath-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonSystem-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkCommonCore-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtksys-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkftgl-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkfreetype-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/local/lib/libvtkzlib-6.3.so.1
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libGL.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_system.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_common.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_octree.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_kdtree.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_search.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_sample_consensus.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_filters.so
../Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI.so
../Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI2.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_io.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_features.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_keypoints.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_tracking.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_ml.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_segmentation.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_visualization.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libqhull.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_surface.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_registration.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_recognition.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_stereo.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_people.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_outofcore.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_system.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_filesystem.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_thread.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_date_time.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_iostreams.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_serialization.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_chrono.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_atomic.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libboost_regex.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_common.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_octree.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libflann_cpp_s.a
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_kdtree.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_search.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_sample_consensus.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_filters.so
../Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI.so
../Examples/Stereo/stereo_euroc: /usr/lib/libOpenNI2.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_io.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_features.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_keypoints.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_tracking.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_ml.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_segmentation.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_visualization.so
../Examples/Stereo/stereo_euroc: /usr/lib/x86_64-linux-gnu/libqhull.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_surface.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_registration.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_recognition.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_stereo.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_people.so
../Examples/Stereo/stereo_euroc: /usr/local/lib/libpcl_outofcore.so
../Examples/Stereo/stereo_euroc: ../Thirdparty/DBoW2/lib/libDBoW2.so
../Examples/Stereo/stereo_euroc: ../Thirdparty/g2o/lib/libg2o.so
../Examples/Stereo/stereo_euroc: CMakeFiles/stereo_euroc.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Linking CXX executable ../Examples/Stereo/stereo_euroc"
	$(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/stereo_euroc.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
CMakeFiles/stereo_euroc.dir/build: ../Examples/Stereo/stereo_euroc

.PHONY : CMakeFiles/stereo_euroc.dir/build

CMakeFiles/stereo_euroc.dir/clean:
	$(CMAKE_COMMAND) -P CMakeFiles/stereo_euroc.dir/cmake_clean.cmake
.PHONY : CMakeFiles/stereo_euroc.dir/clean

CMakeFiles/stereo_euroc.dir/depend:
	cd /home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/robotics/SLAM_Study/Dense_SLAM /home/robotics/SLAM_Study/Dense_SLAM /home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug /home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug /home/robotics/SLAM_Study/Dense_SLAM/cmake-build-debug/CMakeFiles/stereo_euroc.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : CMakeFiles/stereo_euroc.dir/depend
