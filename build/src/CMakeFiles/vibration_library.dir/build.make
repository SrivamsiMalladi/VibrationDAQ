# CMAKE generated file: DO NOT EDIT!
# Generated by "Unix Makefiles" Generator, CMake Version 3.18

# Delete rule output on recipe failure.
.DELETE_ON_ERROR:


#=============================================================================
# Special targets provided by cmake.

# Disable implicit rules so canonical targets will work.
.SUFFIXES:


# Disable VCS-based implicit rules.
% : %,v


# Disable VCS-based implicit rules.
% : RCS/%


# Disable VCS-based implicit rules.
% : RCS/%,v


# Disable VCS-based implicit rules.
% : SCCS/s.%


# Disable VCS-based implicit rules.
% : s.%


.SUFFIXES: .hpux_make_needs_suffix_list


# Command-line flag to silence nested $(MAKE).
$(VERBOSE)MAKESILENT = -s

#Suppress display of executed commands.
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
RM = /usr/bin/cmake -E rm -f

# Escaping for special characters.
EQUALS = =

# The top-level source directory on which CMake was run.
CMAKE_SOURCE_DIR = /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ

# The top-level build directory on which CMake was run.
CMAKE_BINARY_DIR = /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build

# Include any dependencies generated for this target.
include src/CMakeFiles/vibration_library.dir/depend.make

# Include the progress variables for this target.
include src/CMakeFiles/vibration_library.dir/progress.make

# Include the compile flags for this target's objects.
include src/CMakeFiles/vibration_library.dir/flags.make

src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.o: src/CMakeFiles/vibration_library.dir/flags.make
src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.o: ../src/ConfigModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_1) "Building CXX object src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.o"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibration_library.dir/ConfigModule.cpp.o -c /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/ConfigModule.cpp

src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibration_library.dir/ConfigModule.cpp.i"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/ConfigModule.cpp > CMakeFiles/vibration_library.dir/ConfigModule.cpp.i

src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibration_library.dir/ConfigModule.cpp.s"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/ConfigModule.cpp -o CMakeFiles/vibration_library.dir/ConfigModule.cpp.s

src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o: src/CMakeFiles/vibration_library.dir/flags.make
src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o: ../src/VibrationSensorModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_2) "Building CXX object src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o -c /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/VibrationSensorModule.cpp

src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.i"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/VibrationSensorModule.cpp > CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.i

src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.s"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/VibrationSensorModule.cpp -o CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.s

src/CMakeFiles/vibration_library.dir/StorageModule.cpp.o: src/CMakeFiles/vibration_library.dir/flags.make
src/CMakeFiles/vibration_library.dir/StorageModule.cpp.o: ../src/StorageModule.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_3) "Building CXX object src/CMakeFiles/vibration_library.dir/StorageModule.cpp.o"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibration_library.dir/StorageModule.cpp.o -c /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/StorageModule.cpp

src/CMakeFiles/vibration_library.dir/StorageModule.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibration_library.dir/StorageModule.cpp.i"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/StorageModule.cpp > CMakeFiles/vibration_library.dir/StorageModule.cpp.i

src/CMakeFiles/vibration_library.dir/StorageModule.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibration_library.dir/StorageModule.cpp.s"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src/StorageModule.cpp -o CMakeFiles/vibration_library.dir/StorageModule.cpp.s

src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o: src/CMakeFiles/vibration_library.dir/flags.make
src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o: ../lib/loguru/loguru.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_4) "Building CXX object src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o -c /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/loguru/loguru.cpp

src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.i"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/loguru/loguru.cpp > CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.i

src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.s"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/loguru/loguru.cpp -o CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.s

src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o: src/CMakeFiles/vibration_library.dir/flags.make
src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o: ../lib/date/tz.cpp
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_5) "Building CXX object src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -o CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o -c /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/date/tz.cpp

src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.i: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Preprocessing CXX source to CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.i"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -E /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/date/tz.cpp > CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.i

src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.s: cmake_force
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green "Compiling CXX source to assembly CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.s"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && /usr/bin/c++ $(CXX_DEFINES) $(CXX_INCLUDES) $(CXX_FLAGS) -S /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/lib/date/tz.cpp -o CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.s

# Object files for target vibration_library
vibration_library_OBJECTS = \
"CMakeFiles/vibration_library.dir/ConfigModule.cpp.o" \
"CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o" \
"CMakeFiles/vibration_library.dir/StorageModule.cpp.o" \
"CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o" \
"CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o"

# External object files for target vibration_library
vibration_library_EXTERNAL_OBJECTS =

src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/ConfigModule.cpp.o
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/VibrationSensorModule.cpp.o
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/StorageModule.cpp.o
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/__/lib/loguru/loguru.cpp.o
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/__/lib/date/tz.cpp.o
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/build.make
src/libvibration_library.a: src/CMakeFiles/vibration_library.dir/link.txt
	@$(CMAKE_COMMAND) -E cmake_echo_color --switch=$(COLOR) --green --bold --progress-dir=/home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/CMakeFiles --progress-num=$(CMAKE_PROGRESS_6) "Linking CXX static library libvibration_library.a"
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && $(CMAKE_COMMAND) -P CMakeFiles/vibration_library.dir/cmake_clean_target.cmake
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && $(CMAKE_COMMAND) -E cmake_link_script CMakeFiles/vibration_library.dir/link.txt --verbose=$(VERBOSE)

# Rule to build all files generated by this target.
src/CMakeFiles/vibration_library.dir/build: src/libvibration_library.a

.PHONY : src/CMakeFiles/vibration_library.dir/build

src/CMakeFiles/vibration_library.dir/clean:
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src && $(CMAKE_COMMAND) -P CMakeFiles/vibration_library.dir/cmake_clean.cmake
.PHONY : src/CMakeFiles/vibration_library.dir/clean

src/CMakeFiles/vibration_library.dir/depend:
	cd /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build && $(CMAKE_COMMAND) -E cmake_depends "Unix Makefiles" /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/src /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src /home/pi/Documents/SrivamsiMalladi/VibrationDAQ_AE/VibrationDAQ/build/src/CMakeFiles/vibration_library.dir/DependInfo.cmake --color=$(COLOR)
.PHONY : src/CMakeFiles/vibration_library.dir/depend

