# ros-clang-instrumentation
Towards Static Analysis and Instrumentation for Software using Robot Operating System

This is the code for a class project for CSCI-610, Advanced Program Analysis and Verification, by Prof. William G. J. Halfond, Fall 2016.

The goal of the project is to use the static analysis capabilities of clang to instrument C++ ROS nodes for performance and data-flow analysis across ROS nodes.

## Installation

1. Install Ubuntu 16.04
2. Update the system
   ```
   sudo apt update
   sudo apt upgrade
   ```
3. Install ROS (taken from http://wiki.ros.org/kinetic/Installation/Ubuntu)
   ```
   sudo sh -c 'echo "deb http://packages.ros.org/ros/ubuntu $(lsb_release -sc) main" > /etc/apt/sources.list.d/ros-latest.list'
   sudo apt-key adv --keyserver hkp://ha.pool.sks-keyservers.net:80 --recv-key 421C365BD9FF1F717815A3895523BAEEB01FA116
   sudo apt-get update
   sudo apt-get install ros-kinetic-ros-base
   sudo rosdep init
   rosdep update
   echo "source /opt/ros/kinetic/setup.bash" >> ~/.bashrc
   source ~/.bashrc
   ```
4. Install clang (& fix cmake issues on Ubuntu 16.04, see https://github.com/iovisor/bcc/issues/492)
   ```
   sudo apt-get install -y llvm-3.8-dev libclang-3.8-dev
   sudo mkdir -p /usr/lib/llvm-3.8/share/llvm
   sudo ln -s /usr/share/llvm-3.8/cmake /usr/lib/llvm-3.8/share/llvm/cmake
   sudo sed -i -e '/get_filename_component(LLVM_INSTALL_PREFIX/ {s|^|#|}' -e '/^# Compute the installation prefix/i set(LLVM_INSTALL_PREFIX "/usr/lib/llvm-3.8")' /usr/lib/llvm-3.8/share/llvm/cmake/LLVMConfig.cmake
   sudo sed -i '/_IMPORT_CHECK_TARGETS Polly/ {s|^|#|}' /usr/lib/llvm-3.8/share/llvm/cmake/LLVMExports-relwithdebinfo.cmake
   sudo sed -i '/_IMPORT_CHECK_TARGETS sancov/ {s|^|#|}' /usr/lib/llvm-3.8/share/llvm/cmake/LLVMExports-relwithdebinfo.cmake
   sudo ln -s /usr/lib/x86_64-linux-gnu/libLLVM-3.8.so.1 /usr/lib/llvm-3.8/lib/
   ```
5. Clone this repository
   ```
   git clone https://github.com/whoenig/ros-clang-instrumentation.git
   ```
6. Build clang instrumentation
   ```
   cd ros-clang-instrumentation
   cd llvm-passes
   mkdir build
   cd build
   cmake ..
   make
   ```
