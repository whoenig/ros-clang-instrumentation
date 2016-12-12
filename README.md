# ros-clang-instrumentation
Towards Static Analysis and Instrumentation for Software using Robot Operating System

This is the code for a class project for CSCI-610, Advanced Program Analysis and Verification, by Prof. William G. J. Halfond, Fall 2016.

The goal of the project is to use the static analysis capabilities of clang to instrument C++ ROS nodes for performance and data-flow analysis across ROS nodes.

## Installation

1. Install Ubuntu 16.04
2. Update the system & install dependencies

   ```
   sudo apt update
   sudo apt upgrade
   sudo apt install git libyaml-cpp-dev
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
   sudo apt install -y llvm-3.8-dev libclang-3.8-dev clang
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

## Testing

There are three test cases provided.

### Test1

This test is the default publisher/subscriber examples (see http://wiki.ros.org/ROS/Tutorials/WritingPublisherSubscriber(c%2B%2B) ).

1. Building the example with instrumentation

   ```
   cd ros-clang-instrumentation/ros_ws
   ROS_INSTRUMENTATION_CONFIG_FILE=${PWD}/src/test1/test/config.yaml ./catkin_make_with_instrumentation.sh
   source devel/setup.bash
   ```

2. Running the example

   ```
   roslaunch test1 test.launch
   ```

   Use Ctrl+C to exit the example.

3. Viewing the results

   ```
   rosrun ros_instrumentation results.py src/test1/test/config.yaml test1
   ```

4. Expected output

   ```
   Topic: chatter
     Latencies [us]:
       [[ 276.]
        [ 177.]
        [ 357.]
        [ 186.]
        [ 195.]
        [ 195.]]
     Iteration 0:
       talker.cpp:24
       talker.cpp:36
       talker.cpp:43
       talker.cpp:62
       talker.cpp:70
       talker.cpp:98
       talker.cpp:101
       talker.cpp:102
       talker.cpp:102

       Latency: 276.0 us
       chatterCallback(boost::shared_ptr<std_msgs::String_<std::allocator<void> > const> const&)

   ```

   Where the latencies should be the same order of magnitude (below 1000 us) and the path should match exactly for each iteration.


### Test2

This test uses one publisher and one subscriber in the same executable, testing the non-networking code path of the ROS message passing.

1. Building the example with instrumentation

   ```
   cd ros-clang-instrumentation/ros_ws
   ROS_INSTRUMENTATION_CONFIG_FILE=${PWD}/src/test2/test/config.yaml ./catkin_make_with_instrumentation.sh
   source devel/setup.bash
   ```

2. Running the example

   ```
   roslaunch test2 test.launch
   ```

   Use Ctrl+C to exit the example.

3. Viewing the results

   ```
   rosrun ros_instrumentation results.py src/test2/test/config.yaml test2
   ```

4. Expected output

   ```
   Topic: chatter
     Latencies [us]:
       [[ 100199.]
        [ 100089.]
        [ 100097.]
        [ 102300.]
        [ 100040.]
        [ 100173.]
        [ 100016.]
        [ 100248.]]
     Iteration 0:
       talkerAndListener.cpp:14
       talkerAndListener.cpp:26
       talkerAndListener.cpp:33
       talkerAndListener.cpp:52
       talkerAndListener.cpp:53
       talkerAndListener.cpp:61
       talkerAndListener.cpp:88
       talkerAndListener.cpp:91
       talkerAndListener.cpp:92
       talkerAndListener.cpp:92

       Latency: 100199.0 us
       chatterCallback(boost::shared_ptr<std_msgs::String_<std::allocator<void> > const> const&)

   ```

   Where the latencies should be the same order of magnitude (about 100000 us) and the path should match exactly for each iteration. The latency is high in this case because `ros::spinOnce()` is only called every 100ms (and the subscriber callbacks are not threaded in roscpp).

### Test3

This test uses one publisher, a repeater, and one subscriber, creating a pipeline of 3 ROS nodes. The repeater has two different code paths, one being slow and one being fast, emulated by the following code:

```c++
if (counter % 2 == 0) {
  // emulate slow computation
  std::this_thread::sleep_for(std::chrono::milliseconds(50));
} else {
  std::this_thread::sleep_for(std::chrono::milliseconds(1));
}
```

1. Building the example with instrumentation

   ```
   cd ros-clang-instrumentation/ros_ws
   ROS_INSTRUMENTATION_CONFIG_FILE=${PWD}/src/test3/test/config.yaml ./catkin_make_with_instrumentation.sh
   source devel/setup.bash
   ```

2. Running the example

   ```
   roslaunch test3 test.launch
   ```

   Use Ctrl+C to exit the example.

3. Viewing the results

   ```
   rosrun ros_instrumentation results.py src/test3/test/config.yaml test3
   ```

4. Expected output

   ```
   Topic: chatter2
     Latencies [us]:
       [[   284.  50244.    263.]
        [   177.   1119.    106.]
        [   274.  50165.    188.]
        [   226.   1112.    101.]]
     Iteration 0:
       talker.cpp:7
       talker.cpp:9
       talker.cpp:11
       talker.cpp:12
       talker.cpp:13
       talker.cpp:17
       talker.cpp:40
       talker.cpp:43

       Latency: 284.0 us
       chatter2Callback(boost::shared_ptr<geometry_msgs::Point_<std::allocator<void> > const> const&)

       Latency: 50244.0 us
       repeater.cpp:9
       repeater.cpp:14
       repeater.cpp:20

       Latency: 263.0 us
       chatter3Callback(boost::shared_ptr<geometry_msgs::Vector3_<std::allocator<void> > const> const&)

     Iteration 1:
       talker.cpp:7
       talker.cpp:9
       talker.cpp:11
       talker.cpp:12
       talker.cpp:13
       talker.cpp:17
       talker.cpp:40
       talker.cpp:43

       Latency: 177.0 us
       chatter2Callback(boost::shared_ptr<geometry_msgs::Point_<std::allocator<void> > const> const&)

       Latency: 1119.0 us
       repeater.cpp:9
       repeater.cpp:16
       repeater.cpp:20

       Latency: 106.0 us
       chatter3Callback(boost::shared_ptr<geometry_msgs::Vector3_<std::allocator<void> > const> const&)

   ```

   Where the latencies should be the same order of magnitude. The middle column for the repeater should alternate in each iteration between about 50000 us and 1000 us. The path for each iteration should change to either include `repeater.cpp:14` or `repeater.cpp:16`.


## Limitations

* Only ROS C++ nodes are supported (or any other language which can be compiled using LLVM).
* The path computation uses strongly connected components (SCC) and the Ball Larus path profiling approach. Loops will be part of a single SCC.
* The path profiler ignores exception-based control-flow.
