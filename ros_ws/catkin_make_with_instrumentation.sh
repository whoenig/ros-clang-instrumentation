# export ROS_INSTRUMENTATION_CONFIG_FILE=${PWD}/src/test3/test/config.yaml
rm -rf build/
catkin_make -DCMAKE_BUILD_TYPE=RelWithDebInfo -DCMAKE_C_COMPILER_WORKS=1 -DCMAKE_CXX_COMPILER_WORKS=1 -DCMAKE_CXX_COMPILER=clang++ -DCMAKE_C_COMPILER=clang -DCMAKE_CXX_FLAGS="-Xclang -load -Xclang ${PWD}/../llvm-passes/build/ros/librosPass.so"
