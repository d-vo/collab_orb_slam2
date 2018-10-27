echo "Building ROS nodes"

cd Examples/ROS/compression
mkdir build
cd build
cmake .. -DROS_BUILD_TYPE=Release
make -j 2
