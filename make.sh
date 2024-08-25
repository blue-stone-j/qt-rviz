rm -r build

mkdir -p build

cd build

# cmake .. &&
cmake .. -DCMAKE_INSTALL_PREFIX=./install &&

# make -j5
make install -j5 &&

cd ../install/bin &&

./qt_rviz_node