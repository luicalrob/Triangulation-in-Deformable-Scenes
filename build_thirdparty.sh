#!/bin/bash

echo "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6

cd ../../Pangolin

echo "Configuring and building Thirdparty/Pangolin ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6

cd ../../Qhull

echo "Configuring and building Thirdparty/Qhull ..."

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6
ctest
sudo make install

cd ../../Nlopt

echo "Configuring and building Thirdparty/Nlopt ..."

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j 6
sudo make install

cd ../../Open3D

mkdir build
cd build
cmake -DBUILD_SHARED_LIBS=ON -DCMAKE_INSTALL_PREFIX=${HOME}/open3d_install ..
make install -j 6

cd ../../../
