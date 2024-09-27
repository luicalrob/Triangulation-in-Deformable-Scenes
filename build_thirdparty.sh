#!/bin/bash

echo "Configuring and building Thirdparty/g2o ..."

cd Thirdparty/g2o
mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../Sophus

echo "Configuring and building Thirdparty/Sophus ..."

mkdir build
cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4

cd ../../Qhull

echo "Configuring and building Thirdparty/Qhull ..."

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
ctest
sudo make install

cd ../../Nlopt

echo "Configuring and building Thirdparty/Nlopt ..."

cd build
cmake .. -DCMAKE_BUILD_TYPE=Release
make -j4
sudo make install

cd ../../ceres-bin

echo "Configuring and building Thirdparty/Ceres ..."

cmake ../ceres-solver-2.2.0
make -j4
make test
sudo make install

cd ../../../
