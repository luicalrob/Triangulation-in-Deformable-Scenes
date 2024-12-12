#!/bin/bash

cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build --target simulation -j4
cmake --build build --target drunkard -j4
