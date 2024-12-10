#!/bin/bash

cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build --target simulation -j6
cmake --build build --target Drunkard -j6
