#!/bin/bash

cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build --target simulation -j2
cmake --build build --target drunkard -j2
cmake --build build --target realcolon -j2
