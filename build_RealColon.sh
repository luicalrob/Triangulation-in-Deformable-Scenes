#!/bin/bash

cmake -B build -DCMAKE_BUILD_TYPE=RelWithDebInfo

cmake --build build --target realcolon -j2
