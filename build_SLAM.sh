#!/bin/bash

cmake -B build
cmake --build build --target main
cmake --build build --target cotangent
