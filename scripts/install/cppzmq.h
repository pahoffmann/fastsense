#!/usr/bin/env bash

set -e

git clone https://github.com/zeromq/libzmq.git && cd libzmq
mkdir build && cd build
cmake .. && sudo make -j4 install
cd ../..

git clone https://github.com/zeromq/cppzmq.git && cd cppzmq 
mkdir build && cd build
cmake -DCPPZMQ_BUILD_TESTS=OFF .. && sudo make -j4 install
cd ../..

rm -rf libzmq cppzmq



