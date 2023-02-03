#!/bin/bash

# This script is used to build colcon packages.

for arg in "$@"
do
    case $arg in
        -c|--clean)
        CLEAN=1
        shift
        ;;
        *)
        PKG_NAME=$arg
        shift
        ;;
    esac
done

COLCON_ARGS=""

if [ -n "$PKG_NAME" ]; then
    COLCON_ARGS="$COLCON_ARGS --packages-select $PKG_NAME"
fi

if [ -n "$CLEAN" ]; then
    COLCON_ARGS="$COLCON_ARGS --cmake-clean-first"
fi

export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:~/usr/lib/cmake

colcon build $COLCON_ARGS  --cmake-args\
 -DCMAKE_EXPORT_COMPILE_COMMANDS=ON\
