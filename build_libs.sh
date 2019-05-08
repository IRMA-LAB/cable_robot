#!/bin/bash

# Move to cable_robot libs directory
cd "$(dirname "$0")/lib"

## Build state_machine library
cd state_machine
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building state_machine library in $PWD..."
qmake ../state_machine.pro -spec linux-g++-64
make
cd ../..

## Build GRAB common libraries
cd grab_common

# Build numeric library
cd libnumeric
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building numeric library in $PWD..."
qmake ../numeric.pro -spec linux-g++-64
make
cd ../..

# Build geometric library
cd libgeom
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building geometric library in $PWD..."
qmake ../geom.pro -spec linux-g++-64
make
cd ../..

## Build CDPR library
cd libcdpr
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building CDPR library in $PWD..."
qmake ../cdpr.pro -spec linux-g++-64
make
cd ../..

# Build GRAB real-time library
cd libgrabrt
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building GRAB real-time library in $PWD..."
qmake ../grabrt.pro -spec linux-g++-64
make
cd ../..

# Build GRAB EtherCAT library
cd libgrabec
if [ ! -d lib ]
  then
    mkdir lib
fi
cd lib
echo "Building GRAB EtherCAT library in $PWD..."
qmake ../grabec.pro -spec linux-g++-64
make
