#!/bin/bash

# Check if Qt is installed
if ! test -x /usr/bin/qmake
	then
      	# The Qt library is missing...
      	echo "error: This script requires Qt to be installed."
      	exit 1
fi

# Check Qt version
REQUIRED_VERSION=5.12.1
default_qt_version=$(qmake -v | grep -Po '(?<=Qt version )[^in]+')
version () {
	echo "$@" | gawk -F. '{ printf("%03d%03d%03d\n", $1,$2,$3); }';
}
if [ "$(version "$REQUIRED_VERSION")" -gt "$(version "$default_qt_version")" ]
	then
         echo "error: This application requires at least Qt version $REQUIRED_VERSION! Your default version is $default_qt_version"
         QT_CHOOSER="/usr/lib/x86_64-linux-gnu/qt-default/qtchooser/default.conf"
         echo "After downloading latest version, you can set it by default by replacing the filepath of corresponding .bin file in $QT_CHOOSER"
         exit 1
fi

# Move to cable_robot libs directory
cd "$(dirname "$0")/libs"

## Build state_machine library
cd state_machine
if [ -d lib ]
    then
		cd lib
		echo "Cleaning state_machine library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building state_machine library in $PWD..."
qmake ../state_machine.pro -spec linux-g++-64
make
cd ../..

## Build GRAB common libraries
cd grab_common

# Build numeric library
cd libnumeric
if [ -d lib ]
    then
		cd lib
		echo "Cleaning numeric library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building numeric library in $PWD..."
qmake ../numeric.pro -spec linux-g++-64
make
cd ../..

# Build geometric library
cd libgeom
if [ -d lib ]
    then
		cd lib
		echo "Cleaning geometric library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building geometric library in $PWD..."
qmake ../geom.pro -spec linux-g++-64
make
cd ../..

## Build CDPR library
cd libcdpr
if [ -d lib ]
    then
		cd lib
		echo "Cleaning CDPR library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building CDPR library in $PWD..."
qmake ../cdpr.pro -spec linux-g++-64
make
cd ../..

# Build GRAB real-time library
cd libgrabrt
if [ -d lib ]
    then
		cd lib
		echo "Cleaning GRAB real-time library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building GRAB real-time library in $PWD..."
qmake ../grabrt.pro -spec linux-g++-64
make
cd ../..

# Build GRAB EtherCAT library
cd libgrabec
if [ -d lib ]
    then
		cd lib
		echo "Cleaning GRAB EtherCAT library..."
		make clean
	else
        mkdir lib
        echo "Created static library build folder: $PWD"
        cd lib
fi
echo "Building GRAB EtherCAT library in $PWD..."
qmake ../grabec.pro -spec linux-g++-64
make
