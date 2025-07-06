#!/bin/bash

# -------------------------------------------------------------------------------------------------------
# Bash script for building and installing URDF parser dependencies using Unix Makefiles.
# - Place in an arbitrary temporary directory.
# - Specify the locations for the URDF sources OR indicate that these should be downloaded.
# - Specify the install directory.
# - Run the script (sh ./buildURDF.sh).
# - The install directory will contain subdirectories for all necessary dependencies.
#
# Notes:
# - The script accepts 1 optional argument to override the install directory.
# - This script is configured to build ONLY Release configurations using Unix Makefiles.
# -------------------------------------------------------------------------------------------------------

echo -e "\n------------------------ Installing system dependencies\n"
sudo apt-get update
sudo apt-get install libtinyxml-dev libconsole-bridge-dev liburdfdom-headers-dev

DOWNLOAD=ON

URDF_INSTALL_DIR="$HOME/Documents/chrono/libraries/urdf"

# --- Configuration ---
# BUILDDEBUG is OFF because Unix Makefiles is a single-config generator
# and we force CMAKE_BUILD_TYPE=Release below.
BUILDDEBUG=OFF
BUILDSYSTEM="Unix Makefiles"

if [ ${DOWNLOAD} = OFF ]
then
    TINYXML2_SOURCE_DIR="$HOME/Sources/tinyxml2"
    CONSOLE_BRIDGE_SOURCE_DIR="$HOME/Sources/console_bridge"
    URDFDOM_HEADERS_SOURCE_DIR="$HOME/Sources/urdfdom_headers"
    URDFDOM_SOURCE_DIR="$HOME/Sources/urdfdom"
fi

# ------------------------------------------------------------------------
# Allow overriding installation directory through command line argument

if [ $# -eq 1 ]
then
    URDF_INSTALL_DIR=$1
fi

# ------------------------------------------------------------------------

if [ ${DOWNLOAD} = ON ]
then
    echo "Download sources from GitHub"

    rm -rf download_urdf
    mkdir download_urdf

    echo "  ... tinyxml2"
    # Use a specific tag/commit for tinyxml2 for reproducibility if desired, e.g.:
    # git clone --depth 1 --branch 9.0.0 "https://github.com/leethomason/tinyxml2.git" "download_urdf/tinyxml2"
    git clone "https://github.com/leethomason/tinyxml2.git" "download_urdf/tinyxml2"
    TINYXML2_SOURCE_DIR="download_urdf/tinyxml2"

    echo "  ... console_bridge"
    # Use a specific tag/commit if desired
    git clone "https://github.com/ros/console_bridge.git" "download_urdf/console_bridge"
    CONSOLE_BRIDGE_SOURCE_DIR="download_urdf/console_bridge"

    echo "  ... urdfdom_headers"
     # Use a specific tag/commit if desired
    git clone "https://github.com/ros/urdfdom_headers.git" "download_urdf/urdfdom_headers"
    URDFDOM_HEADERS_SOURCE_DIR="download_urdf/urdfdom_headers"
    
    echo "  ... urdfdom"
     # Use a specific tag/commit if desired
    git clone "https://github.com/ros/urdfdom.git" "download_urdf/urdfdom"
    URDFDOM_SOURCE_DIR="download_urdf/urdfdom"
else
    echo "Using provided source directories"
fi

echo -e "\nSources in:"
echo "  "  ${TINYXML2_SOURCE_DIR}
echo "  "  ${CONSOLE_BRIDGE_SOURCE_DIR}
echo "  "  ${URDFDOM_HEADERS_SOURCE_DIR}
echo "  "  ${URDFDOM_SOURCE_DIR}

# ------------------------------------------------------------------------

rm -rf ${URDF_INSTALL_DIR}
mkdir -p ${URDF_INSTALL_DIR}

# --- tinyxml2 -------------------------------------------------------------
#
# Pass "-fPIC" to allow linking in a shared library.
# Set CMAKE_BUILD_TYPE="Release" as Makefiles is single-config.
#

echo -e "\n------------------------ Configure tinyxml2\n"
rm -rf build_tinyxml2
# Use BUILDSYSTEM variable, ensure CMAKE_BUILD_TYPE is set
cmake -E env CXXFLAGS="-fPIC" cmake -G "${BUILDSYSTEM}" -B build_tinyxml2 -S ${TINYXML2_SOURCE_DIR} \
      -DCMAKE_BUILD_TYPE="Release" \
      -Dtinyxml2_INSTALL_CMAKEDIR:PATH="CMake" \
      -DBUILD_SHARED_LIBS=OFF # Explicitly build static tinyxml2 if needed, adjust if shared needed

echo -e "\n------------------------ Build and install tinyxml2\n"
# No --config needed for build/install with single-config generator
cmake --build build_tinyxml2
cmake --install build_tinyxml2 --prefix ${URDF_INSTALL_DIR}

# --- console_bridge ----------------------------------------------------------------

echo -e "\n------------------------ Configure console_bridge\n"
rm -rf build_console_bridge
# Use BUILDSYSTEM, ensure CMAKE_BUILD_TYPE is set
cmake -G "${BUILDSYSTEM}" -B build_console_bridge -S ${CONSOLE_BRIDGE_SOURCE_DIR}  \
      -DCMAKE_BUILD_TYPE="Release" \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

echo -e "\n------------------------ Build and install console_bridge\n"
# No --config needed for build/install with single-config generator
cmake --build build_console_bridge
cmake --install build_console_bridge --prefix ${URDF_INSTALL_DIR}
# Note: The following Debug build commands won't work correctly with single-config Release
if [ ${BUILDDEBUG} = ON ]
then
    echo "Attempting Debug build/install of console_bridge (will likely just run Release again)..."
    cmake --build build_console_bridge # --config Debug # Ignored/Invalid
    cmake --install build_console_bridge --prefix ${URDF_INSTALL_DIR} # --config Debug # Ignored/Invalid
else
    echo "No Debug build of console_bridge"
fi

# --- urdfdom_headers ---------------------------------------------------------

echo -e "\n------------------------ Configure urdfdom_headers\n"
rm -rf build_urdfdom_headers
# Use BUILDSYSTEM, ensure CMAKE_BUILD_TYPE is set
cmake  -G "${BUILDSYSTEM}" -B build_urdfdom_headers -S ${URDFDOM_HEADERS_SOURCE_DIR}  \
       -DCMAKE_BUILD_TYPE="Release" \
       -DCMAKE_DEBUG_POSTFIX=_d \
       -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd

echo -e "\n------------------------ Build and install urdfdom_headers\n"
# No --config needed for build/install with single-config generator
cmake --build build_urdfdom_headers
cmake --install build_urdfdom_headers --prefix ${URDF_INSTALL_DIR}
# Note: The following Debug build commands won't work correctly with single-config Release
if [ ${BUILDDEBUG} = ON ]
then
    echo "Attempting Debug build/install of urdfdom_headers (will likely just run Release again)..."
    cmake --build build_urdfdom_headers # --config Debug # Ignored/Invalid
    cmake --install build_urdfdom_headers --prefix ${URDF_INSTALL_DIR} # --config Debug # Ignored/Invalid
else
    echo "No Debug build of urdfdom_headers"
fi

# --- urdfdom -----------------------------------------------------------

echo -e "\n------------------------ Configure urdfdom\n"
rm -rf build_urdfdom
# Use BUILDSYSTEM, ensure CMAKE_BUILD_TYPE is set
cmake -G "${BUILDSYSTEM}" -B build_urdfdom -S ${URDFDOM_SOURCE_DIR} \
      -DCMAKE_BUILD_TYPE="Release" \
      -DCMAKE_DEBUG_POSTFIX=_d \
      -DCMAKE_RELWITHDEBINFO_POSTFIX=_rd \
      -Dconsole_bridge_DIR:PATH=${URDF_INSTALL_DIR}/lib/console_bridge/cmake \
      -Durdfdom_headers_DIR:PATH=${URDF_INSTALL_DIR}/lib/urdfdom_headers/cmake \
      -DTINYXML2_INCLUDE_DIR:PATH=${URDF_INSTALL_DIR}/include \
      -DTinyXML2_DIR:PATH=${URDF_INSTALL_DIR}/CMake  
      
echo -e "\n------------------------ Build and install urdfdom\n"
# No --config needed for build/install with single-config generator
cmake --build build_urdfdom
cmake --install build_urdfdom --prefix ${URDF_INSTALL_DIR}
# Note: The following Debug build commands won't work correctly with single-config Release
if [ ${BUILDDEBUG} = ON ]
then
    echo "Attempting Debug build/install of urdfdom (will likely just run Release again)..."
    cmake --build build_urdfdom # --config Debug # Ignored/Invalid
    cmake --install build_urdfdom --prefix ${URDF_INSTALL_DIR} # --config Debug # Ignored/Invalid
else
    echo "No Debug build of urdfdom"
fi

