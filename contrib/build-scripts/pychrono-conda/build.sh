#!/bin/bash
echo "Started build.sh for packaging pre-built PyChrono installation"

# Define the source directory
SRC_DIR=/usr/local
CHRONO_DIR=$HOME/Documents/chrono_build
URDF_LIB_DIR=$HOME/Documents/chrono/libraries/urdf/lib
echo "Copying from source: $SRC_DIR"

# Create the destination directories
mkdir -p $PREFIX/include
mkdir -p $PREFIX/lib/python3.9/site-packages/pychrono
mkdir -p $PREFIX/lib/python3.9/site-packages
mkdir -p $PREFIX/share/chrono

# Copy the Python modules (.py files)
echo "Copying Python module files"
cp -r $CHRONO_DIR/bin/pychrono/*.py $PREFIX/lib/python3.9/site-packages/pychrono/

# Copy demos from the source repository
echo "Copying Python demos"
mkdir -p $PREFIX/lib/python3.9/site-packages/pychrono/demos
cp -r $SRC_DIR/share/chrono/python/pychrono/demos/* $PREFIX/lib/python3.9/site-packages/pychrono/demos/

# Copy the shared objects (.so files) - these are the compiled Python extensions
echo "Copying Python extension modules"
cp $SRC_DIR/share/chrono/python/*.so $PREFIX/lib/python3.9/site-packages/

# Copy library dependencies
echo "Copying library dependencies"
cp -r $SRC_DIR/lib/* $PREFIX/lib/

# For external dependencies from URDF library
echo "Copying external URDF dependencies"
cp $URDF_LIB_DIR/libconsole_bridge.so* $PREFIX/lib/
cp $URDF_LIB_DIR/liburdfdom_model*.so* $PREFIX/lib/
cp $URDF_LIB_DIR/liburdfdom_world*.so* $PREFIX/lib/
cp $URDF_LIB_DIR/liburdfdom_sensor*.so* $PREFIX/lib/
cp $URDF_LIB_DIR/liburdfdom_model_state*.so* $PREFIX/lib/
# Copy tinyxml2 if it exists as a shared library
if [ -f "$URDF_LIB_DIR/libtinyxml2.so" ]; then
    cp $URDF_LIB_DIR/libtinyxml2.so* $PREFIX/lib/
fi

# Copy header files if available
if [ -d "$SRC_DIR/include" ]; then
    echo "Copying include files"
    cp -r $SRC_DIR/include/* $PREFIX/include/
fi

# Copy data files if they exist
if [ -d "$SRC_DIR/share/chrono/data" ]; then
    echo "Copying data files"
    cp -r $SRC_DIR/share/chrono/data $PREFIX/share/chrono/
elif [ -d "$SRC_DIR/data" ]; then
    echo "Copying data files from alternate location"
    cp -r $SRC_DIR/data $PREFIX/share/chrono/
fi

# Make sure the __init__.py file exists
if [ ! -f "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py" ]; then
    echo "Creating __init__.py file"
    touch $PREFIX/lib/python3.9/site-packages/pychrono/__init__.py
fi

# Create path_utils.py file to solve data path issues
echo "Creating path_utils.py file"
cat > $PREFIX/lib/python3.9/site-packages/pychrono/path_utils.py << 'EOF'
# PyChrono imports
import pychrono as chrono
import pychrono.vehicle as veh
import os
from pathlib import Path

def SetChronoDataDirectories():
    """
    Set data directory
    This is useful so data directory paths don't need to be changed everytime
    you pull from or push to github.
    """
    CONDA_PREFIX = os.environ.get('CONDA_PREFIX')
    CHRONO_DATA_DIR = os.environ.get('CHRONO_DATA_DIR')
    if CONDA_PREFIX and not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(
            CONDA_PREFIX, 'share', 'chrono', 'data', '')
    if not CHRONO_DATA_DIR:
        CHRONO_DATA_DIR = os.path.join(Path(os.path.dirname(
            os.path.realpath(__file__))).parents[1], 'chrono', 'data', '')
    elif not CHRONO_DATA_DIR:
        raise Exception(
            'Cannot find the chrono data directory. Please verify that CHRONO_DATA_DIR is set correctly.')
    chrono.SetChronoDataPath(CHRONO_DATA_DIR)
    veh.SetDataPath(os.path.join(CHRONO_DATA_DIR, 'vehicle', ''))
EOF

# Update the __init__.py file to call the SetChronoDataDirectories function
echo "Updating __init__.py to set correct data paths"
# Check if __init__.py exists
if [ -f "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py" ]; then
    # Check if it already has the path_utils import
    if ! grep -q "path_utils" "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py"; then
        # Add the import and function call to the end of __init__.py
        cat >> $PREFIX/lib/python3.9/site-packages/pychrono/__init__.py << 'EOF'

# Import and run the path utility function to set correct data directories
try:
    from .path_utils import SetChronoDataDirectories
    SetChronoDataDirectories()
except Exception as e:
    print(f"Warning: Could not set data directories automatically: {e}")
EOF
    fi
else
    # Create a new __init__.py file
    cat > $PREFIX/lib/python3.9/site-packages/pychrono/__init__.py << 'EOF'
# Import and run the path utility function to set correct data directories
try:
    from .path_utils import SetChronoDataDirectories
    SetChronoDataDirectories()
except Exception as e:
    print(f"Warning: Could not set data directories automatically: {e}")
EOF
fi

# Patch __init__.py to set sensor shader directory to conda path
echo "Patching __init__.py to set sensor shader directory to conda path"
if grep -q "SetSensorShaderDir" "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py"; then
    # Replace the hardcoded path with one that uses the conda environment path
    sed -i 's|_*sensor\.SetSensorShaderDir.*|_sensor.SetSensorShaderDir(os.path.join(os.environ.get("CONDA_PREFIX", ""), "lib", "sensor_ptx") + "/")|' "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py"
fi

# Patch the hardcoded SetChronoDataPath line
echo "Patching __init__.py to fix hardcoded data path"
if grep -q "SetChronoDataPath('/usr/local/share/chrono/data/')" "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py"; then
    # Comment out the hardcoded path
    sed -i 's|SetChronoDataPath.*share/chrono/data.*|# Data path will be set by path_utils.py|' "$PREFIX/lib/python3.9/site-packages/pychrono/__init__.py"
fi

echo "PyChrono packaging completed"

