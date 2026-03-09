source "/opt/ros/jazzy/setup.bash"
export ROBOTPKG=/opt/openrobots

export PATH=$ROBOTPKG/bin:$ROBOTPKG/sbin:$PATH
export PKG_CONFIG_PATH=$ROBOTPKG/lib/pkgconfig:$PKG_CONFIG_PATH

export PYTHONPATH=$ROBOTPKG/lib/python3.12/site-packages:$PYTHONPATH

export LD_LIBRARY_PATH=$ROBOTPKG/lib:$LD_LIBRARY_PATH

export AMENT_PREFIX_PATH=$ROBOTPKG:$AMENT_PREFIX_PATH
export CMAKE_PREFIX_PATH=$ROBOTPKG:/usr:$CMAKE_PREFIX_PATH

