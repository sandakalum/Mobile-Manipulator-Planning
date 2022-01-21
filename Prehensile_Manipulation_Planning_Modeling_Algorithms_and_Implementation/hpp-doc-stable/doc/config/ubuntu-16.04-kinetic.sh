export INSTALL_HPP_DIR=$DEVEL_HPP_DIR/install
export ROBOTPKG=/opt/openrobots
export ROS=/opt/ros/kinetic

if [ -f $INSTALL_HPP_DIR/setup.bash ]; then
    source $INSTALL_HPP_DIR/setup.bash
else
    source $ROS/setup.bash
    export ROS_PACKAGE_PATH=$INSTALL_HPP_DIR/share:$ROS_PACKAGE_PATH
    export CMAKE_PREFIX_PATH=$INSTALL_HPP_DIR:$CMAKE_PREFIX_PATH
    export LD_LIBRARY_PATH=$INSTALL_HPP_DIR/lib:$LD_LIBRARY_PATH
    export PATH=$INSTALL_HPP_DIR/bin:${PATH}
    export PKG_CONFIG_PATH=$INSTALL_HPP_DIR/lib/pkgconfig:$PKG_CONFIG_PATH
fi

if [ -f "${INSTALL_HPP_DIR}/etc/hpp-tools/bashrc" ]; then
    source "${INSTALL_HPP_DIR}/etc/hpp-tools/bashrc"
fi

# Make sure that /opt/ros/kinetic is in the ROS_PACKAGE_PATH,
# otherwise, you should add it by hand in the line below.
export ROS_PACKAGE_PATH=$ROS_PACKAGE_PATH:$ROBOTPKG/share
export PYTHONPATH=$INSTALL_HPP_DIR/lib/python2.7/site-packages:$INSTALL_HPP_DIR/lib/python2.7/dist-packages:$ROBOTPKG/lib/python2.7/site-packages:$PYTHONPATH
export CMAKE_PREFIX_PATH=$CMAKE_PREFIX_PATH:$ROBOTPKG
export LD_LIBRARY_PATH=$LD_LIBRARY_PATH:$ROBOTPKG/lib
export PATH=$INSTALL_HPP_DIR/sbin:${PATH}
export PKG_CONFIG_PATH=$PKG_CONFIG_PATH:$ROBOTPKG/lib/pkgconfig
