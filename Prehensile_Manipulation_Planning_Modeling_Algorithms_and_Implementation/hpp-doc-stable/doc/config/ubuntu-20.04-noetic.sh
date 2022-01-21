export INSTALL_HPP_DIR=$DEVEL_HPP_DIR/install
export ROBOTPKG=/opt/openrobots
export ROS=/opt/ros/noetic

export PATH=$INSTALL_HPP_DIR/sbin:$INSTALL_HPP_DIR/bin:$ROBOTPKG/bin:$ROBOTPKG/sbin:$ROS/bin:$PATH
export PKG_CONFIG_PATH=$INSTALL_HPP_DIR/lib/pkgconfig/:$ROBOTPKG/lib/pkgconfig:$ROS/lib/pkgconfig

export PYTHONPATH=$INSTALL_HPP_DIR/lib/python3/dist-packages:$ROBOTPKG/lib/python3.8/site-packages:$ROS/lib/python3/dist-packages:$PYTHONPATH

export LD_LIBRARY_PATH=$INSTALL_HPP_DIR/lib:$ROBOTPKG/lib:$INSTALL_HPP_DIR/lib64:$ROS/lib:$LD_LIBRARY_PATH

# Make sure that /opt/ros/noetic is in the ROS_PACKAGE_PATH,
# otherwise, you should add it by hand in the line below.
export ROS_PACKAGE_PATH=$INSTALL_HPP_DIR/share:$ROBOTPKG/share:$ROS/share
export CMAKE_PREFIX_PATH=$INSTALL_HPP_DIR:$ROBOTPKG:$ROS:/usr

if [ -f "${INSTALL_HPP_DIR}/etc/hpp-tools/bashrc" ]; then
    source "${INSTALL_HPP_DIR}/etc/hpp-tools/bashrc"
fi
