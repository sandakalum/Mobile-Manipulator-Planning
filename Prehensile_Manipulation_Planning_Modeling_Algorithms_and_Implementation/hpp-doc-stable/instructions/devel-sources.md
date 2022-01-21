## Source installation on ubuntu-18.04 64 bit with ros-melodic

To install all the packages on ubuntu 18.04 LTS 64 bit, you should do the following steps:

  1. install ROS-melodic: follow steps 1.1 to 1.3 of [the ROS installation website.](http://wiki.ros.org/melodic/Installation/Ubuntu).

  2. install robotpkg: follow [the robotpkg installation website](http://robotpkg.openrobots.org/debian.html).

  3. install by apt-get
  ```bash
    sudo apt-get install \
      g++ \
      cmake \
      doxygen \
      libboost-dev \
      liburdfdom-dev \
      libassimp-dev \
      libeigen3-dev \
      libgraphviz-dev \
      robotpkg-omniorb \
      robotpkg-qpoases+doc \
      robotpkg-romeo-description \
      robotpkg-py36-omniorbpy \
      ros-melodic-xacro \
      libccd-dev \
      ros-melodic-octomap \
      ros-melodic-resource-retriever \
      ros-melodic-srdfdom \
      ros-melodic-pr2-description \
      git \
      libltdl-dev \
      python-omniorb \
      python-matplotlib \
      python3-catkin \
      libxml2 \
      libtinyxml2-dev \
      qt4-dev-tools \
      libqt4-opengl-dev \
      libqtgui4 \
      libqtwebkit-dev \
      oxygen-icon-theme \
      robotpkg-openscenegraph \
      libpcre3-dev \
      wget \
      libcdd-dev
    ```

  4. Choose a directory on your file system and define the environment
     variable `DEVEL_HPP_DIR` with the full path to this directory.
     - the packages will be cloned into `$DEVEL_HPP_DIR/src`,
     - the packages will be installed in `$DEVEL_HPP_DIR/install`.
     It is recommanded to set variable `DEVEL_HPP_DIR` in your `.bashrc` for future use.

    ```bash
    mkdir -p $DEVEL_HPP_DIR/src
    ```
  5. Copy Config and Makefile

    ```bash
    wget -O $DEVEL_HPP_DIR/config.sh https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/stable/doc/config/ubuntu-18.04-melodic.sh
    wget -O $DEVEL_HPP_DIR/src/Makefile https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/stable/makefiles/devel.mk
    ```

  6. cd into `$DEVEL_HPP_DIR` and type

    ```bash
    cd ${DEVEL_HPP_DIR}
    source config.sh
    ```

  7. cd into `$DEVEL_HPP_DIR/src` and type

    ```bash
    cd ${DEVEL_HPP_DIR}/src
    source ../config.sh;
    make all
    ```

  8. open `$DEVEL_HPP_DIR/install/share/doc/hpp-doc/index.html` in a web brower and you
  will have access to the documentation of most packages.

## Source installation on ubuntu-20.04 64 bit with ros-noetic

To install all the packages on ubuntu 20.04 LTS 64 bit, you should do the following steps:

  1. install ROS-noetic: follow steps 1.1 to 1.3 of [the ROS installation website.](http://wiki.ros.org/noetic/Installation/Ubuntu).

  2. install robotpkg: follow [the robotpkg installation website](http://robotpkg.openrobots.org/debian.html).

  3. install by apt-get
  ```bash
    sudo apt-get install \
      g++ \
      cmake \
      doxygen \
      libboost-dev \
      liburdfdom-dev \
      libassimp-dev \
      libeigen3-dev \
      libgraphviz-dev \
      graphviz \
      robotpkg-omniorb \
      robotpkg-qpoases+doc \
      robotpkg-romeo-description \
      robotpkg-py38-omniorbpy \
      ros-noetic-xacro \
      libccd-dev \
      ros-noetic-octomap \
      ros-noetic-resource-retriever \
      ros-noetic-srdfdom \
      ros-noetic-pr2-description \
      git \
      libltdl-dev \
      python-omniorb \
      python3-matplotlib \
      qtbase5-private-dev \
      qtdeclarative5-dev \
      qtmultimedia5-dev \
      libqt5svg5-dev \
      libxml2 \
      libtinyxml2-dev \
      oxygen-icon-theme \
      robotpkg-openscenegraph \
      libpcre3-dev \
      wget \
      libcdd-dev
    ```

  4. Choose a directory on your file system and define the environment
     variable `DEVEL_HPP_DIR` with the full path to this directory.
     - the packages will be cloned into `$DEVEL_HPP_DIR/src`,
     - the packages will be installed in `$DEVEL_HPP_DIR/install`.
     It is recommanded to set variable `DEVEL_HPP_DIR` in your `.bashrc` for future use.

    ```bash
    mkdir -p $DEVEL_HPP_DIR/src
    ```
  5. Copy Config and Makefile

    ```bash
    wget -O $DEVEL_HPP_DIR/config.sh https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/stable/doc/config/ubuntu-20.04-noetic.sh
    wget -O $DEVEL_HPP_DIR/src/Makefile https://raw.githubusercontent.com/humanoid-path-planner/hpp-doc/stable/makefiles/devel.mk
    ```

  6. cd into `$DEVEL_HPP_DIR` and type

    ```bash
    cd ${DEVEL_HPP_DIR}
    source config.sh
    ```

  7. cd into `$DEVEL_HPP_DIR/src` and type

    ```bash
    cd ${DEVEL_HPP_DIR}/src
    source ../config.sh;
    make all
    ```

  8. open `$DEVEL_HPP_DIR/install/share/doc/hpp-doc/index.html` in a web brower and you
  will have access to the documentation of most packages.
