FROM ubuntu:18.04

ENV DEVEL_HPP_DIR=/workspace UBUNTU=bionic ROS=melodic

RUN apt-get update -y && apt-get install -y \
   curl \
   gnupg2 \
   && rm -rf /var/lib/apt/lists/*

ADD repos.key /
RUN echo "deb http://packages.ros.org/ros/ubuntu ${UBUNTU} main" > /etc/apt/sources.list.d/ros-latest.list \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/wip/packages/debian/pub ${UBUNTU} robotpkg" \
 >> /etc/apt/sources.list.d/robotpkg.list \
 && echo "deb [arch=amd64] http://robotpkg.openrobots.org/packages/debian/pub ${UBUNTU} robotpkg" \
 >> /etc/apt/sources.list.d/robotpkg.list \
 && apt-key add /repos.key

RUN apt-get update -qqy \
 && DEBIAN_FRONTEND=noninteractive apt-get install -qqy \
    cmake \
    doxygen \
    g++ \
    git \
    graphviz \
    libassimp-dev \
    libboost-dev \
    libccd-dev \
    libcdd-dev \
    libeigen3-dev \
    libglpk-dev \
    libgraphviz-dev \
    libltdl-dev \
    libomniorb4-dev \
    libpcre3-dev \
    libpythonqt-qtall-qt5-python2-dev \
    libpythonqt-qtall-qt5-python3-dev \
    libqt4-opengl-dev \
    libqtgui4 \
    libqtwebkit-dev \
    libtinyxml2-dev \
    liburdfdom-dev \
    libxml2 \
    omniorb-nameserver \
    oxygen-icon-theme \
    psmisc \
    python-matplotlib \
    python-omniorb \
    qt4-dev-tools \
    qtbase5-dev \
    qttools5-dev \
    robotpkg-omniorb \
    robotpkg-openscenegraph \
    robotpkg-qpoases+doc \
    robotpkg-romeo-description \
    robotpkg-py36-omniorbpy \
    robotpkg-qt5-osgqt \
    ros-${ROS}-octomap \
    ros-${ROS}-pr2-description \
    ros-${ROS}-resource-retriever \
    ros-${ROS}-srdfdom \
    ros-${ROS}-xacro \
    sudo \
    wget \
 && apt-get remove -y texlive-latex-base texlive-binaries ghostscript \
 && apt-get autoremove -y \
 && rm -rf /var/lib/apt/lists/*

COPY auto-install-hpp.sh /
