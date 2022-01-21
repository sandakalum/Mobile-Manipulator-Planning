## Binary installation on ubuntu-18.04 64 bit with ros-melodic

To install all the packages on ubuntu 18.04 LTS 64 bit, you should do the following steps:

  1. install ROS-melodic: follow steps 1.1 to 1.3 of [the ROS installation website.](http://wiki.ros.org/melodic/Installation/Ubuntu).

  2. install robotpkg: follow [the robotpkg installation website](http://robotpkg.openrobots.org/debian.html).

  3. install HPP:

    ```bash
    #pyver=27
    pyver=36
    sudo apt-get install robotpkg-py${pyver}-hpp-manipulation-corba robotpkg-py${pyver}-qt5-hpp-gepetto-viewer
    ```

  4. install (optionnal) extra packages for demonstrations:

    - Tutorials:

      ```bash
      sudo apt-get install robotpkg-py${pyver}-hpp-tutorial
      ```

    - GUI:

      ```bash
      sudo apt-get install robotpkg-py${pyver}-qt5-hpp-gui robotpkg-py${pyver}-qt5-hpp-plot
      ```

    - Some robot descriptions:

      ```bash
      sudo apt-get install ros-melodic-pr2-description robotpkg-py${pyver}-hpp-environments robotpkg-romeo-description
      ```

  5. setup your environment variables by adding the following lines (fix Python version if necessary) to your `.bashrc`:

    ```bash
    source /opt/ros/melodic/setup.bash

    export PATH=/opt/openrobots/bin${!PATH:-:}${PATH}
    export LD_LIBRARY_PATH=/opt/openrobots/lib${!LD_LIBRARY_PATH:-:}${LD_LIBRARY_PATH}
    export PYTHONPATH=/opt/openrobots/lib/python2.7/site-packages${!PYTHONPATH:-:}${PYTHONPATH}
    export ROS_PACKAGE_PATH=/opt/openrobots/share${!ROS_PACKAGE_PATH:-:}${ROS_PACKAGE_PATH}

    export CMAKE_PREFIX_PATH=/opt/openrobots${!CMAKE_PREFIX_PATH:-:}${CMAKE_PREFIX_PATH}
    export PKG_CONFIG_PATH=/opt/openrobots${!PKG_CONFIG_PATH:-:}${PKG_CONFIG_PATH}
    ```

  6. open `/opt/openrobots/share/doc/hpp-doc/index.html` in a web brower and you
  will have access to the documentation of most packages.
