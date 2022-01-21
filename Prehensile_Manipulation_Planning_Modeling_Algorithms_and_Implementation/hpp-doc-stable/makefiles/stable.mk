#
# Copyright (c) 2014 CNRS-LAAS
# Author: Florent Lamiraux
#

FCL_REPO=https://github.com/flexible-collision-library
LAAS_REPO=https://github.com/laas
HPP_REPO=https://github.com/humanoid-path-planner
SOT_REPO=https://github.com/stack-of-tasks
GEPETTO_REPO=https://github.com/Gepetto
TRAC_REPO=ssh://trac.laas.fr/git/jrl/robots/ros-hrp2
LOCO3D_REPO=https://github.com/loco-3d

SRC_DIR=${DEVEL_HPP_DIR}/src
ifndef INSTALL_HPP_DIR
INSTALL_HPP_DIR=${DEVEL_HPP_DIR}/install
endif

BUILD_TYPE?=Release
BUILD_TESTING?=ON
ifeq (${BUILD_TYPE},Debug)
  BUILD_FOLDER=build
else
  BUILD_FOLDER=build-rel
  BUILD_TESTING=OFF
endif

OSG_PACKAGE=osg-dae
#OSG_PACKAGE=OpenSceneGraph-3.4.0

WGET=wget --quiet
UNZIP=unzip -qq
TAR=tar
GIT_QUIET=--quiet
# Qt version should be either 4 or 5
QT_VERSION=5
INSTALL_DOCUMENTATION=ON

# Either a version tag (e.g. v4.3.0), stable or devel
HPP_VERSION=v4.11.0
HPP_EXTRA_FLAGS= -DBUILD_TESTING=${BUILD_TESTING}$

##################################
# {{{ Dependencies

eigenpy_branch=v2.6.8
eigenpy_repository=${SOT_REPO}


pinocchio_branch=v2.6.3
pinocchio_repository=${SOT_REPO}
pinocchio_extra_flags= -DBUILD_UNIT_TESTS=OFF -DBUILD_WITH_COLLISION_SUPPORT=ON

hpp-template-corba_branch=${HPP_VERSION}
hpp-template-corba_repository=${HPP_REPO}

# }}}
##################################
# {{{ Packages supporting HPP_VERSION
hpp-util_branch=${HPP_VERSION}
hpp-util_repository=${HPP_REPO}

hpp-fcl_branch=v1.7.5
hpp-fcl_repository=${HPP_REPO}
hpp-fcl_extra_flags= -DCMAKE_BUILD_TYPE=Release

hpp-statistics_branch=${HPP_VERSION}
hpp-statistics_repository=${HPP_REPO}

hpp-pinocchio_branch=${HPP_VERSION}
hpp-pinocchio_repository=${HPP_REPO}
hpp-pinocchio_extra_flags=${HPP_EXTRA_FLAGS}

hpp-constraints_branch=${HPP_VERSION}
hpp-constraints_repository=${HPP_REPO}
hpp-constraints_extra_flags=${HPP_EXTRA_FLAGS} -DUSE_QPOASES=OFF

hpp-core_branch=${HPP_VERSION}
hpp-core_repository=${HPP_REPO}
hpp-core_extra_flags=${HPP_EXTRA_FLAGS}

hpp-corbaserver_branch=${HPP_VERSION}
hpp-corbaserver_repository=${HPP_REPO}

hpp-doc_branch=stable
hpp-doc_repository=${HPP_REPO}

hpp-manipulation_branch=${HPP_VERSION}
hpp-manipulation_repository=${HPP_REPO}

hpp-manipulation-urdf_branch=${HPP_VERSION}
hpp-manipulation-urdf_repository=${HPP_REPO}

hpp-manipulation-corba_branch=${HPP_VERSION}
hpp-manipulation-corba_repository=${HPP_REPO}

hpp_tutorial_branch=${HPP_VERSION}
hpp_tutorial_repository=${HPP_REPO}

hpp-gepetto-viewer_branch=${HPP_VERSION}
hpp-gepetto-viewer_repository=${HPP_REPO}

hpp-plot_branch=${HPP_VERSION}
hpp-plot_repository=${HPP_REPO}

hpp-gui_branch=${HPP_VERSION}
hpp-gui_repository=${HPP_REPO}

hpp-practicals_branch=${HPP_VERSION}
hpp-practicals_repository=${HPP_REPO}

# }}}
##################################
# {{{ Robot specific package + test packages

example-robot-data_branch=v3.10.0
example-robot-data_repository=${GEPETTO_REPO}
example-robot-data_extra_flags= -DBUILD_PYTHON_INTERFACE=OFF

hrp2-14-description_branch=master
hrp2-14-description_repository=${TRAC_REPO}

hpp-hrp2_branch=v4.9.0
hpp-hrp2_repository=${HPP_REPO}

robot_capsule_urdf_branch=groovy
robot_capsule_urdf_repository=${LAAS_REPO}

robot_model_py_branch=groovy
robot_model_py_repository=${LAAS_REPO}

hpp_benchmark_branch=devel
hpp_benchmark_repository=${HPP_REPO}

hpp-environments_branch=v4.11.1
hpp-environments_repository=${HPP_REPO}

universal_robot_branch=kinetic
universal_robot_repository=${HPP_REPO}

hpp-universal-robot_branch=${HPP_VERSION}
hpp-universal-robot_repository=${HPP_REPO}

hpp-baxter_branch=v4.11.1
hpp-baxter_repository=${HPP_REPO}

hpp_romeo_branch=${HPP_VERSION}
hpp_romeo_repository=${HPP_REPO}


# }}}
##################################
# {{{ Packages for rbprm

hpp-affordance_branch=${HPP_VERSION}
hpp-affordance_repository=${HPP_REPO}
hpp-affordance_extra_flags=${HPP_EXTRA_FLAGS}

hpp-affordance-corba_branch=${HPP_VERSION}
hpp-affordance-corba_repository=${HPP_REPO}
hpp-affordance-corba_extra_flags=${HPP_EXTRA_FLAGS}

anymal-rbprm_branch=${HPP_VERSION}
anymal-rbprm_repository=${HPP_REPO}

hyq-rbprm_branch=${HPP_VERSION}
hyq-rbprm_repository=${HPP_REPO}

simple-humanoid-rbprm_branch=${HPP_VERSION}
simple-humanoid-rbprm_repository=${HPP_REPO}

solo-rbprm_branch=${HPP_VERSION}
solo-rbprm_repository=${HPP_REPO}

talos-rbprm_branch=${HPP_VERSION}
talos-rbprm_repository=${HPP_REPO}

hpp-rbprm_branch=${HPP_VERSION}
hpp-rbprm_repository=${HPP_REPO}
hpp-rbprm_extra_flags=${HPP_EXTRA_FLAGS}

hpp-rbprm-corba_branch=${HPP_VERSION}
hpp-rbprm-corba_repository=${HPP_REPO}
hpp-rbprm-corba_extra_flags=${HPP_EXTRA_FLAGS}

hpp-centroidal-dynamics_branch=${HPP_VERSION}
hpp-centroidal-dynamics_repository=${HPP_REPO}

hpp-bezier-com-traj_branch=${HPP_VERSION}
hpp-bezier-com-traj_repository=${HPP_REPO}
hpp-bezier-com-traj_extra_flags= -DBUILD_PYTHON_INTERFACE=ON

ndcurves_branch=v1.1.0
ndcurves_repository=${LOCO3D_REPO}
ndcurves_extra_flags= -DBUILD_PYTHON_INTERFACE=ON

eigenpy_branch=v2.6.7
eigenpy_repository=${SOT_REPO}

# }}}
##################################
# {{{ Packages for gepetto-gui

collada-dom_branch=master
collada-dom_repository=${HPP_REPO}
collada-dom_extra_flags=-DBUILD_SHARED_LIBS=TRUE -DOPT_COLLADA15=FALSE

osg-dae_branch=master
osg-dae_repository=${GEPETTO_REPO}
osg-dae_extra_flags= -DCOLLADA_DYNAMIC_LIBRARY=${INSTALL_HPP_DIR}/lib/libcollada14dom.so -DCOLLADA_INCLUDE_DIR=${INSTALL_HPP_DIR}/include/collada-dom
OpenSceneGraph-3.4.0_extra_flags= -DDESIRED_QT_VERSION=${QT_VERSION} -DCOLLADA_DYNAMIC_LIBRARY=${INSTALL_HPP_DIR}/lib/libcollada14dom.so -DCOLLADA_INCLUDE_DIR=${INSTALL_HPP_DIR}/include/collada-dom -DLIB_POSTFIX=""

gepetto-viewer_branch=v4.12.0
gepetto-viewer_repository=${GEPETTO_REPO}
ifeq (${QT_VERSION}, 5)
	gepetto-viewer_extra_flags= -DPROJECT_USE_QT4=OFF
else
	gepetto-viewer_extra_flags= -DPROJECT_USE_QT4=ON
endif

gepetto-viewer-corba_branch=v5.6.0
gepetto-viewer-corba_repository=${GEPETTO_REPO}

qgv_branch=v1.3.2
qgv_repository=${HPP_REPO}
ifeq (${QT_VERSION}, 5)
	qgv_extra_flags=-DBINDINGS_QT5=ON -DBINDINGS_QT4=OFF
else
	qgv_extra_flags=-DBINDINGS_QT5=OFF -DBINDINGS_QT4=ON
endif

hpp-tools_branch=v4.9.0
hpp-tools_repository=${HPP_REPO}
hpp-tools_extra_flags=

# }}}
##################################
# {{{ High-level targets

all: hpp_tutorial.install hpp-gepetto-viewer.install hpp-plot.install hpp-gui.install
	${MAKE} hpp-doc.install

# For test on gepgitlab, install robot packages first
test-ci: example-robot-data.install  hpp-environments.install \
	hpp-baxter.install
	${MAKE} hpp_tutorial.install hpp-gepetto-viewer.install hpp-rbprm-corba.install \
	hpp-universal-robot.install && \
	${MAKE} hpp-doc.install

# For benchmark, install robot packages first
benchmark: example-robot-data.install \
	hpp-environments.install hrp2-14-description.install
	${MAKE} hpp_tutorial.install hpp-gepetto-viewer.install; \
	${MAKE} hpp-baxter.install hpp_romeo.install \
	hpp-universal-robot.install hpp-plot.install hpp-gui.install; \
	${MAKE} hpp_benchmark.checkout; \
	${MAKE} hpp-doc.install

rbprm: hpp-rbprm-corba.install hpp-gepetto-viewer.install
	${MAKE} hpp-doc.install


# }}}
##################################
# {{{ Dependencies declaration

hpp-doc.configure.dep: hpp-doc.checkout
hpp-fcl.configure.dep: hpp-fcl.checkout eigenpy.install
hpp-util.configure.dep: hpp-util.checkout
hpp-model-urdf.configure.dep: hpp-model.install hpp-model-urdf.checkout
pinocchio.configure.dep: hpp-fcl.install pinocchio.checkout
hpp-pinocchio.configure.dep: pinocchio.install hpp-util.install \
	hpp-pinocchio.checkout
hpp-statistics.configure.dep: hpp-util.install hpp-statistics.checkout
hpp-core.configure.dep: example-robot-data.install hpp-constraints.install \
	hpp-statistics.install hpp-core.checkout
hpp-constraints.configure.dep: hpp-pinocchio.install hpp-statistics.install \
	hpp-environments.install hpp-constraints.checkout
hpp-manipulation.configure.dep: hpp-core.install hpp-constraints.install \
	hpp-manipulation.checkout
hpp-manipulation-corba.configure.dep: hpp-manipulation-urdf.install \
	hpp-manipulation.install hpp-corbaserver.install \
	hpp-template-corba.install hpp-manipulation-corba.checkout
hpp-plot.configure.dep: hpp-corbaserver.install hpp-manipulation-corba.install \
	qgv.install hpp-plot.checkout
hpp-manipulation-urdf.configure.dep:hpp-manipulation.install \
	hpp-manipulation-urdf.checkout
hpp-corbaserver.configure.dep: hpp-core.install hpp-template-corba.install \
	hpp-constraints.install hpp-corbaserver.checkout
hpp-template-corba.configure.dep: hpp-util.install hpp-template-corba.checkout
qgv.configure.dep: qgv.checkout
robot_model_py.configure.dep: robot_model_py.checkout
robot_capsule_urdf.configure.dep: robot_model_py.install \
	robot_capsule_urdf.checkout
hpp-hrp2.configure.dep: hrp2-14-description.install hpp-corbaserver.install \
	hpp-hrp2.checkout
hrp2-14-description.configure.dep: robot_capsule_urdf.install \
	robot_model_py.install hrp2-14-description.checkout
test-hpp.configure.dep: \
	hpp-gepetto-viewer.install hpp-hrp2.install test-hpp.checkout
hpp_tutorial.configure.dep: hpp-gepetto-viewer.install \
	hpp-manipulation-corba.install hpp_tutorial.checkout
hpp_benchmark.configure.dep: hpp_tutorial.install hpp_benchmark.checkout
collada-dom.configure.dep: collada-dom.checkout
osg-dae.configure.dep: collada-dom.install \
	osg-dae.checkout
OpenSceneGraph-3.4.0.configure.dep: collada-dom.install \
	OpenSceneGraph-3.4.0.checkout
gepetto-viewer.configure.dep: gepetto-viewer.checkout
gepetto-viewer-corba.configure.dep: gepetto-viewer.install \
	gepetto-viewer-corba.checkout
hpp-gepetto-viewer.configure.dep: gepetto-viewer-corba.install \
	hpp-corbaserver.install \
	hpp-gepetto-viewer.checkout
hpp-gui.configure.dep: hpp-gui.checkout
universal_robot.configure.dep: universal_robot.checkout
hpp-universal-robot.configure.dep: example-robot-data.install \
	hpp-universal-robot.checkout
example-robot-data.configure.dep: example-robot-data.checkout
hpp-environments.configure.dep: hpp-environments.checkout
hpp-baxter.configure.dep: example-robot-data.install hpp-baxter.checkout
hpp_romeo.configure.dep: hpp_romeo.checkout
hpp-affordance.configure.dep: hpp-core.install hpp-fcl.install hpp-affordance.checkout
hpp-affordance-corba.configure.dep: hpp-affordance.install hpp-template-corba.install \
 hpp-corbaserver.install hpp-affordance-corba.checkout
anymal-rbprm.configure.dep: anymal-rbprm.checkout
hyq-rbprm.configure.dep: hyq-rbprm.checkout
simple-humanoid-rbprm.configure.dep: simple-humanoid-rbprm.checkout
solo-rbprm.configure.dep: solo-rbprm.checkout
talos-rbprm.configure.dep: talos-rbprm.checkout
hpp-rbprm.configure.dep: hpp-core.install hpp-centroidal-dynamics.install \
	anymal-rbprm.install hyq-rbprm.install talos-rbprm.install simple-humanoid-rbprm.install solo-rbprm.install \
	hpp-affordance.install ndcurves.install \
	hpp-bezier-com-traj.install hpp-rbprm.checkout
hpp-rbprm-corba.configure.dep: hpp-rbprm.install hpp-affordance-corba.install \
 hpp-corbaserver.install hpp-rbprm-corba.checkout
hpp-centroidal-dynamics.configure.dep: eigenpy.install hpp-centroidal-dynamics.checkout
hpp-bezier-com-traj.configure.dep: hpp-centroidal-dynamics.install ndcurves.install hpp-bezier-com-traj.checkout
ndcurves.configure.dep: ndcurves.checkout
eigenpy.configure.dep: eigenpy.checkout
hpp-tools.configure.dep: hpp-tools.checkout

# }}}
##################################
# {{{ Targets

status:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".status; \
	done

log:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".log; \
	done

fetch:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".fetch; \
	done

update:
	@for child_dir in $$(ls ${SRC_DIR}); do \
		test -d "$$child_dir" || continue; \
		test -d "$$child_dir/.git" || continue; \
		${MAKE} "$$child_dir".update; \
	done

%.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		git clone ${GIT_QUIET} --recursive -b ${$(@:.checkout=)_branch} ${$(@:.checkout=)_repository}/$(@:.checkout=); \
	fi \

%.fetch:
	if [ "${$(@:.fetch=)_repository}" = "" ]; then \
		echo "$(@:.fetch=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.fetch=);\
		git fetch ${GIT_QUIET} origin; \
		git fetch ${GIT_QUIET} origin --tags; \
	fi

%.update:
	if [ "${$(@:.update=)_repository}" = "" ]; then \
		echo "$(@:.update=) is not referenced"; \
	else \
		cd ${SRC_DIR}/$(@:.update=);\
		git remote rm origin;\
		git remote add origin ${$(@:.update=)_repository}/$(@:.update=);\
		git fetch origin;\
		git fetch origin --tags;\
		git checkout -q --detach;\
		git branch -f ${$(@:.update=)_branch} origin/${$(@:.update=)_branch};\
		git checkout -q ${$(@:.update=)_branch};\
		git submodule update; \
	fi


%.configure.dep: %.checkout

%.configure: %.configure.dep
	${MAKE} $(@:.configure=).configure_nodep

%.configure_nodep:%.checkout
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} \
			-DENFORCE_MINIMAL_CXX_STANDARD=ON \
			-DINSTALL_DOCUMENTATION=${INSTALL_DOCUMENTATION} \
			-DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" \
			${$(@:.configure_nodep=)_extra_flags} ..

%.install:%.configure
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/${BUILD_FOLDER} install

%.install_nodep:%.configure_nodep
	${MAKE} -C ${SRC_DIR}/$(@:.install_nodep=)/${BUILD_FOLDER} install

%.uninstall:
	${MAKE} -C ${SRC_DIR}/$(@:.uninstall=)/${BUILD_FOLDER} uninstall

%.clean:
	${MAKE} -C ${SRC_DIR}/$(@:.clean=)/${BUILD_FOLDER} clean

%.very-clean:
	rm -rf ${SRC_DIR}/$(@:.very-clean=)/${BUILD_FOLDER}/*

%.status:
	@cd ${SRC_DIR}/$(@:.status=); \
	echo \
	"\033[1;36m------- Folder $(@:.status=) ---------------\033[0m"; \
	git --no-pager -c status.showUntrackedFiles=no status --short --branch;\

%.log:
	@cd ${SRC_DIR}/$(@:.log=); \
	if [ -f .git/refs/heads/${$(@:.log=)_branch} ]; then \
		echo -n "$(@:.log=): "; \
		cat .git/refs/heads/${$(@:.log=)_branch}; \
	fi

OpenSceneGraph-3.4.0.checkout:
	if [ -d $(@:.checkout=) ]; then \
		echo "$(@:.checkout=) already checkout out."; \
	else \
		wget http://www.openscenegraph.org/downloads/stable_releases/OpenSceneGraph-3.4.0/source/OpenSceneGraph-3.4.0.zip;\
		cd ${SRC_DIR}; unzip OpenSceneGraph-3.4.0.zip;\
		rm -f OpenSceneGraph-3.4.0.zip;\
	fi

robot_model_py.configure: robot_model_py.configure.dep
	cd ${SRC_DIR}/$(@:.configure=)/xml_reflection;\
	mkdir -p ${BUILD_FOLDER}; \
	cd ${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..
	cd ${SRC_DIR}/$(@:.configure=)/urdf_parser_py;\
	mkdir -p ${BUILD_FOLDER}; \
	cd ${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${INSTALL_HPP_DIR} -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} ..

robot_model_py.install: robot_model_py.configure
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/xml_reflection/${BUILD_FOLDER} install; \
	${MAKE} -C ${SRC_DIR}/$(@:.install=)/urdf_parser_py/${BUILD_FOLDER} install;

universal_robot.configure_nodep:
	mkdir -p ${SRC_DIR}/$(@:.configure_nodep=)/ur_description/${BUILD_FOLDER}; \
	cd ${SRC_DIR}/$(@:.configure_nodep=)/ur_description/${BUILD_FOLDER}; \
	cmake -DCMAKE_INSTALL_PREFIX=${DEVEL_HPP_DIR}/install -DCMAKE_INSTALL_LIBDIR=lib -DCMAKE_BUILD_TYPE=${BUILD_TYPE} -DCMAKE_CXX_FLAGS_RELWITHDEBINFO="-g -O3 -DNDEBUG" ${$(@:.configure_nodep=)_extra_flags} ..

universal_robot.install_nodep:universal_robot.configure_nodep
	cd ${SRC_DIR}/$(@:.install_nodep=)/ur_description/${BUILD_FOLDER};\
	make install

universal_robot.install:universal_robot.configure
	cd ${SRC_DIR}/$(@:.install=)/ur_description/${BUILD_FOLDER};\
	make install

# }}}

# vim: foldmethod=marker foldlevel=0
