cmake_minimum_required(VERSION 3.1)

# These variables have to be defined before running setup_project
set(PROJECT_NAME hpp-doc)
set(PROJECT_DESCRIPTION "Documentation for HPP")
set(PROJECT_URL https://humanoid-path-planner.github.io/hpp-doc/index.html)

include(cmake/hpp.cmake)

COMPUTE_PROJECT_ARGS(PROJECT_ARGS LANGUAGES CXX)
PROJECT(${PROJECT_NAME} ${PROJECT_ARGS})

# Arguments are:
# - (required) package name
# - (optional) git url
# - (optional) robotpkg name
MACRO(ADD_PKG pkg)
  SET (OPT_ARGS ${ARGN})
  LIST(APPEND SETUP_DOC_PKG ${pkg})
  LIST(LENGTH OPT_ARGS N_OPT_ARGS)
  IF(${N_OPT_ARGS} GREATER 0)
    LIST(GET OPT_ARGS 0 ${pkg}_giturl)
  ENDIF(${N_OPT_ARGS} GREATER 0)
  IF(${N_OPT_ARGS} GREATER 1)
    LIST(GET OPT_ARGS 1 ${pkg}_robotpkg_package)
  ENDIF(${N_OPT_ARGS} GREATER 1)
ENDMACRO(ADD_PKG)

MACRO(ADD_HPP_PKG pkg)
  ADD_PKG(${pkg} git://github.com:humanoid-path-planner/${pkg})
ENDMACRO(ADD_HPP_PKG)

SET(SETUP_DOC_PKG)

ADD_HPP_PKG(hpp-fcl)
ADD_HPP_PKG(hpp-util)
ADD_HPP_PKG(pinocchio)
ADD_HPP_PKG(hpp-pinocchio)
ADD_HPP_PKG(hpp-core)
ADD_HPP_PKG(hpp-corbaserver)
ADD_HPP_PKG(hpp-constraints)
ADD_HPP_PKG(hpp-manipulation)
ADD_HPP_PKG(hpp-manipulation-urdf)
ADD_HPP_PKG(hpp-manipulation-corba)
ADD_HPP_PKG(gepetto-viewer)
ADD_HPP_PKG(gepetto-viewer-corba)
ADD_HPP_PKG(hpp-gepetto-viewer)
ADD_HPP_PKG(hpp_tutorial)

FOREACH(PKG ${SETUP_DOC_PKG})
  ADD_PROJECT_DEPENDENCY(${PKG})

  SET(PREFIX "${PKG}")
  SET(DOC_VAR "${PKG}")
  LIST(APPEND LOGGING_WATCHED_VARIABLES
    ${PREFIX}_FOUND)
  IF(${PREFIX}_FOUND)
    IF(DEFINED ${PREFIX}_DOXYGENDOCDIR)
      SET(DOC_PREFIX ${${PREFIX}_DOXYGENDOCDIR})
    ELSEIF(DEFINED ${PREFIX}_DOCDIR)
      SET(DOC_PREFIX ${${PREFIX}_DOCDIR})
    ELSE(DEFINED ${PREFIX}_DOCDIR)
      SET(DOC_PREFIX ${${PREFIX}_PREFIX}/share/doc/${PKG}/doxygen-html)
    ENDIF(DEFINED ${PREFIX}_DOXYGENDOCDIR)
    STRING(REPLACE ${CMAKE_INSTALL_PREFIX}/share/doc ../.. DOC_PREFIX "${DOC_PREFIX}")
    IF (EXISTS ${DOC_PREFIX}/main.html)
      SET(${DOC_VAR}_URL ${DOC_PREFIX}/main.html)
    ELSE (EXISTS ${DOC_PREFIX}/main.html)
      SET(${DOC_VAR}_URL ${DOC_PREFIX}/index.html)
    ENDIF (EXISTS ${DOC_PREFIX}/main.html)
    SET(${DOC_VAR}_LINK "<a href='${${DOC_VAR}_URL}'>${PKG}</a>")
  ELSE(${PREFIX}_FOUND)
    SET(PACKAGE_NAME ${PKG})
    SET(DOC_PKG_NAME "hpp_doc_missing_${DOC_VAR}")
    SET(${DOC_VAR}_LINK "\\ref ${DOC_PKG_NAME}")
    SET(${DOC_VAR}_URL ${${DOC_VAR}_LINK})
    SET(STEPS)
    IF(DEFINED ${PKG}_giturl)
      SET(STEPS "${STEPS}\\li get the source code through \\c git:
\\code git clone --recursive ${${PKG}_giturl} \\endcode
Then configure, compile and install the package.\n")
    ENDIF(DEFINED ${PKG}_giturl)
    IF(DEFINED ${PKG}_robotpkg_package)
      SET(STEPS "${STEPS}\\li install it through \\c robotpkg :
\\code
cd robotpkg/${${PKG}_robotpkg_package}
make update
\\endcode")
    ENDIF(DEFINED ${PKG}_robotpkg_package)
    CONFIGURE_FILE(doc/missing.dox.in ${CMAKE_BINARY_DIR}/doc/${PKG}-missing.dox @ONLY)
  ENDIF(${PREFIX}_FOUND)
    LIST(APPEND LOGGING_WATCHED_VARIABLES ${DOC_VAR}_URL ${DOC_VAR}_LINK)

ENDFOREACH(PKG ${SETUP_DOC_PKG})

CONFIG_FILES(doc/main.dox doc/doxygenLayout.xml)

ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_BINARY_DIR}/graphDep.dot
  COMMAND ${CMAKE_SOURCE_DIR}/scripts/packageDep --quiet --output ${CMAKE_BINARY_DIR}/graphDep.dot ${SETUP_DOC_PKG})
ADD_CUSTOM_TARGET(generate-html ALL
  DEPENDS graphDep.dot)

INSTALL(FILES
  doc/index.html
  doc/jquery-3.3.1.min.js
  ${CMAKE_BINARY_DIR}/doc/main.dox
  DESTINATION ${CMAKE_INSTALL_DOCDIR})
INSTALL(DIRECTORY
  doc/figures
  DESTINATION ${CMAKE_INSTALL_DOCDIR})
IF(NOT DEFINED INSTALL_DOCUMENTATION OR ${INSTALL_DOCUMENTATION})
  INSTALL(
    FILES ${CMAKE_CURRENT_BINARY_DIR}/doc/doxygen-xml/index.xml
    DESTINATION ${CMAKE_INSTALL_DOCDIR}/doxygen-html)
ENDIF()
