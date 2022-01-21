########################################################################
# Cmake module for finding PugiXML
#
# The following variabled will be defined:
#
# Find the pugixml XML parsing library.
#
# Sets the usual variables expected for find_package scripts:
#
#  PUGI_XML_FOUND  - true if pugixml was found.
#  PUGI_XML_INCLUDE_DIRS  - header location
#  PUGI_XML_LIBRARIES  - library to link against
#

#PUGI_XML
find_path(PUGI_XML_INCLUDE_DIRS pugixml.hpp
	  PATHS /usr/include
         /user/local/include)

if(PUGI_XML_INCLUDE_DIRS)
   find_library (PUGI_XML_LIBRARIES
                NAMES pugixml libpugixml
                PATHS /usr/lib
                      /usr/local/lib
                      /user/local/lib)
   if(PUGI_XML_LIBRARIES)
      set(PUGI_XML_FOUND true)
   endif(PUGI_XML_LIBRARIES)
endif(PUGI_XML_INCLUDE_DIRS)

# Support the REQUIRED and QUIET arguments, and set PUGIXML_FOUND if found.
include (FindPackageHandleStandardArgs)
find_package_handle_standard_args (PugiXML REQUIRED_VARS
                                  PUGI_XML_LIBRARIES
                                  PUGI_XML_INCLUDE_DIRS)

if (PUGI_XML_FOUND)
    set (PUGIXML_LIBRARIES ${PUGIXML_LIBRARY})
    message (STATUS "Found PugiXML includes on: ${PUGI_XML_INCLUDE_DIRS}")
    message (STATUS "Found PugiXML libraries on: ${PUGI_XML_LIBRARIES}")
else (PUGI_XML_FOUND)
    message (STATUS "No PugiXML found")
endif(PUGI_XML_FOUND)

mark_as_advanced (PUGI_XML_INCLUDE_DIRS PUGI_XML_LIBRARIES)

