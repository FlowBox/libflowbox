cmake_minimum_required(VERSION 2.6)
#USE user-defined compile flags as specified in the override.cmake file
IF( EXISTS ${CMAKE_SOURCE_DIR}/override.cmake )
  MESSAGE("-- Using custom CXX compile flags...")
  SET(CMAKE_USER_MAKE_RULES_OVERRIDE ${CMAKE_SOURCE_DIR}/override.cmake)
ENDIF()

PROJECT(libflowbox-c++)
SET(MAJOR_VERSION 1)
SET(MINOR_VERSION 0)
SET(PATCH_VERSION 0)


####################### FUNCTIONS #############################

FUNCTION( SPLIT_VERSION_STRING _VersionString _Major _Minor _Patch )
  STRING( STRIP _VersionString ${_VersionString} )
  STRING( REGEX MATCH "^[0-9]+" _MajorProxy "${_VersionString}" )
  STRING( REGEX MATCH "[^0-9][0-9]+[^0-9]" _MinorProxy "${_VersionString}" )
  STRING( REGEX MATCH "[0-9]+" _MinorProxy "${_MinorProxy}" )
  STRING( REGEX MATCH "[0-9]+$" _PatchProxy "${_VersionString}" )
  SET( ${_Major} "${_MajorProxy}" PARENT_SCOPE )
  SET( ${_Minor} "${_MinorProxy}" PARENT_SCOPE )
  SET( ${_Patch} "${_PatchProxy}" PARENT_SCOPE )
ENDFUNCTION( SPLIT_VERSION_STRING )

FUNCTION( ASSERT_GE_VERSION _PackageName _ActualVersion _MinimumVersion )
  SPLIT_VERSION_STRING( ${_ActualVersion} _ActualMajor _ActualMinor _ActualPatch )
  SPLIT_VERSION_STRING( ${_MinimumVersion} _MinimumMajor _MinimumMinor _MinimumPatch )
  SET( _FAIL_MSG "Version conflict for package ${_PackageName}: needed at least ${_MinimumVersion} but got: ${_ActualVersion}" )

  # CMake 2.6 issue:
  # The reason for writing here stupid code and repeating my self is that cmake
  # is not able to evaluate AND expressions before OR expressions nor it does
  # not support operator precedence. Just look into the book of crap:
  # "Mastering CMake" page 38 for the most moronic way of operator precedence!
  IF( _ActualMajor LESS _MinimumMajor )
      MESSAGE( FATAL_ERROR "Assertion failed: ${_FAIL_MSG}" )
  ELSEIF( _ActualMajor EQUAL _MinimumMajor AND _ActualMinor LESS _MinimumMinor )
      MESSAGE( FATAL_ERROR "Assertion failed: ${_FAIL_MSG}" )
  ELSEIF( _ActualMajor EQUAL _MinimumMajor AND _ActualMinor EQUAL _MinimumMinor AND  _ActualPatch LESS _MinimumPatch )
      MESSAGE( FATAL_ERROR "Assertion failed: ${_FAIL_MSG}" )
  ENDIF()
ENDFUNCTION( ASSERT_GE_VERSION )


################## COMPILATION  ##############################
file( GLOB cppfiles "src/*.cc" )
set( libFlowBoxCC 	${cppfiles} )

file( GLOB headerfiles "include/*.h" )
set( headerFlowBox ${headerfiles} )


#BZIP2
find_package( BZip2 REQUIRED )

#ZLIB
find_package( ZLIB REQUIRED )

#THREADS
find_package( Threads )
if (  CMAKE_USE_PTHREADS_INIT )

else (  CMAKE_USE_PTHREADS_INIT )
	message(FATAL_ERROR "libpthread required!")
endif (  CMAKE_USE_PTHREADS_INIT )

if (UNIX)
	set( libflowbox ${libFlowBoxCC} )
	include_directories( include ${BZIP2_INCLUDE_DIR} ${ZLIB_INCLUDE_DIR})
else (UNIX)
	message(FATAL_ERROR "UNIX Platform required!")
endif (UNIX)

# Supported Build Types are:
#    * Debug (CMAKE_C_FLAGS_DEBUG)
#    * Release (CMAKE_C_FLAGS_RELEASE)
#    * RelWithDebInfo (CMAKE_C_FLAGS_RELWITHDEBINFO)
#    * MinSizeRel (CMAKE_C_FLAGS_MINSIZEREL)
# If no build type requested, default to Debug
IF (NOT CMAKE_BUILD_TYPE)
        SET (CMAKE_BUILD_TYPE Debug)
ENDIF (NOT CMAKE_BUILD_TYPE)

add_library( flowbox SHARED ${libFlowBoxCC})
target_link_libraries( flowbox pthread m ${BZIP2_LIBRARIES} ${ZLIB_LIBRARIES})
set_target_properties( flowbox PROPERTIES VERSION ${MAJOR_VERSION}.${MINOR_VERSION}.${PATCH_VERSION} SOVERSION ${MAJOR_VERSION})
install(TARGETS flowbox DESTINATION lib COMPONENT libraries)
install(FILES ${headerfiles} DESTINATION include/flowbox COMPONENT headers)


############ FOR PACKAGE CREATION #######################
#IF(EXISTS "${CMAKE_ROOT}/CPack.cmake")
INCLUDE(InstallRequiredSystemLibraries)

SET(CPACK_SET_DESTDIR "on")
SET(CPACK_PACKAGING_INSTALL_PREFIX "/tmp")
SET (BUILD_ARCH "${CMAKE_SYSTEM_PROCESSOR}")
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
        SET (CPACK_DEBIAN_PACKAGE_ARCHITECTURE "i386")
        SET (CPACK_RPM_PACKAGE_ARCHITECTURE "i686" )
        SET (BUILD_ARCH "${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
ENDIF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "i686")
IF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")
        SET (CPACK_DEBIAN_PACKAGE_ARCHITECTURE "amd64")
        SET (CPACK_RPM_PACKAGE_ARCHITECTURE "x86_64" )
        SET (BUILD_ARCH "${CPACK_DEBIAN_PACKAGE_ARCHITECTURE}")
ENDIF (${CMAKE_SYSTEM_PROCESSOR} MATCHES "x86_64")

SET(CPACK_GENERATOR "DEB;RPM;STGZ;ZIP")
SET (CPACK_RESOURCE_FILE_LICENSE "${CMAKE_SOURCE_DIR}/COPYING")
SET(CPACK_PACKAGE_DESCRIPTION "FlowBox C++ library")
SET(CPACK_PACKAGE_DESCRIPTION_SUMMARY "This is the FlowBox library for processing network flow data in various flow formats (currently only Cisco Netflow v5 and v9)")
SET(CPACK_PACKAGE_VENDOR "FlowBox")
SET(CPACK_PACKAGE_CONTACT "info@flowbox.info")
SET(CPACK_PACKAGE_VERSION_MAJOR "${MAJOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_MINOR "${MINOR_VERSION}")
SET(CPACK_PACKAGE_VERSION_PATCH "${PATCH_VERSION}")
SET(CPACK_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}_${BUILD_ARCH}")
SET(CPACK_SOURCE_PACKAGE_FILE_NAME "${CMAKE_PROJECT_NAME}_${MAJOR_VERSION}.${MINOR_VERSION}.${CPACK_PACKAGE_VERSION_PATCH}_${BUILD_ARCH}")
SET(CPACK_DEBIAN_PACKAGE_DEPENDS "libc6 (>= 2.7), libgcc1 (>= 1:4.3.0), libbz2-1.0 (>=1.0), zlib1g (>=2.0.0), gcc, g++, cmake (>= 2.6.0)")
SET(CPACK_DEBIAN_PACKAGE_PRIORITY "optional")
SET(CPACK_DEBIAN_PACKAGE_SECTION "devel")

SET(CPACK_COMPONENTS_ALL Libraries ApplicationData)
INCLUDE(CPack)

#ENDIF(EXISTS "${CMAKE_ROOT}/CPack.cmake")
