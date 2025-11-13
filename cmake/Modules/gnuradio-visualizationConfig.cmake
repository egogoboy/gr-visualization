find_package(PkgConfig)

PKG_CHECK_MODULES(PC_GR_VISUALIZATION gnuradio-visualization)

FIND_PATH(
    GR_VISUALIZATION_INCLUDE_DIRS
    NAMES gnuradio/visualization/api.h
    HINTS $ENV{VISUALIZATION_DIR}/include
        ${PC_VISUALIZATION_INCLUDEDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/include
          /usr/local/include
          /usr/include
)

FIND_LIBRARY(
    GR_VISUALIZATION_LIBRARIES
    NAMES gnuradio-visualization
    HINTS $ENV{VISUALIZATION_DIR}/lib
        ${PC_VISUALIZATION_LIBDIR}
    PATHS ${CMAKE_INSTALL_PREFIX}/lib
          ${CMAKE_INSTALL_PREFIX}/lib64
          /usr/local/lib
          /usr/local/lib64
          /usr/lib
          /usr/lib64
          )

include("${CMAKE_CURRENT_LIST_DIR}/gnuradio-visualizationTarget.cmake")

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(GR_VISUALIZATION DEFAULT_MSG GR_VISUALIZATION_LIBRARIES GR_VISUALIZATION_INCLUDE_DIRS)
MARK_AS_ADVANCED(GR_VISUALIZATION_LIBRARIES GR_VISUALIZATION_INCLUDE_DIRS)
