#
# project: serdp_gpmf
#
cmake_minimum_required(VERSION 3.5)
project(serdp_gpmf)

if( FIPS_CONFIG AND NOT FIPS_IMPORT )
  get_filename_component(FIPS_ROOT_DIR "../fips" ABSOLUTE)
  include("${FIPS_ROOT_DIR}/cmake/fips.cmake")

  fips_setup()
else()
  find_package(catkin QUIET)
endif()

## C++ flags applied to all builds
set( CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

if (FIPS_CONFIG)

  fips_begin_module( serdp_gpmf )

    fips_src( lib/ )

    fips_deps( gpmf g3logger)

    fips_include_directories(
      ${CMAKE_SOURCE_DIR}/include
     )

  fips_end_module()

  fips_add_subdirectory( tools/ )
  if( NOT FIPS_IMPORT )

    # if( FIPS_UNITTESTS )
    #   gtest_begin( serdp_gpmf )
    #   fips_src( test/unit/ )
    #   fips_deps(  )
    #   gtest_end()
    # endif()

  endif()

  fips_finish()

else()

    find_package(catkin REQUIRED
      roscpp
      gpmf
      g3log_catkin
      liboculus REQUIRED
    )

    catkin_package(
    	CATKIN_DEPENDS liboculus gpmf
      INCLUDE_DIRS include
      LIBRARIES serdp_gpmf_LIB
    )

    include_directories(
        ${catkin_INCLUDE_DIRS}
        include
        ros/include/
      )


    file(GLOB serdp_gpmf_SRCS
        lib/*.cpp
      )

    add_library(serdp_gpmf ${serdp_gpmf_SRCS})
    target_link_libraries(serdp_gpmf ${catkin_LIBRARIES})

    install(TARGETS serdp_gpmf
          ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
          RUNTIME DESTINATION ${CATKIN_GLOBAL_BIN_DESTINATION})

    ## Install headers
    install(DIRECTORY  include/${PROJECT_NAME}/
        DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
        FILES_MATCHING PATTERN "*.hpp" PATTERN "*.h"
        PATTERN ".git" EXCLUDE)

endif()
