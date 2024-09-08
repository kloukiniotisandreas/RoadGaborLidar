cmake_minimum_required(VERSION 2.8)
set(SOLUTIONTITLE RoadGaborLidar CACHE TYPE STRING)
project(${SOLUTIONTITLE})

file(GLOB FILES_SRC
    "src/*.cc"
    "src/*.hpp"
    "src/*.cpp"


)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++14")
SET(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -pthread")
set(VVRFRAMEWORK_DIR "" CACHE PATH "Location of VVR Framework")



find_package( OpenCV REQUIRED )
find_package( PCL 1.9 REQUIRED )
find_package(Boost REQUIRED COMPONENTS system filesystem thread date_time)

include_directories(${CMAKE_SOURCE_DIR})
include_directories( ${OpenCV_INCLUDE_DIRS} ${Boost_INCLUDE_DIRS} )


add_executable(${SOLUTIONTITLE} ${FILES_SRC})
add_definitions( ${PCL_DEFINITIONS} )



target_link_libraries(
        ${SOLUTIONTITLE}

)

target_link_libraries( ${SOLUTIONTITLE} ${OpenCV_LIBS} ${Boost_LIBRARIES} )

if( PCL_FOUND )
  # [C/C++]>[General]>[Additional Include Directories]
  include_directories( ${PCL_INCLUDE_DIRS} )

  # [C/C++]>[Preprocessor]>[Preprocessor Definitions]
  add_definitions( ${PCL_DEFINITIONS} )

  # For Use Not PreCompiled Features
  add_definitions( -DPCL_NO_PRECOMPILE )

  # [Linker]>[General]>[Additional Library Directories]
  link_directories( ${PCL_LIBRARY_DIRS} )

  # [Linker]>[Input]>[Additional Dependencies]
  target_link_libraries( ${SOLUTIONTITLE}  ${PCL_LIBRARIES} )
endif()


# Add test executable
add_executable(test_sample tests/test_sample.cpp)
target_link_libraries(test_sample ${Boost_LIBRARIES})

# Enable testing and add tests
# enable_testing()
# add_test(NAME test_sample COMMAND test_sample)
