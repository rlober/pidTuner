CMAKE_MINIMUM_REQUIRED(VERSION 2.6)
SET(PROJECTNAME pidTuner)
PROJECT(${PROJECTNAME})

find_package(YARP)
find_package(ICUB)
find_package(Boost COMPONENTS system filesystem REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

FILE(GLOB folder_source ./src/*.cpp)
FILE(GLOB folder_header ./include/*.h)

SOURCE_GROUP("Source Files" FILES ${folder_source})
SOURCE_GROUP("Header Files" FILES ${folder_header})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS} 
                    ${YARP_INCLUDE_DIRS} 
                    ${Boost_INCLUDE_DIR} )


add_executable(${PROJECTNAME} ${folder_source} ${folder_header})

target_link_libraries(${PROJECTNAME} ${YARP_LIBRARIES} ${Boost_LIBRARIES} icubmod)
