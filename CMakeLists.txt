CMAKE_MINIMUM_REQUIRED(VERSION 2.8.11)


set(PROJECTNAME_CONTROLLER pidTunerController)
set(PROJECTNAME_GUI pidTunerGui)
project(pidTuner)


set(CMAKE_CXX_FLAGS "-fPIC")


# Tell CMake to run moc when necessary:
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
# As moc files are generated in the binary dir, tell CMake
# to always look for includes there:
set(CMAKE_INCLUDE_CURRENT_DIR ON)

if(DEFINED ENV{Qt5_DIR})
  set(CODYCO_QT_CMAKE_PATH $ENV{Qt5_DIR})
  message("Found CODYCO_QT_CMAKE_PATH: " ${CODYCO_QT_CMAKE_PATH})

  find_package(Qt5Widgets PATHS ${CODYCO_QT_CMAKE_PATH}../Qt5Widgets REQUIRED)
  find_package(Qt5PrintSupport PATHS ${CODYCO_QT_CMAKE_PATH}../Qt5PrintSupport REQUIRED)

else()
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5PrintSupport REQUIRED)

endif()


find_package(YARP)
find_package(ICUB)
find_package(Boost COMPONENTS system filesystem REQUIRED)
find_package(TinyXML REQUIRED)

set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

LINK_DIRECTORIES(${Boost_LIBRARY_DIRS})

FILE(GLOB controller_source ./src/controlModule/*.cpp)
FILE(GLOB controller_header ./include/controlModule/*.h)

FILE(GLOB gui_source ./src/guiModule/*.cpp)
FILE(GLOB gui_header ./include/guiModule/*.h)

# SOURCE_GROUP("Source Files" FILES ${controller_source} ${gui_source})
# SOURCE_GROUP("Header Files" FILES ${controller_header} ${gui_header})



include_directories(${PROJECT_SOURCE_DIR}/include/controlModule
                    ${PROJECT_SOURCE_DIR}/include/guiModule
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5PrintSupport_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS})


add_executable(${PROJECTNAME_CONTROLLER} ${controller_source} ${controller_header})
target_link_libraries(${PROJECTNAME_CONTROLLER} ${YARP_LIBRARIES} ${Boost_LIBRARIES} Qt5::Widgets Qt5::PrintSupport )#icubmod )

add_executable(${PROJECTNAME_GUI} ${gui_source} ${gui_header})
target_link_libraries(${PROJECTNAME_GUI} Qt5::Widgets Qt5::PrintSupport ${YARP_LIBRARIES} ${TinyXML_LIBRARIES})
