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


if(DEFINED ENV{Qt5_DIR})
  set(CODYCO_QT_CMAKE_PATH $ENV{Qt5_DIR})
  message(STATUS "Found CODYCO_QT_CMAKE_PATH: " ${CODYCO_QT_CMAKE_PATH})

  find_package(Qt5Widgets PATHS ${CODYCO_QT_CMAKE_PATH}../Qt5Widgets REQUIRED)
  find_package(Qt5PrintSupport PATHS ${CODYCO_QT_CMAKE_PATH}../Qt5PrintSupport REQUIRED)
  find_package(Qt5Core ${CODYCO_QT_CMAKE_PATH}../Qt5Core REQUIRED)
  find_package(Qt5Gui ${CODYCO_QT_CMAKE_PATH}../Qt5Gui REQUIRED)
else()
  find_package(Qt5Widgets REQUIRED)
  find_package(Qt5PrintSupport REQUIRED)
  find_package(Qt5Core REQUIRED)
  find_package(Qt5Gui REQUIRED)
endif()

find_package(YARP)
find_package(ICUB)
find_package(Boost 1.50 COMPONENTS system filesystem chrono thread REQUIRED)
find_package(TinyXML QUIET)

if (NOT TinyXML_FOUND)
  find_package(PkgConfig REQUIRED)
  PKG_CHECK_MODULES(TinyXML tinyxml REQUIRED)
endif()


set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${YARP_MODULE_PATH})
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} ${ICUB_MODULE_PATH})
include(iCubHelpers)

set(CMAKE_EXE_LINKER_FLAGS "${CMAKE_EXE_LINKER_FLAGS} ${ICUB_LINK_FLAGS}")

FILE(GLOB controller_source ${CMAKE_CURRENT_SOURCE_DIR}/src/controlModule/*.cpp)
FILE(GLOB controller_header ${CMAKE_CURRENT_SOURCE_DIR}/include/controlModule/*.h)

FILE(GLOB gui_source ${CMAKE_CURRENT_SOURCE_DIR}/src/guiModule/*.cpp )
FILE(GLOB gui_header ${CMAKE_CURRENT_SOURCE_DIR}/include/guiModule/*.h)
FILE(GLOB gui_forms ${CMAKE_CURRENT_SOURCE_DIR}/src/guiModule/*.ui )

# For MSVS
SOURCE_GROUP("Source Files" FILES ${controller_source} ${gui_source})
SOURCE_GROUP("Header Files" FILES ${controller_header} ${gui_header})

# For QtCreator
SOURCE_GROUP("SOURCES" FILES ${controller_source} ${gui_source})
SOURCE_GROUP("HEADERS" FILES ${controller_header} ${gui_header})
SOURCE_GROUP("FORMS" FILES ${gui_forms})

include_directories(${PROJECT_SOURCE_DIR}/include
                    ${ICUB_INCLUDE_DIRS}
                    ${YARP_INCLUDE_DIRS}
                    ${Boost_INCLUDE_DIR}
                    ${Qt5Widgets_INCLUDE_DIRS}
                    ${Qt5PrintSupport_INCLUDE_DIRS}
                    ${TinyXML_INCLUDE_DIRS})

add_executable(${PROJECTNAME_CONTROLLER} ${controller_source} ${controller_header})
target_link_libraries(${PROJECTNAME_CONTROLLER}
    ${YARP_LIBRARIES}
    ${Boost_LIBRARIES}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5PrintSupport_LIBRARIES} )

message("
    gui_forms
    ${gui_forms}
    ")

add_executable(${PROJECTNAME_GUI} ${gui_source} ${gui_header} ${gui_forms})
target_link_libraries(${PROJECTNAME_GUI}
    ${Qt5Widgets_LIBRARIES}
    ${Qt5Core_LIBRARIES}
    ${Qt5Gui_LIBRARIES}
    ${Qt5PrintSupport_LIBRARIES}
    ${YARP_LIBRARIES}
    ${TinyXML_LIBRARIES}
    ${Boost_LIBRARIES})

INSTALL(TARGETS ${PROJECTNAME_CONTROLLER} DESTINATION bin)
INSTALL(TARGETS ${PROJECTNAME_GUI} DESTINATION bin)
