cmake_minimum_required (VERSION 2.6 FATAL_ERROR)

project      (pcl-visualizer)
find_package (Qt5Widgets)
#find_package (Qt5 COMPONENT Core REQUIRED)
find_package (VTK REQUIRED)
find_package (PCL 1.7.1 REQUIRED)

if( NOT CMAKE_BUILD_TYPE )
  set( CMAKE_BUILD_TYPE Release CACHE STRING
       "Choosing Release build type - the fastest option."
       FORCE )
endif()

find_package(OpenMP)
if (OPENMP_FOUND)
    set (CMAKE_C_FLAGS "${CMAKE_C_FLAGS} ${OpenMP_C_FLAGS}")
    set (CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} ${OpenMP_CXX_FLAGS}")
endif()

include_directories (${PCL_INCLUDE_DIRS})
link_directories    (${PCL_LIBRARY_DIRS})
add_definitions     (${PCL_DEFINITIONS})

include_directories ("${PROJECT_SOURCE_DIR}/include")

set  (project_SOURCES src/main.cpp src/pclviewer.cpp src/voxSAC.cpp src/regiongrowing.cpp src/preanalysis.cpp src/regions.cpp src/segmentPatch.cpp src/splitmerge.cpp src/recognition.cpp src/planeshape.cpp src/Stairs.cpp src/StairVector.cpp)
set  (project_HEADERS include/stairs/pclviewer.h)
set  (project_FORMS   src/pclviewer.ui)
# set  (VTK_LIBRARIES   vtkRendering vtkGraphics vtkHybrid QVTK)

QT5_WRAP_CPP (project_HEADERS_MOC   ${project_HEADERS})
QT5_WRAP_UI  (project_FORMS_HEADERS ${project_FORMS})

#INCLUDE         (${QT_USE_FILE})
ADD_DEFINITIONS (${QT_DEFINITIONS})

ADD_EXECUTABLE  (pcl_visualizer ${project_SOURCES}
                                ${project_FORMS_HEADERS}
                                ${project_HEADERS_MOC})

TARGET_LINK_LIBRARIES (pcl_visualizer ${QT_LIBRARIES} ${PCL_LIBRARIES} ${VTK_LIBRARIES})

#set  (project_HEADERS include/stairs/referenceSegmentation.h)
#ADD_EXECUTABLE  (refSeg src/referenceSegmentation.cpp
#                                ${sr_FORMS_HEADERS})
#
#TARGET_LINK_LIBRARIES (refSeg ${PCL_LIBRARIES})

