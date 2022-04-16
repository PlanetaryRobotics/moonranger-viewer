#remove_definitions(-DNDEBUG)

macro(remove_cxx_flag flag)
  string(REPLACE "${flag}" "" CMAKE_CXX_FLAGS_RELWITHDEBINFO "${CMAKE_CXX_FLAGS_RELWITHDEBINFO}")
endmacro()
remove_cxx_flag("-DNDEBUG")

set(libIGLIncludes ${CMAKE_CURRENT_SOURCE_DIR}/../../external/libigl/include/)
set(EigenIncludes ${CMAKE_CURRENT_SOURCE_DIR}/../../external/eigen/)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

#Build common as static library
add_library(common  STATIC
                    common/comms_interface.cc 
                    common/db.cc 
                    common/safety_gate.cc 
                    common/util.cc 
                    common/config.cc 
                    common/rotation_matrix.cc)

target_include_directories(common PUBLIC ${CMAKE_CURRENT_SOURCE_DIR}/common/)

target_link_libraries(common LINK_PUBLIC 
                    pthread
)

#Build tmap_helpers as static library
add_library(tmap_helpers  STATIC
                        tmap_helpers/decimationGrid.cc
                        tmap_helpers/lineHelpers.cc
                        tmap_helpers/planeHelpers.cc 
                        tmap_helpers/tmapOctTree.cc 
                        tmap_helpers/dynamicObstacleMap.cc 
                        tmap_helpers/matrixClass.cc 
                        tmap_helpers/obstacleMap.cc 
                        tmap_helpers/tmapKDTree.cc 
                        tmap_helpers/tmapQuadTree.cc
)

target_include_directories(tmap_helpers PUBLIC 
                ${CMAKE_CURRENT_SOURCE_DIR}/tmap_helpers/ 
                ${CMAKE_CURRENT_SOURCE_DIR}/tmap/
)

target_link_libraries(tmap_helpers LINK_PUBLIC common)

#Build tmap as static library
add_library(tmap STATIC
                    tmap/tmap_base.cc 
                    tmap/tmap_associated.cc
                    tmap/tmap_group.cc 
                    tmap/tmap_moveable.cc 
                    tmap/tmap_searchable.cc
                    tmap/tmap_cleanable.cc 
                    tmap/tmap_triangulate.cc
                    tmap/tmap_pasteable.cc 
                    tmap/tmap_icp.cc 
                    tmap/tmap_astar.cc
                    tmap/tmap_shrinkable.cc 
                    tmap/tmap_full.cc 
                    tmap/triangle.cc
)

target_include_directories(tmap PUBLIC 
                ${CMAKE_CURRENT_SOURCE_DIR}/tmap/
                ${libIGLIncludes}
                ${EigenIncludes}
)

target_link_libraries(tmap LINK_PUBLIC 
                common
                tmap_helpers
                ${OpenCV_LIBRARIES}
)

add_definitions(-DENABLE_TMAP)
add_library(rasm_viewer STATIC
                viewer/plotClouds.cc 
                viewer/glLighting.cc 
                viewer/glObject.cc 
                viewer/solidContours.cc 
                viewer/myplane.cc 
                viewer/smfUtil.cc 
                viewer/spacial.cc 
                viewer/wrl.cc 
                viewer/helpers.cc 
                viewer/shadowModel.cc 
                viewer/plotClouds_light.cc 
                viewer/plotClouds_path.cc 
                viewer/plotClouds_draw.cc 
                viewer/plotClouds_color.cc 
                viewer/plotClouds_globals.cc 
                viewer/plotClouds_file.cc 
                viewer/plotClouds_wrl.cc 
                viewer/plotClouds_edge.cc 
                viewer/plotClouds_triangulate.cc 
                viewer/plotClouds_selectTri.cc 
                viewer/plotClouds_tmap.cc
)

target_include_directories(rasm_viewer PUBLIC 
                ${CMAKE_CURRENT_SOURCE_DIR}/viewer/
)

target_link_libraries(rasm_viewer LINK_PUBLIC
                tmap
                tmap_helpers
                common
                pthread
                m
                dl
                glut
                GL
                GLU
                pthread
                Xmu
                Xi                
)

add_library(rasm INTERFACE)
target_link_libraries(rasm INTERFACE
                            common 
                            tmap 
                            tmap_helpers 
                            rasm_viewer
)

