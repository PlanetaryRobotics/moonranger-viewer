find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_definitions(-D_LINUX)
add_definitions(-D_x64)

add_library(moonranger_arc_model SHARED
                        moonranger_arc_model/factory.cc
                        moonranger_arc_model/moonranger_arc_model.cc 
)

target_include_directories(moonranger_arc_model PUBLIC
                        ${CMAKE_CURRENT_SOURCE_DIR}/moonranger_arc_model/
                        ${ExternalLinuxIncludes}
)

target_link_libraries(moonranger_arc_model LINK_PUBLIC
                        common
)
