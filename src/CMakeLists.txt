set(controller_SRC
  SbsController.cpp
  ModifiedTasks.cpp
)

set(controller_HDR
  SbsController.h
  ModifiedTasks.h
)

find_package(catkin REQUIRED COMPONENTS
  roscpp
  geometry_msgs
)
catkin_package(CATKIN_DEPENDS roscpp geometry_msgs)
include_directories(SYSTEM ${catkin_INCLUDE_DIRS})
link_directories(
  ${catkin_LIBRARIES}
)

add_controller(${PROJECT_NAME} ${controller_SRC} ${controller_HDR})
set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DSbsController_EXPORTS")
# find_library(nifalcon usb-1.0)
# target_link_libraries(${PROJECT_NAME} PUBLIC nifalcon usb-1.0)
target_link_libraries(${PROJECT_NAME} PUBLIC  ${catkin_LIBRARIES})
