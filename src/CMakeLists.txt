# set(controller_SRC
#   SbsController.cpp
#   ModifiedTasks.cpp
# )

# set(controller_HDR
#   ${PROJECT_SOURCE_DIR}/include/SbsController/SbsController.h
#   ${PROJECT_SOURCE_DIR}/include/SbsController/ModifiedTasks.h
# )
option(USE_FALCON "Use falcon" OFF)

add_library(${PROJECT_NAME} SHARED
  SbsController.cpp
  ModifiedTasks.cpp
  )
target_link_libraries(${PROJECT_NAME} PUBLIC mc_rtc::mc_control_fsm mc_rtc::mc_rtc_ros)
target_link_libraries(${PROJECT_NAME} PUBLIC ${catkin_LIBRARIES})

if(USE_FALCON)
  target_compile_definitions(${PROJECT_NAME} PRIVATE USE_FALCON)
endif()

install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR} EXPORT ${TARGETS_EXPORT_NAME})

add_controller(${PROJECT_NAME}_controller lib.cpp "")
set_target_properties(${PROJECT_NAME}_controller PROPERTIES OUTPUT_NAME "${PROJECT_NAME}")
target_link_libraries(${PROJECT_NAME}_controller PUBLIC ${PROJECT_NAME})




