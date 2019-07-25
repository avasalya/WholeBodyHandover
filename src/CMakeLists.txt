set(controller_SRC
  handover_controller.cpp
  handover_trajectories.cpp
  handover_approachObject.cpp
)

set(controller_HDR
  handover_controller.h
  handover_trajectories.h
  handover_approachObject.h
)


add_library(${PROJECT_NAME} SHARED ${controller_SRC} ${controller_HDR})

set_target_properties(${PROJECT_NAME} PROPERTIES COMPILE_FLAGS "-DMC_CONTROL_EXPORTS" PREFIX "")

# pkg_config_use_dependency(${PROJECT_NAME} mc_rtc)
# target_link_libraries(${PROJECT_NAME} mc_control_fsm)
target_link_libraries(${PROJECT_NAME} mc_rtc::mc_control_fsm)

install(TARGETS ${PROJECT_NAME} DESTINATION ${MC_RTC_LIBDIR}/mc_controller)
