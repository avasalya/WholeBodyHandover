# configure_file(datapath.h.in "${CMAKE_BINARY_DIR}/include/datapath.h")

set(FSM_STATES_INSTALL_PREFIX "${MC_RTC_LIBDIR}/mc_controller/handovercontroller/states")

include_directories(${CMAKE_CURRENT_SOURCE_DIR}/../src)

macro(add_fsm_state state_name state_SRC state_HDR)
	add_library(${state_name} SHARED  ${state_SRC} ${state_HDR})
	set_target_properties(${state_name} PROPERTIES PREFIX "")
# 	pkg_config_use_dependency(${state_name} mc_rtc)
	target_link_libraries(${state_name} handovercontroller)
	install(TARGETS ${state_name} DESTINATION ${FSM_STATES_INSTALL_PREFIX})
	install(FILES ${state_HDR} DESTINATION ${CMAKE_INSTALL_PREFIX}/include/mc_control/fsm/states)
endmacro()


macro(add_fsm_handover_state state_name)
  add_fsm_state(${state_name} ${state_name}.cpp ${state_name}.h)
endmacro()

add_fsm_handover_state(StartMocapStep)
target_link_libraries(StartMocapStep cortex_sdk pthread m)

# matlibplot #
# target_link_libraries(StartMocapStep ${PYTHON_LIBRARIES})
