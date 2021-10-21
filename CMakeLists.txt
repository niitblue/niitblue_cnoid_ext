if(UNIX)
  add_custom_target(
    symlink_to_niitblue_cnoid_ext ALL
    COMMAND ${CMAKE_COMMAND} -E create_symlink
    ${CMAKE_CURRENT_SOURCE_DIR} ${CNOID_BINARY_SHARE_DIR}/niitblue_cnoid_ext)
endif()

install(DIRECTORY
  model
  project
  script
  test
  DESTINATION ${CNOID_SHARE_SUBDIR}/niitblue_cnoid_ext)

add_subdirectory(controller)
