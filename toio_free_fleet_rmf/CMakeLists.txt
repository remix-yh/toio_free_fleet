cmake_minimum_required(VERSION 3.5)
project(toio_free_fleet_rmf)

find_package(ament_cmake REQUIRED)

# Install the static assets verbatim — map background image, building.yaml,
# launch xml, fleet/zenoh configs.
install(DIRECTORY config/
  DESTINATION share/${PROJECT_NAME}/config
)
install(DIRECTORY launch/
  DESTINATION share/${PROJECT_NAME}/launch
)
install(DIRECTORY maps/
  DESTINATION share/${PROJECT_NAME}/maps
)

# Generate nav_graphs/<N>.yaml from every *.building.yaml under maps/ at build
# time. Mirrors free_fleet_examples — see its CMakeLists.txt for the pattern.
file(GLOB_RECURSE traffic_editor_paths "maps/*.building.yaml")

foreach(path ${traffic_editor_paths})
  string(REGEX REPLACE "\\.[^.]*\.[^.]*$" "" no_extension_path ${path})
  string(REGEX MATCH "[^\/]+$" world_name ${no_extension_path})

  set(map_path ${path})
  set(output_world_name ${world_name})
  set(output_dir ${CMAKE_CURRENT_BINARY_DIR}/maps/${output_world_name})
  set(output_nav_graphs_dir ${output_dir}/nav_graphs/)
  set(output_nav_graphs_phony ${output_nav_graphs_dir}/phony)

  add_custom_command(
    OUTPUT ${output_nav_graphs_phony}
    COMMAND ros2 run rmf_building_map_tools building_map_generator nav
            ${map_path} ${output_nav_graphs_dir}
    DEPENDS ${map_path}
  )

  add_custom_target(generate_${output_world_name}_nav_graphs ALL
    DEPENDS ${output_nav_graphs_phony}
  )

  install(DIRECTORY ${output_dir}
    DESTINATION share/${PROJECT_NAME}/maps
  )
endforeach()

ament_package()
