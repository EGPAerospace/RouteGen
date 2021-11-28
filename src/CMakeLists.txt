project (routegen)

add_subdirectory (route)
add_subdirectory (constrainer)
add_subdirectory (generator)
add_subdirectory (modeler)
add_subdirectory (mapper)

# Set current folders source files
set(SOURCE_FILES
  main.cpp
)

# Creating the future executable file
add_executable (routegen ${SOURCE_FILES})

# Linking created libraries in subfolders
target_link_libraries (routegen
  route
  constrainer
  generator
  modeler
  mapper
  ${PARADISEO_LIBRARIES})

# Path to install generated binary
install(TARGETS routegen DESTINATION ${ROUTEGEN_INSTALL_BIN_DIR})
