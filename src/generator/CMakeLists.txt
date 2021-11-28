project (generator)

# Set current folders source files
set(SOURCE_FILES
  generator.hpp
  generator.cpp
)

# Creating a static library from previous files
add_library(generator STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS generator DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES generator.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
