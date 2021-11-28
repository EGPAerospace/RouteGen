project(optimizer)

# Set current folders source files
set(SOURCE_FILES
  optimizer.hpp
  optimizer.cpp
)

# Creating a static library from previous files
add_library(optimizer STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS optimizer DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES optimizer.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
