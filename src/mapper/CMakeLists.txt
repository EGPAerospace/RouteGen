project(mapper)

# Set current folders source files
set(SOURCE_FILES
  mapper.hpp
  mapper.cpp
)

# Creating a static library from previous files
add_library(mapper STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS mapper DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES mapper.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
