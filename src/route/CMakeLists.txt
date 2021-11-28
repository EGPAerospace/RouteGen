project (route)

# Set current folders source files
set(SOURCE_FILES
  route.hpp
  route.cpp
)

#Creating a static library from previous files
add_library(route STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS route DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES route.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
