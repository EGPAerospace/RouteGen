project (dubins)

# Include additional headers used by dubins
# include_directories("../bezier")

# Set current folder source files
set(SOURCE_FILES
  dubins.hpp
  dubins.cpp
)

#Creating a static library from previous files
add_library(dubins STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS dubins DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES dubins.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
