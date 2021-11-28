project (bezier)

# Set current folders source files
set(SOURCE_FILES
  bezier.hpp
  bezier.cpp
)

# Creating a static library from previous files
add_library(bezier STATIC ${SOURCE_FILES})
target_include_directories(bezier PUBLIC "../../../ext/Custom/")
target_link_directories(bezier PUBLIC "../../../build/ext/Custom/")
target_link_libraries(bezier custom)

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS bezier DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES bezier.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
