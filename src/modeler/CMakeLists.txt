project (modeler)

#Adding subdirectories
add_subdirectory (dubins)
add_subdirectory (bezier)

# Set current folder source files
set(SOURCE_FILES
  modeler.hpp
  modeler.cpp
)

#Creating a static library from previous files
add_library(modeler STATIC ${SOURCE_FILES})

# Linking created libraries in subfolders
target_link_libraries (
  modeler dubins bezier
)


# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS modeler DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES modeler.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
