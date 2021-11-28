project (constrainer)

# Set current folders source files
set(SOURCE_FILES
  constrainer.hpp
  constrainer.cpp
)

#Creating a static library from previous files
add_library(constrainer STATIC ${SOURCE_FILES})

# Storing objects (targets) and files (headers) in the corresponding directories
install(TARGETS constrainer DESTINATION ${ROUTEGEN_INSTALL_LIB_DIR})
install(FILES constrainer.hpp DESTINATION ${ROUTEGEN_INSTALL_INCLUDE_DIR})
