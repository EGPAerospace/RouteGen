project (custom)

# Set current folder source files
set(SOURCE_FILES
  combinatorics.hpp
  combinatorics.cpp
  geometry.hpp
  geometry.cpp
  random_utils.cpp
  random_utils.hpp
)

#Creating a static library from previous files
add_library(custom STATIC ${SOURCE_FILES})
