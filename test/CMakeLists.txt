project (testing)

# Add googletest library directory
find_package(PkgConfig)
pkg_search_module(GTEST REQUIRED gtest)

# add_subdirectory("ext/googletest" EXCLUDE_FROM_ALL)

# Add required headers of the project itself
include_directories(${ROUTE_HEADERS_DIR})

# Add required headers for google test
# include_directories("ext/googletest/googletest/include/gtest")

# Add required source files
set(SOURCE_FILES
  main.cpp
  src/modeler/dubins_test.cpp
  src/modeler/bezier_test.cpp
)

add_executable(testing ${SOURCE_FILES})
target_link_libraries(testing constrainer generator modeler route ${GTEST_LDFLAGS})

install(TARGETS testing DESTINATION build/bin)
