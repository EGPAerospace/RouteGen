# Adding subdirectories
add_subdirectory(Custom)
find_package(Paradiseo REQUIRED)


# Setting data for Eigen header-only library
include_directories("Eigen/")
include_directories("Eigen/src/")
