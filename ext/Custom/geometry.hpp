#ifndef GEOMETRY_H
#define GEOMETRY_H

#include <math.h>
#include <vector>


/*****************************************************************************
* \brief utilsGeometry is the namespace that holds all the classes and
         functions related to space geometry calculation

******************************************************************************/

namespace utilsGeometry{

    int check_point_inside_sphere(std::vector<double> point_coords,
                                  std::vector<double> sphere_coords,
                                  double radius);





}



#endif // GEOMETRY_H
