#include "geometry.hpp"

/*****************************************************************************
* namespace: utilsGeometry
* method   : check_point_inside_sphere
******************************************************************************/

int utilsGeometry::check_point_inside_sphere(std::vector<double> point_coords,
                                             std::vector<double> sphere_coords,
                                             double radius){

    double x_p, y_p, z_p;
    double x_s, y_s, z_s;
    double distance;
    int status;

    x_p = point_coords[0];
    y_p = point_coords[1];
    z_p = point_coords[2];

    x_s = sphere_coords[0];
    y_s = sphere_coords[1];
    z_s = sphere_coords[2];

    distance = sqrt(pow(x_p - x_s, 2) + pow(y_p - y_s, 2) + pow(z_p - z_s, 2));

    if (radius > distance){

        status = 1;

    } else {

        status = 0;

    };

    return status;

};
