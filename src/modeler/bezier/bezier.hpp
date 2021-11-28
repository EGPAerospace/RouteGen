#ifndef BEZIER_H_
#define BEZIER_H_

#include <math.h>
#include <memory>
#include <vector>

#include "../../../ext/Custom/combinatorics.hpp"


namespace bezier{

/*
 * \class Bezier
 *
 * \brief Interface to calculate a Bezier curve
 *
 */

    class Bezier{

        private:
            int point_resolution;
            std::shared_ptr<std::vector<std::vector<double>>> control_coordinates;

            void control_coordinates_conversion();

            std::shared_ptr<std::vector<double>> pos_percentage_vector();

            double bernstein_polynomial(int total_points,
                                        int num_point,
                                        double pos_percentage);

            std::shared_ptr<std::vector<std::vector<double>>> calculated_coordinates_conversion(std::vector<std::vector<double>>& coordinates);

        public:
            Bezier(){};

            Bezier(int point_resolution,
                   std::shared_ptr<std::vector<std::vector<double>>> control_coordinates):
                point_resolution(point_resolution),
                control_coordinates(control_coordinates){};

            ~Bezier(){};

            std::shared_ptr<std::vector<std::vector<double>>> calculate_coordinates();

    };

};


#endif // BEZIER_H_

//TODO
//1. Refactor control_coordinates_conversion() and put it on a generic external library
//   folder. It is used in other files. Is a generic function used in other files.
