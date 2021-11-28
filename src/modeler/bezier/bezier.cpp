#include "bezier.hpp"


/*
 * namespace : bezier
 * class : Bezier
 * method : control_coordinates_conversion()
 *
 */

void bezier::Bezier::control_coordinates_conversion(){

    typedef std::vector<std::vector<double>> data;
    int num_control_coords = (*this->control_coordinates).size();
    std::shared_ptr<data> converted_coordinates;

    converted_coordinates = std::make_shared<data>(3, std::vector<double>(num_control_coords));

    for (int i = 0; i < num_control_coords; i++){

        (*converted_coordinates)[0][i] = (*this->control_coordinates)[i][0];
        (*converted_coordinates)[1][i] = (*this->control_coordinates)[i][1];
        (*converted_coordinates)[2][i] = (*this->control_coordinates)[i][2];

    };

    /* Deleting old coordinates pointer and updating with new one */
    (this->control_coordinates).reset();
    this->control_coordinates = converted_coordinates;

};


/*
 * namespace : bezier
 * class : Bezier
 * method : pos_percentage_vector()
 *
 */

std::shared_ptr<std::vector<double>> bezier::Bezier::pos_percentage_vector(){

    typedef std::vector<double> data;

    double perc_value {0.0};
    std::shared_ptr<data> perc_vec;
    data perc_vec_container(this->point_resolution);
    data::iterator it;

    perc_vec_container[0] = 0.0;
    for (it = perc_vec_container.begin() + 1; it < perc_vec_container.end(); it++){

        perc_value += 1.0 / (double(point_resolution) - 1.0);
        (*it) = perc_value;

    };


    perc_vec = std::make_shared<data>(perc_vec_container);

    return perc_vec;

};


/*
 * namespace : bezier
 * class : Bezier
 * method : bernstein_polynomial()
 *
 */

double bezier::Bezier::bernstein_polynomial(int total_points,
                                            int num_point,
                                            double pos_percentage){

    double bernstein;
    int bin_coeff;
    double term_1, term_2;
    std::shared_ptr<combinatorics::BinomialCoeff> binomial = std::make_shared<combinatorics::BinomialCoeff>();

    bin_coeff = binomial->calculate(total_points, num_point);
    term_1 = pow((1.0 - pos_percentage), (total_points - num_point));
    term_2 = pow(pos_percentage, num_point);

    bernstein = (double)bin_coeff * term_1 * term_2;

    return bernstein;

};


/*
 * namespace : bezier
 * class : Bezier
 * method : calculated_coordinates_conversion()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> bezier::Bezier::calculated_coordinates_conversion(std::vector<std::vector<double>>& coordinates){

    typedef std::vector<std::vector<double>> data;
    int num_control_coords = coordinates[0].size();
    std::shared_ptr<data> converted_coordinates;

    converted_coordinates = std::make_shared<data>(num_control_coords, std::vector<double>(3));

    for (int i = 0; i < num_control_coords; i++){

        (*converted_coordinates)[i][0] = coordinates[0][i];
        (*converted_coordinates)[i][1] = coordinates[1][i];
        (*converted_coordinates)[i][2] = coordinates[2][i];

    };

    return converted_coordinates;

};


/*
 * namespace : bezier
 * class : Bezier
 * method : calculate_coordinates()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> bezier::Bezier::calculate_coordinates(){

    typedef std::vector<std::vector<double>> data;

    int total_points;
    double bernstein;
    std::shared_ptr<data> coords;
    data coords_container(3);
    std::shared_ptr<std::vector<double>> perc_vec;
    std::vector<double>::iterator it_perc;
    std::vector<double> summation(3);

    /* Converting input control_coordinates into suitable vector format */
    control_coordinates_conversion();

    /* Get the number of control points */
    total_points = (*this->control_coordinates)[0].size();

    /* Calculating position percentage vector */
    perc_vec = pos_percentage_vector();

    for (it_perc = (*perc_vec).begin(); it_perc < (*perc_vec).end(); it_perc++){

        for (int i = 0; i < total_points; i++){
            bernstein = bernstein_polynomial(total_points, i, (*it_perc));
            summation[0] += bernstein * (*this->control_coordinates)[0][i];
            summation[1] += bernstein * (*this->control_coordinates)[1][i];
            summation[2] += bernstein * (*this->control_coordinates)[2][i];
        };

        bernstein = bernstein_polynomial(total_points, total_points, (*it_perc));
        summation[0] += bernstein * (*this->control_coordinates)[0].back();
        summation[1] += bernstein * (*this->control_coordinates)[1].back();
        summation[2] += bernstein * (*this->control_coordinates)[2].back();


        /* Storing resulting coordinates for a position percentage */
        coords_container[0].push_back(summation[0]);
        coords_container[1].push_back(summation[1]);
        coords_container[2].push_back(summation[2]);

        summation[0] = 0.0;
        summation[1] = 0.0;
        summation[2] = 0.0;


    };

    coords = calculated_coordinates_conversion(coords_container);

    return coords;
};
