#include "random_utils.hpp"

/*
 * namespace : random_utils
 * method : interval_random_generator()
 *
 */

double random_utils::interval_random_generator(double min_val,
                                               double max_val){

    double rand_number;

    std::random_device rd;
    std::default_random_engine generator(rd()); // rd() provides a random seed
    std::uniform_real_distribution<double> distribution(min_val, max_val);

    rand_number = distribution(generator);

    return rand_number;

};
