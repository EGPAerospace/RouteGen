#include "constrainer.hpp"

double constrainer::AircraftDynamics::turn_radius(){

    double turn_radius;

    turn_radius = pow(this->speed, 2) / (this->gravity * tan(this->bank_angle));

    return turn_radius;
};



double constrainer::AircraftDynamics::turn_rate(){

    double turn_rate;

    turn_rate = (this->gravity * tan(this->bank_angle)) / this->speed;

    return turn_rate;

};
