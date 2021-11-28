#ifndef CONSTRAINER_H_
#define CONSTRAINER_H_

#include <math.h>
#include <memory>
#include <vector>

namespace constrainer{

    class AircraftDynamics{

        private:
            double speed;
            double bank_angle;
            double gravity {9.80655};

        public:
            AircraftDynamics(){};

            AircraftDynamics(double speed,
                             double bank_angle):
                speed(speed),
                bank_angle(bank_angle){};

            ~AircraftDynamics(){};

            double turn_radius();
            double turn_rate();

    };




};


#endif // CONSTRAINER_H_
