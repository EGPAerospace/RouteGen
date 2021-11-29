#include <gtest/gtest.h>

#define private public
#include "../../../src/modeler/modeler.hpp"


/*****************************************************************************
* Tested namespace  : modeler
* Tested class      : Route
* Testing name      : RouteParametersTesting
* Testing objective : Test that Route bezier-dubins methods give correct
*                     results
******************************************************************************/

class RouteParametersTesting : public testing::Test{
    protected:

        std::unique_ptr<modeler::Route> route;


        virtual void SetUp(){


        };

        virtual void TearDown(){


        };

};
