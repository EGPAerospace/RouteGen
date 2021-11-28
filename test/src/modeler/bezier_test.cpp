#include "../../ext/googletest/googletest/include/gtest/gtest.h"

#define private public
#include "../../../src/modeler/bezier/bezier.hpp"


/*****************************************************************************
* Tested namespace  : bezier
* Tested class      : Bezier
* Testing name      : OutputCoordinatesTesting
* Testing objective : Test Bezier class for the classical 3D coordinates
******************************************************************************/

class BezierTesting : public testing::Test{
    protected:

        typedef std::vector<std::vector<double>> data;
        std::unique_ptr<bezier::Bezier> bezier_curve;
        std::shared_ptr<data> control_coordinates;

        virtual void SetUp(){
            data control_coords_vec {
                                      {0.0, 0.0, 0.0},
                                      {30.0, 50.0, 0.0},
                                      {100.0, 100.0, 0.0}
                                    };

            control_coordinates = std::make_shared<data>(control_coords_vec);
            bezier_curve = std::make_unique<bezier::Bezier>(5, control_coordinates);

        };

        virtual void TearDown(){

        };

};


TEST_F(BezierTesting, control_coordinates_conversion){

    data result {
                  {0.0, 30.0, 100.0},
                  {0.0, 50.0, 100.0},
                  {0.0, 0.0, 0.0}
                };

    double error {1.0e-2};

    bezier_curve->control_coordinates_conversion();

    for(int i = 0; i < 3; ++i){
        for(int j = 0; j < 3; ++j){
            ASSERT_NEAR((*bezier_curve->control_coordinates)[i][j], result[i][j], error);
        };
    };
};


TEST_F(BezierTesting, pos_percentage_vector_is_ok){

    std::vector<double> result {0.0, 0.25, 0.5, 0.75, 1.0};
    std::shared_ptr<std::vector<double>> perc_vec;
    double error { 1.0e-2 };

    perc_vec = bezier_curve->pos_percentage_vector();

    for(int i = 0; i < 5; ++i){
        ASSERT_NEAR((*perc_vec)[i], result[i], error);
    };

};


TEST_F(BezierTesting, bernstein_polynomial_k_0_is_ok){

    double result { 0.729 };
    double error { 1.0e-2 };
    double bernstein;

    bernstein = bezier_curve->bernstein_polynomial(3, 0, 0.1);

    ASSERT_NEAR(bernstein, result, error);

};


TEST_F(BezierTesting, bernstein_polynomial_k_1_is_ok){

    double result { 0.243 };
    double error { 1.0e-2 };
    double bernstein;

    bernstein = bezier_curve->bernstein_polynomial(3, 1, 0.1);

    ASSERT_NEAR(bernstein, result, error);

};


TEST_F(BezierTesting, bernstein_polynomial_k_2_is_ok){

    double result { 0.027 };
    double error { 1.0e-2 };
    double bernstein;

    bernstein = bezier_curve->bernstein_polynomial(3, 2, 0.1);

    ASSERT_NEAR(bernstein, result, error);

};


TEST_F(BezierTesting, calculated_coordinates_conversion_are_ok){

    data input {
    {0.0, 28.28, 61.25, 88.69, 100.0},
    {0.0, 36.72, 68.75, 91.41, 100.0},
    {0.0, 0.0, 0.0, 0.0, 0.0}

};

    data result {
                  {0.0, 0.0, 0.0},
                  {28.28, 36.72, 0.0},
                  {61.25, 68.75, 0.0},
                  {88.69, 91.41, 0.0},
                  {100.0, 100.0, 0.0}
                };

    double error { 1.0e-2 };
    std::shared_ptr<data> converted_coords;

    converted_coords = bezier_curve->calculated_coordinates_conversion(input);

    for(int i = 0; i < 5; ++i){
        for(int j = 0; j < 3; ++j){
            ASSERT_NEAR((*converted_coords)[i][j], result[i][j], error);
        };
    };

};


TEST_F(BezierTesting, output_coordinates_are_ok){

    data result {
                  {0.0, 0.0, 0.0},
                  {28.28, 36.72, 0.0},
                  {61.25, 68.75, 0.0},
                  {88.69, 91.41, 0.0},
                  {100.0, 100.0, 0.0}
                };

    double error { 1.0e-1 };
    std::shared_ptr<data> calculated_coords;

    calculated_coords = bezier_curve->calculate_coordinates();

    for(int i = 0; i < 5; ++i){
        for(int j = 0; j < 3; ++j){
            ASSERT_NEAR((*calculated_coords)[i][j], result[i][j], error);
        };
    };

};
