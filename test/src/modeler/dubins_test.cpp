#include "../../ext/googletest/googletest/include/gtest/gtest.h"

#define private public
#include "../../../src/modeler/dubins/dubins.hpp"


/*****************************************************************************
* Tested namespace  : dubins
* Tested class      : Segment
* Testing name      : ClockWiseToClockWiseTesting
* Testing objective : Test Segment class when it encounters a clockwise
*                     turn
******************************************************************************/

class ClockWiseToClockWiseTesting : public testing::Test{
    protected:

        std::shared_ptr<dubins::Member> member_initial_turn;
        std::shared_ptr<dubins::Member> member_straight;
        std::shared_ptr<dubins::Member> member_last_turn;
        std::shared_ptr<dubins::Trajectory> trajectory_initial_turn = std::make_shared<dubins::Trajectory>();
        std::shared_ptr<dubins::Trajectory> trajectory_last_turn = std::make_shared<dubins::Trajectory>();
        std::shared_ptr<dubins::Segment> segment_initial;
        std::shared_ptr<dubins::Segment> segment_last;
        int rotation {1};
        int rotation_straight {0};
        std::vector<double> center_coords_initial, center_coords_last;
        std::vector<std::vector<double>> center_coords_vec_1, center_coords_vec_2;
        std::vector<std::shared_ptr<dubins::Member>> members_initial, members_last;

        virtual void SetUp(){
            double heading_initial_turn = (315.0 * M_PI) / 180.0;
            double turn_radius_initial_turn = 100.0;
            double heading_last_turn = (135.0 * M_PI) / 180.0;
            double turn_radius_last_turn = 100.0;
            double heading_straight = (33.3426 * M_PI) / 180.0;


            member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                                   turn_radius_initial_turn,
                                                                   rotation);

            member_straight = std::make_shared<dubins::Member>(heading_straight,
                                                               0.0,
                                                               rotation_straight);

            member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                                   turn_radius_last_turn,
                                                                   rotation);



            trajectory_initial_turn->set_coord_precision(5);
            trajectory_initial_turn->set_initial_point({20.0, 20.0});
            trajectory_last_turn->set_coord_precision(5);
            trajectory_last_turn->set_initial_point({545.7484, 964.2542});
            trajectory_last_turn->set_last_point({700.0, 980.0});


            center_coords_initial = {90.7106, 90.7106};
            center_coords_last = {629.2893, 909.2893};

            center_coords_vec_1.push_back(center_coords_initial);
            center_coords_vec_2.push_back(center_coords_last);

            //trajectory->exit_point.insert(trajectory->exit_point.begin(), {90.0, 130.0})
            members_initial.push_back(member_initial_turn);
            members_initial.push_back(member_straight);
            segment_initial = std::make_shared<dubins::Segment>(members_initial,
                                                                trajectory_initial_turn,
                                                                center_coords_vec_1);

            members_last.push_back(member_straight);
            members_last.push_back(member_last_turn);
            segment_last = std::make_shared<dubins::Segment>(members_last,
                                                             trajectory_last_turn,
                                                             center_coords_vec_2);

        };

        virtual void TearDown(){


        };

};


TEST_F(ClockWiseToClockWiseTesting, initial_turn_length_is_ok){

    double result { 136.7336 };
    double calculated_length;
    double error { 1.0e-4 };

    calculated_length = segment_initial->calculate_length();

    ASSERT_NEAR(calculated_length, result, error);

};


TEST_F(ClockWiseToClockWiseTesting, last_turn_length_is_ok){

    double result { 177.4256 };
    double calculated_length;
    double error { 1.0e-3 };

    calculated_length = segment_last->calculate_length();

    ASSERT_NEAR(calculated_length, result, error);



};


TEST_F(ClockWiseToClockWiseTesting, initial_turn_coords_are_ok){

    std::vector<std::vector<double>> result { {20.0,  0.3878, -8.7723,  -6.4207,   7.1707},
                                              {20.0, 47.7943, 80.5549, 114.4907, 145.6750}};

    std::shared_ptr<std::vector<std::vector<double>>> coordinates;
    double error { 1.0e-2 };

    coordinates = segment_initial->calculate_coords();

    for(int i = 0; i < 2; ++i){
        for(int j = 0; j < 5; ++j){
            ASSERT_NEAR((*coordinates)[i][j], result[i][j], error);
        };
    };


};


TEST_F(ClockWiseToClockWiseTesting, last_turn_coords_are_ok){

    std::vector<std::vector<double>> result { {545.7484, 577.4222,  619.1336,  662.8106, 700.0},
                                              {964.2542, 994.2542, 1008.7722, 1003.5035, 980 } };

    std::shared_ptr<std::vector<std::vector<double>>> coordinates;
    double error { 1.0 };

    coordinates = segment_last->calculate_coords();

    for(int i = 0; i < 2; ++i){
        for(int j = 0; j < 5; ++j){
            ASSERT_NEAR((*coordinates)[i][j], result[i][j], error);
        };
    };


};


/*****************************************************************************
* Tested namespace  : dubins
* Tested class      : Segment
* Testing name      : CounterClockToCounterClockTesting
* Testing objective : Test Segment class when it encounters a counterclockwise
*                     turn
******************************************************************************/

class CounterClockToCounterClockTesting : public testing::Test{
    protected:
        std::shared_ptr<dubins::Member> member_initial_turn;
        std::shared_ptr<dubins::Member> member_straight;
        std::shared_ptr<dubins::Member> member_last_turn;
        std::shared_ptr<dubins::Trajectory> trajectory_initial_turn = std::make_shared<dubins::Trajectory>();
        std::shared_ptr<dubins::Trajectory> trajectory_last_turn = std::make_shared<dubins::Trajectory>();
        std::shared_ptr<dubins::Segment> segment_initial;
        std::shared_ptr<dubins::Segment> segment_last;
        int rotation {-1};
        int rotation_straight {0};
        std::vector<double> center_coords_initial, center_coords_last;
        std::vector<std::vector<double>> center_coords_vec_1, center_coords_vec_2;
        std::vector<std::shared_ptr<dubins::Member>> members_initial, members_last;

        virtual void SetUp(){
            double heading_initial_turn = (315.0 * M_PI) / 180.0;
            double turn_radius_initial_turn = 100.0;
            double heading_last_turn = (135.0 * M_PI) / 180.0;
            double turn_radius_last_turn = 100.0;
            double heading_straight = (213.3426 * M_PI) / 180.0;


            member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                                   turn_radius_initial_turn,
                                                                   rotation);

            member_straight = std::make_shared<dubins::Member>(heading_straight,
                                                               0.0,
                                                               rotation_straight);

            member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                                   turn_radius_last_turn,
                                                                   rotation);



            trajectory_initial_turn->set_coord_precision(5);
            trajectory_initial_turn->set_initial_point({700.0, 980.0});
            trajectory_last_turn->set_coord_precision(5);
            trajectory_last_turn->set_initial_point({7.1707, 145.6750});
            trajectory_last_turn->set_last_point({20.0, 20.0});


            center_coords_initial = {629.2893, 909.2893};
            center_coords_last = {90.7106, 90.7106};

            center_coords_vec_1.push_back(center_coords_initial);
            center_coords_vec_2.push_back(center_coords_last);

            //trajectory->exit_point.insert(trajectory->exit_point.begin(), {90.0, 130.0})
            members_initial.push_back(member_initial_turn);
            members_initial.push_back(member_straight);
            segment_initial = std::make_shared<dubins::Segment>(members_initial,
                                                                trajectory_initial_turn,
                                                                center_coords_vec_1);

            members_last.push_back(member_straight);
            members_last.push_back(member_last_turn);
            segment_last = std::make_shared<dubins::Segment>(members_last,
                                                             trajectory_last_turn,
                                                             center_coords_vec_2);

        };

        virtual void TearDown(){


        };

};


TEST_F(CounterClockToCounterClockTesting, initial_turn_length_is_ok){

    double result { 177.4256 };
    double calculated_length;
    double error { 1.0e-4 };

    calculated_length = segment_initial->calculate_length();

    ASSERT_NEAR(calculated_length, result, error);

};


TEST_F(CounterClockToCounterClockTesting, last_turn_length_is_ok){

    double result { 136.7336 };
    double calculated_length;
    double error { 1.0e-3 };

    calculated_length = segment_last->calculate_length();

    ASSERT_NEAR(calculated_length, result, error);



};


TEST_F(CounterClockToCounterClockTesting, initial_turn_coords_are_ok){

    std::vector<std::vector<double>> result { {700.0,  662.8108,  619.1336, 577.4220, 545.7484},
                                              {980.0, 1003.5034, 1008.7720, 994.7867, 964.2542} };

    std::shared_ptr<std::vector<std::vector<double>>> coordinates;
    std::vector<std::vector<double>>::iterator rows;
    std::vector<double>::iterator cols;
    double error { 1.0e-2 };

    coordinates = segment_initial->calculate_coords();

    for(int i = 0; i < 2; ++i){
        for(int j = 0; j < 5; ++j){
            ASSERT_NEAR((*coordinates)[i][j], result[i][j], error);
        };
    };


};


TEST_F(CounterClockToCounterClockTesting, last_turn_coords_are_ok){

    std::vector<std::vector<double>> result { {  7.1706,  -6.4134, -8.7754,  0.3746, 20.0},
                                              {145.6750, 114.5205, 80.5855, 47.8221, 20.0} };

    std::shared_ptr<std::vector<std::vector<double>>> coordinates;
    std::vector<std::vector<double>>::iterator rows;
    std::vector<double>::iterator cols;
    double error { 1.0e-1 };

    coordinates = segment_last->calculate_coords();

    for(int i = 0; i < 2; ++i){
        for(int j = 0; j < 5; ++j){
            ASSERT_NEAR((*coordinates)[i][j], result[i][j], error);
        };
    };


};


/*****************************************************************************
* Tested namespace  : dubins
* Tested class      : Dubins
* Testing name      : DubinsConfigurationDataTesting
* Testing objective : Test Dubins class including the methods that will
*                     configure the Segment classes
******************************************************************************/

class DubinsConfigurationDataTesting : public testing::Test{
    protected:
        std::shared_ptr<dubins::Member> member_initial_turn, member_last_turn;
        std::shared_ptr<dubins::Trajectory> trajectory = std::make_shared<dubins::Trajectory>();
        std::shared_ptr<dubins::Dubins> dubins_path;
        int rotation_initial, rotation_last;
        std::vector<std::shared_ptr<dubins::Member>> members;

        virtual void SetUp(){

        };

        virtual void TearDown(){

        };

};


TEST_F(DubinsConfigurationDataTesting, clockwise_circumference_center_coords_are_ok){

    std::vector<double> result {90.7106, 90.7106};
    std::vector<double> coordinates;
    double error { 1.0e-2 };

    double heading = (315.0 * M_PI) / 180.0;
    double turn_radius = 100.0;
    int rotation {1};


    member_initial_turn = std::make_shared<dubins::Member>(heading,
                                                           turn_radius,
                                                           rotation);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({20.0, 20.0});

    coordinates = dubins_path->circumference_center(member_initial_turn,
                                                    trajectory->get_initial_point());

    for(int i = 0; i < 2; ++i){
        ASSERT_NEAR(coordinates[i], result[i], error);
    };


};


TEST_F(DubinsConfigurationDataTesting, counterclockwise_circumference_center_coords_are_ok){

    std::vector<double> result {629.2893, 909.2893};
    std::vector<double> coordinates;
    double error { 1.0e-2 };

    double heading = (315.0 * M_PI) / 180.0;
    double turn_radius = 100.0;
    int rotation {-1};


    member_initial_turn = std::make_shared<dubins::Member>(heading,
                                                           turn_radius,
                                                           rotation);

    trajectory->set_coord_precision(5);
    trajectory->set_last_point({700.0, 980.0});

    coordinates = dubins_path->circumference_center(member_initial_turn,
                                                    trajectory->get_last_point());

    for(int i = 0; i < 2; ++i){
        ASSERT_NEAR(coordinates[i], result[i], error);
    };


};


TEST_F(DubinsConfigurationDataTesting, transfer_angle_clockwise_to_clockwise_is_ok){

    double result = (33.3426 * M_PI) / 180.0;
    double angle;
    double error { 1.0e-2 };

    double heading_initial_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;
    std::vector<double> start_turn_coords {90.7106, 90.7106};
    std::vector<double> end_turn_coords {629.2893, 909.2893};

    rotation_initial = 1;
    rotation_last = 1;

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn,
                                                           rotation_initial);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn,
                                                        rotation_last);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({20.0, 20.0});
    trajectory->set_last_point({700.0, 980.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(0, members, trajectory);

    angle = dubins_path->transfer_angle(start_turn_coords, end_turn_coords);

    ASSERT_NEAR(angle, result, error);

};


TEST_F(DubinsConfigurationDataTesting, transfer_angle_clockwise_to_counterclockwise_is_ok){

    double result = (45.12 * M_PI) / 180.0;
    double angle;
    double error { 1.0e-2 };

    double heading_initial_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;
    std::vector<double> start_turn_coords {90.7106, 90.7106};
    std::vector<double> end_turn_coords {629.2893, 909.2893};

    rotation_initial = 1;
    rotation_last = -1;

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn,
                                                           rotation_initial);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn,
                                                        rotation_last);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({20.0, 20.0});
    trajectory->set_last_point({700.0, 980.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(0, members, trajectory);

    angle = dubins_path->transfer_angle(start_turn_coords, end_turn_coords);

    ASSERT_NEAR(angle, result, error);

};


TEST_F(DubinsConfigurationDataTesting, transfer_angle_counterclockwise_to_clockwise_is_ok){

    double result = (21.5652 * M_PI) / 180.0;
    double angle;
    double error { 1.0e-2 };

    double heading_initial_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;
    std::vector<double> start_turn_coords {90.7106, 90.7106};
    std::vector<double> end_turn_coords {629.2893, 909.2893};

    rotation_initial = -1;
    rotation_last = 1;

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn,
                                                           rotation_initial);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn,
                                                        rotation_last);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({20.0, 20.0});
    trajectory->set_last_point({700.0, 980.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(0, members, trajectory);

    angle = dubins_path->transfer_angle(start_turn_coords, end_turn_coords);

    ASSERT_NEAR(angle, result, error);

};


TEST_F(DubinsConfigurationDataTesting, transfer_angle_counterclockwise_to_counterclockwise_is_ok){

    double result = (33.3426 * M_PI) / 180.0;
    double angle;
    double error { 1.0e-2 };

    double heading_initial_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;
    std::vector<double> start_turn_coords {629.2893, 909.2893};
    std::vector<double> end_turn_coords {90.7106, 90.7106};

    rotation_initial = -1;
    rotation_last = -1;

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn,
                                                           rotation_initial);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn,
                                                        rotation_last);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({700.0, 980.0});
    trajectory->set_last_point({20.0, 20.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(0, members, trajectory);

    angle = dubins_path->transfer_angle(start_turn_coords, end_turn_coords);

    ASSERT_NEAR(angle, result, error);

};


TEST_F(DubinsConfigurationDataTesting, length_trajectory_clockwise_to_clockwise_is_ok){

    double result = 1294.0255;
    double length;
    double error { 1.0e-2 };

    double heading_initial_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;

    std::vector<int> rotations { {1, 0, 1} };

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn);

    trajectory->set_coord_precision(100);
    trajectory->set_initial_point({20.0, 20.0});
    trajectory->set_last_point({700.0, 980.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(-1, members, trajectory);

    length = dubins_path->length_single_trajectory(rotations);

    ASSERT_NEAR(length, result, error);

};


TEST_F(DubinsConfigurationDataTesting, coordinates_trajectory_clockwise_to_clockwise_is_ok){

    std::vector<std::vector<double>> result { {20.0,  0.3878, -8.7723,  -6.4207,   7.1707,
                                               7.1707, 141.8150, 276.4593, 411.1037, 545.7484,
                                               545.7484, 577.4222,  619.1336,  662.8106, 700.0},

                                              {20.0, 47.7943, 80.5549, 114.4907, 145.6750,
                                               145.6750, 350.3197, 554.9645, 759.6092, 964.2542,
                                               964.2542, 994.2542, 1008.7722, 1003.5035, 980}

                                            };

    std::shared_ptr<std::vector<std::vector<double>>> coordinates;

    double error { 1.0 };

    double heading_initial_turn = (315.0 * M_PI) / 180.0;
    double turn_radius_initial_turn = 100.0;
    double heading_last_turn = (135.0 * M_PI) / 180.0;
    double turn_radius_last_turn = 100.0;

    std::vector<int> rotations { {1, 0, 1} };

    member_initial_turn = std::make_shared<dubins::Member>(heading_initial_turn,
                                                           turn_radius_initial_turn);

    member_last_turn = std::make_shared<dubins::Member>(heading_last_turn,
                                                        turn_radius_last_turn);

    trajectory->set_coord_precision(5);
    trajectory->set_initial_point({20.0, 20.0});
    trajectory->set_last_point({700.0, 980.0});

    members.push_back(member_initial_turn);
    members.push_back(member_last_turn);

    dubins_path = std::make_shared<dubins::Dubins>(-1, members, trajectory);

    coordinates = dubins_path->coordinates_single_trajectory(rotations);

    for(int i = 0; i < 2; ++i){
        for(int j = 0; j < 15; ++j){
            ASSERT_NEAR((*coordinates)[i][j], result[i][j], error);
        };
    };


};


TEST_F(DubinsConfigurationDataTesting, converted_coordinates_are_ok){

    typedef std::vector<std::vector<double>> data;
    data coords_vec, result;
    std::shared_ptr<data> provisional_coordinates;
    double error {1.0e-2};

    trajectory->set_coord_precision(5);

    dubins_path = std::make_shared<dubins::Dubins>(0, members, trajectory);

    coords_vec = {
                   {1.0, 2.0, 3.0, 4.0, 5.0},
                   {6.0, 7.0, 8.0, 9.0, 10.0}
};

    result = {
               {1.0, 6.0},
               {2.0, 7.0},
               {3.0, 8.0},
               {4.0, 9.0},
               {5.0, 10.0}
};

    provisional_coordinates = std::make_shared<data>(coords_vec);
    dubins_path->coordinates = provisional_coordinates;

    dubins_path->coordinate_conversor();

    for(int i = 0; i < 5; ++i){
        for(int j = 0; j < 2; ++j){
            ASSERT_NEAR((*dubins_path->coordinates)[i][j], result[i][j], error);
        };
    };



};
