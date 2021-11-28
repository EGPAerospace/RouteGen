#include "dubins.hpp"

/*
 * namespace : dubins
 * class : Segment
 * method : classify_member_category()
 *
 */

void dubins::Segment::classify_member_category(){

    bool is_turn, is_straight;
    std::vector<std::shared_ptr<dubins::Member>>::iterator it;

    for (it = member.begin(); it < member.end(); it++){

        is_turn = ((*it)->get_rotation() != 0);
        is_straight = ((*it)->get_rotation() == 0);

        if (is_turn){

            turn_member = (*it);

        } else if (is_straight){

            straight_member = (*it);

        }

    };

};


/*
 * namespace : dubins
 * class : Segment
 * method : heading_conversor()
 *
 */

double dubins::Segment::heading_conversor(double heading){

    double corrected_heading;
    bool is_greater, is_smaller;

    is_smaller = (heading < M_PI);
    is_greater = (heading >= M_PI);

    if (is_smaller){

        corrected_heading = heading;

    } else if (is_greater){

        corrected_heading = heading - 2.0 * M_PI;

    };

    return corrected_heading;

};


/*
 * namespace : dubins
 * class : Segment
 * method : heading_range()
 *
 */


double dubins::Segment::heading_range(){

    double heading_range;
    double heading_start, heading_end;
    bool clockwise, counterclockwise;
    bool initial_greater_half, initial_smaller_half;
    bool last_greater_half, last_smaller_half;
    bool both_greater, both_smaller, initial_smaller_last_greater, initial_greater_last_smaller;

    heading_start = member[0]->get_heading();
    heading_end   = member[1]->get_heading();

    clockwise            = (turn_member->get_rotation() == 1);
    counterclockwise     = (turn_member->get_rotation() == -1);
    initial_greater_half = (heading_start > M_PI);
    initial_smaller_half = (heading_start <= M_PI);
    last_greater_half    = (heading_end > M_PI);
    last_smaller_half    = (heading_end <= M_PI);

    both_greater = (initial_greater_half && last_greater_half);
    both_smaller = (initial_smaller_half && last_smaller_half);
    initial_smaller_last_greater = (initial_smaller_half && last_greater_half);
    initial_greater_last_smaller = (initial_greater_half && last_smaller_half);

    if (clockwise){

        if (both_greater || initial_greater_last_smaller){
            heading_range = abs(2.0 * M_PI - heading_start) + heading_end;

        } else if (both_smaller || initial_smaller_last_greater){
            heading_range = abs(heading_end - heading_start);

        };

    } else if (counterclockwise){

        if (both_greater || initial_greater_last_smaller){
            heading_range = abs(heading_end - heading_start);

        } else if (both_smaller || initial_smaller_last_greater){
            heading_range = abs(2.0 * M_PI - heading_end) + heading_start;

       };

    };

    return heading_range;

};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_length_turn()
 *
 */

void dubins::Segment::calculate_length_turn(){

    double range_heading;
    double length;

    /* Calculate total angular displacement */
    range_heading = heading_range();

    length = turn_member->get_turn_radius() * range_heading;

    trajectory->set_length(length);

};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_length_straight()
 *
 */

void dubins::Segment::calculate_length_straight(){

    double length;
    double x_s, y_s;
    double x_f, y_f;

    x_s = center_coords[0][0];
    y_s = center_coords[0][1];
    x_f = center_coords[1][0];
    y_f = center_coords[1][1];

    length = sqrt(pow((x_f - x_s), 2) + pow((y_f - y_s), 2));

    trajectory->set_length(length);

};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_length()
 *
 */


double dubins::Segment::calculate_length(){

    bool has_turn, has_no_turn;

    classify_member_category();

    has_turn = (turn_member != NULL);
    has_no_turn = (turn_member == NULL);

    if (has_turn){

        calculate_length_turn();

    } else if (has_no_turn){

        calculate_length_straight();

    };

    return trajectory->get_length();

};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_coordinates_turn()
 *
 */

void dubins::Segment::calculate_coordinates_turn(){

    typedef std::vector<std::vector<double>> data;

    int steps { trajectory->get_coord_precision() };
    double start_heading, range_heading, step_heading;
    double actual_heading;
    double x, y;
    data coords_container(2, std::vector<double>(steps));
    std::shared_ptr<data> coords;

    /* Calculate total angular displacement */
    start_heading = heading_conversor(member[0]->get_heading());
    range_heading = heading_range();

    /* Fill initial value */
    actual_heading = start_heading;
    step_heading = (1.0 / (double)(steps - 1)) * range_heading;
    /* x-coordinates */
    x = center_coords[0][0] + turn_member->get_turn_radius() * sin(actual_heading - turn_member->get_rotation() * M_PI_2);
    coords_container[0][0] = x;

    /* y_coordinates */
    y = center_coords[0][1] + turn_member->get_turn_radius() * cos(actual_heading - turn_member->get_rotation() * M_PI_2);
    coords_container[1][0] = y;

    /* Fill the remaining values */
    for (int i = 1; i < steps; i++){
        actual_heading = actual_heading + turn_member->get_rotation() * step_heading;
        actual_heading = heading_conversor(actual_heading);

        /* x-coordinates */
        x = center_coords[0][0] + turn_member->get_turn_radius() * sin(actual_heading - turn_member->get_rotation() * M_PI_2);
        coords_container[0][i] = x;

        /* y_coordinates */
        y = center_coords[0][1] + turn_member->get_turn_radius() * cos(actual_heading - turn_member->get_rotation() * M_PI_2);
        coords_container[1][i] = y;
    }

    /* Storing last points in trajectory struct */
    trajectory->set_last_point( {x, y} );

    coords = std::make_shared<data>(coords_container);
    trajectory->set_coordinates(coords);
};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_coordinates_straight()
 *
 */

void dubins::Segment::calculate_coordinates_straight(){

    typedef std::vector<std::vector<double>> data;

    int steps { trajectory->get_coord_precision() };
    double step_distance, total_distance;
    double x_old, y_old, x_new, y_new;
    data coords_container(2, std::vector<double>(steps));
    std::shared_ptr<data> coords;

    /* Fill initial value */
    calculate_length_straight();
    total_distance = trajectory->get_length();
    step_distance = (1.0 / (double)(steps - 1)) * total_distance;

    /* x-coordinates */
    x_old = trajectory->get_initial_point()[0];
    x_new = x_old;
    coords_container[0][0] = x_new;

    /* y_coordinates */
    y_old = trajectory->get_initial_point()[1];
    y_new = y_old;
    coords_container[1][0] = y_new;

    /* Fill the remaining values */
    for (int i = 1; i < steps; i++){
        x_old = x_new;
        y_old = y_new;

        /* x-coordinates */
        x_new = x_old + step_distance * sin(straight_member->get_heading());
        coords_container[0][i] = x_new;

        /* y_coordinates */
        y_new = y_old + step_distance * cos(straight_member->get_heading());
        coords_container[1][i] = y_new;
    }

    /* Storing last points in trajectory struct */
    trajectory->set_last_point( {x_new, y_new} );

    coords = std::make_shared<data>(coords_container);
    trajectory->set_coordinates(coords);

};


/*
 * namespace : dubins
 * class : Segment
 * method : calculate_coords()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> dubins::Segment::calculate_coords(){

    std::vector<double> x_coords;
    std::vector<double> y_coords;
    std::shared_ptr<std::vector<std::vector<double>>> coords;
    bool has_turn, has_no_turn;

    classify_member_category();

    has_turn = (turn_member != NULL);
    has_no_turn = (turn_member == NULL);

    if (has_turn){

        calculate_coordinates_turn();

    } else if (has_no_turn){

        calculate_coordinates_straight();

    }

    return trajectory->get_coordinates();

};


/*
 * namespace : dubins
 * class : Dubins
 * method : circumference_center()
 *
 */

std::vector<double> dubins::Dubins::circumference_center(std::shared_ptr<dubins::Member> member,
                                                         std::vector<double> point){

    std::vector<double> coords;
    double x_i, y_i;
    double x_tc, y_tc;
    bool clockwise, counterclock;

    x_i = point[0];
    y_i = point[1];
    clockwise = (member->get_rotation() == 1);
    counterclock = (member->get_rotation() == -1);

    if(clockwise){

        x_tc = x_i + member->get_turn_radius() * cos(member->get_heading());
        y_tc = y_i - member->get_turn_radius() * sin(member->get_heading());

    } else if (counterclock){

        x_tc = x_i - member->get_turn_radius() * cos(member->get_heading());
        y_tc = y_i + member->get_turn_radius() * sin(member->get_heading());

    }

    coords.push_back(x_tc);
    coords.push_back(y_tc);

    return coords;

};


/*
 * namespace : dubins
 * class : Dubins
 * method : transfer_angle()
 *
 */

double dubins::Dubins::transfer_angle(std::vector<double>& start_turn_center_coords,
                                      std::vector<double>& end_turn_center_coords){

    double theta, eta, gamma;
    double line_step;
    double x_s, y_s, x_f, y_f;
    double distance_centers, distance_cross;
    bool is_external, is_internal;
    bool is_negative;

    x_s = start_turn_center_coords[0];
    y_s = start_turn_center_coords[1];
    x_f = end_turn_center_coords[0];
    y_f = end_turn_center_coords[1];

    is_external = (member[0]->get_rotation() == member[1]->get_rotation());
    is_internal = (member[0]->get_rotation() != member[1]->get_rotation());

    line_step = atan((y_f - y_s) / (x_f - x_s));
    is_negative = (line_step < 0.0);

    if (is_external){

        theta = M_PI_2 - line_step;;

    } else if (is_internal){

        distance_centers = sqrt(pow((x_f - x_s), 2) + pow((y_f - y_s), 2));
        distance_cross = sqrt(pow(distance_centers, 2) - 4.0 * pow(member[0]->get_turn_radius(), 2));
        eta = M_PI_2 - line_step;
        gamma = atan(distance_cross / (2.0 * member[0]->get_turn_radius()));
        theta = eta + member[0]->get_rotation() * (M_PI_2 - gamma);

    };

    if (is_negative){

        theta = theta + M_PI;

    };

    return theta;

};


/*
 * namespace : dubins
 * class : Dubins
 * method : length_single_trajectory()
 *
 */

double dubins::Dubins::length_single_trajectory(std::vector<int> rotations){

    double length {0.0};
    double angle_transf;
    std::shared_ptr<dubins::Trajectory> initial_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Trajectory> mid_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Trajectory> last_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Member> initial_member_segment;
    std::shared_ptr<dubins::Member> last_member_segment;
    std::vector<std::shared_ptr<dubins::Member>> member_vector;
    std::shared_ptr<dubins::Segment> segment;
    std::vector<double> center_coords_initial, center_coords_last;
    std::vector<std::vector<double>> center_coords_vec;

    /* Set rotation to original member vector */
    this->member[0]->set_rotation(rotations[0]);
    this->member[1]->set_rotation(rotations[2]);


    /* Calculate turn circumference centers for both turns */
    center_coords_initial = circumference_center(this->member[0], this->trajectory->get_initial_point());
    center_coords_last = circumference_center(this->member[1], this->trajectory->get_last_point());

    /* INITIAL SEGMENT */
    /* Setting trajectory data */
    initial_trajectory_segment->set_coord_precision(3);
    initial_trajectory_segment->set_initial_point(trajectory->get_initial_point());

    /* Setting initial member data */
    initial_member_segment = std::make_shared<dubins::Member>(this->member[0]->get_heading(),
                                                              this->member[0]->get_turn_radius(),
                                                              rotations[0]);

    /* Setting last member data */
    /* Calculate tranfer angle */
    angle_transf = transfer_angle(center_coords_initial, center_coords_last);

    last_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                           0.0,
                                                           rotations[1]);

    member_vector.push_back(initial_member_segment);
    member_vector.push_back(last_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_initial);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                initial_trajectory_segment,
                                                center_coords_vec);

    length += segment->calculate_length();
    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();


    // /* MID SEGMENT */
    mid_trajectory_segment->set_coord_precision(3);
    mid_trajectory_segment->set_initial_point(initial_trajectory_segment->get_initial_point());

    /* Setting first segment trajectory data before starting iteration */
    initial_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                              0.0,
                                                              rotations[1]);

    member_vector.push_back(initial_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_initial);
    center_coords_vec.push_back(center_coords_last);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                mid_trajectory_segment,
                                                center_coords_vec);

    length += segment->calculate_length();
    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();


    /* LAST SEGMENT */
    /* Setting trajectory data */
    last_trajectory_segment->set_coord_precision(3);
    last_trajectory_segment->set_initial_point(mid_trajectory_segment->get_initial_point());

    /* Setting first segment trajectory data before starting iteration */
    initial_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                              0.0,
                                                              rotations[1]);

    last_member_segment = std::make_shared<dubins::Member>(this->member[1]->get_heading(),
                                                           this->member[1]->get_turn_radius(),
                                                           rotations[2]);

    member_vector.push_back(initial_member_segment);
    member_vector.push_back(last_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_last);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                last_trajectory_segment,
                                                center_coords_vec);

    length += segment->calculate_length();
    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();


    return length;

};


/*
 * namespace : dubins
 * class : Dubins
 * method : coordinates_single_trajectory()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> dubins::Dubins::coordinates_single_trajectory(std::vector<int> rotations){

    std::shared_ptr<std::vector<std::vector<double>>> coordinates_trajectory, coords_buffer;
    double angle_transf;
    std::shared_ptr<dubins::Trajectory> initial_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Trajectory> mid_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Trajectory> last_trajectory_segment = std::make_shared<dubins::Trajectory>();
    std::shared_ptr<dubins::Member> initial_member_segment;
    std::shared_ptr<dubins::Member> last_member_segment;
    std::vector<std::shared_ptr<dubins::Member>> member_vector;
    std::shared_ptr<dubins::Segment> segment;
    std::vector<double> center_coords_initial, center_coords_last;
    std::vector<std::vector<double>> center_coords_vec;

    /* Set rotation to original member vector */
    this->member[0]->set_rotation(rotations[0]);
    this->member[1]->set_rotation(rotations[2]);


    /* Calculate turn circumference centers for both turns */
    center_coords_initial = circumference_center(this->member[0], this->trajectory->get_initial_point());
    center_coords_last = circumference_center(this->member[1], this->trajectory->get_last_point());

    /* INITIAL SEGMENT */
    /* Setting trajectory data */
    initial_trajectory_segment->set_coord_precision(this->trajectory->get_coord_precision());
    initial_trajectory_segment->set_initial_point(trajectory->get_initial_point());

    /* Setting initial member data */
    initial_member_segment = std::make_shared<dubins::Member>(this->member[0]->get_heading(),
                                                              this->member[0]->get_turn_radius(),
                                                              rotations[0]);

    /* Setting last member data */
    /* Calculate tranfer angle */
    angle_transf = transfer_angle(center_coords_initial, center_coords_last);

    last_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                           0.0,
                                                           rotations[1]);

    member_vector.push_back(initial_member_segment);
    member_vector.push_back(last_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_initial);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                initial_trajectory_segment,
                                                center_coords_vec);

    coordinates_trajectory = segment->calculate_coords();
    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();


    // /* MID SEGMENT */
    mid_trajectory_segment->set_coord_precision(this->trajectory->get_coord_precision());
    mid_trajectory_segment->set_initial_point(initial_trajectory_segment->get_last_point());

    /* Setting first segment trajectory data before starting iteration */
    initial_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                              0.0,
                                                              rotations[1]);

    member_vector.push_back(initial_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_initial);
    center_coords_vec.push_back(center_coords_last);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                mid_trajectory_segment,
                                                center_coords_vec);

    coords_buffer = segment->calculate_coords();

    /* Appending this segment coordinates to the general coordinates vector */
    (*coordinates_trajectory)[0].insert((*coordinates_trajectory)[0].end(),
                                        (*coords_buffer)[0].begin(),
                                        (*coords_buffer)[0].end());

    (*coordinates_trajectory)[1].insert((*coordinates_trajectory)[1].end(),
                                        (*coords_buffer)[1].begin(),
                                        (*coords_buffer)[1].end());

    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();
    coords_buffer.reset();


    /* LAST SEGMENT */
    /* Setting trajectory data */
    last_trajectory_segment->set_coord_precision(this->trajectory->get_coord_precision());
    last_trajectory_segment->set_initial_point(mid_trajectory_segment->get_initial_point());

    /* Setting first segment trajectory data before starting iteration */
    initial_member_segment = std::make_shared<dubins::Member>(angle_transf,
                                                              0.0,
                                                              rotations[1]);

    last_member_segment = std::make_shared<dubins::Member>(this->member[1]->get_heading(),
                                                           this->member[1]->get_turn_radius(),
                                                           rotations[2]);

    member_vector.push_back(initial_member_segment);
    member_vector.push_back(last_member_segment);

    /* Initialize and run segment calculations */
    center_coords_vec.push_back(center_coords_last);
    segment = std::make_shared<dubins::Segment>(member_vector,
                                                last_trajectory_segment,
                                                center_coords_vec);

    coords_buffer = segment->calculate_coords();

    /* Appending this segment coordinates to the general coordinates vector */
    (*coordinates_trajectory)[0].insert((*coordinates_trajectory)[0].end(),
                                        (*coords_buffer)[0].begin(),
                                        (*coords_buffer)[0].end());

    (*coordinates_trajectory)[1].insert((*coordinates_trajectory)[1].end(),
                                        (*coords_buffer)[1].begin(),
                                        (*coords_buffer)[1].end());

    segment.reset();
    center_coords_vec.clear();
    member_vector.clear();
    initial_member_segment.reset();
    last_member_segment.reset();
    coords_buffer.reset();

    return coordinates_trajectory;

};


/*
 * namespace : dubins
 * class : Dubins
 * method : longest_path()
 *
 */

void dubins::Dubins::longest_path(){

    /* Define a set of rotations that must be analyzed iteratively */
    /* Each row corresponds to a specific trajectory type */
    std::vector<std::vector<int>> rotations {
                                              { 1,  0,  1}, /* R S R */
                                              {-1,  0, -1}, /* L S L */
                                              { 1,  0, -1}, /* R S L */
                                              {-1,  0,  1}, /* L S R */
                                            };

    std::vector<int> rotations_best;
    double length_old {0.0};
    double length_new;
    bool is_longer;
    std::vector<std::vector<int>>::iterator iter_traj;

    for (iter_traj = rotations.begin(); iter_traj < rotations.end(); iter_traj++){

        /* Calculating total length for the given trajectory configuration */
        length_new = length_single_trajectory((*iter_traj));
        is_longer = (length_new > length_old);

        if (is_longer){
            rotations_best.clear();
            rotations_best = (*iter_traj);
            length_old = length_new;
        };

    };

    /* Storing best length */
    this->length = length_new;

    /* Calculating coordinates for best candidate */
    this->coordinates = coordinates_single_trajectory(rotations_best);

};


/*
 * namespace : dubins
 * class : Dubins
 * method : shortest_path()
 *
 */


void dubins::Dubins::shortest_path(){

    /* Define a set of rotations that must be analyzed iteratively */
    /* Each row corresponds to a specific trajectory type */
    std::vector<std::vector<int>> rotations {
                                              { 1,  0,  1}, /* R S R */
                                              {-1,  0, -1}, /* L S L */
                                              { 1,  0, -1}, /* R S L */
                                              {-1,  0,  1}, /* L S R */
                                            };

    std::vector<int> rotations_best;
    double length_old;
    double length_new;
    bool is_shorter;
    std::vector<std::vector<int>>::iterator iter_traj;

    length_old = length_single_trajectory(rotations[0]);

    for (iter_traj = rotations.begin(); iter_traj < rotations.end(); iter_traj++){

        /* Calculating total length for the given trajectory configuration */
        length_new = length_single_trajectory((*iter_traj));
        is_shorter = (length_new < length_old);

        if (is_shorter){
            rotations_best.clear();
            rotations_best = (*iter_traj);
            length_old = length_new;
        };

    };

    /* Storing best length */
    this->length = length_new;

    /* Calculating coordinates for best candidate */
    this->coordinates = coordinates_single_trajectory(rotations_best);

};


/*
 * namespace : dubins
 * class : Dubins
 * method : coordinate_conversor()
 *
 */

void dubins::Dubins::coordinate_conversor(){

    typedef std::vector<std::vector<double>> data;
    int num_waypoints {this->trajectory->get_coord_precision() * 3};
    std::shared_ptr<data> converted_coordinates;

    converted_coordinates = std::make_shared<data>(num_waypoints, std::vector<double>(2));

    for (int i = 0; i < num_waypoints; i++){

        (*converted_coordinates)[i][0] = (*this->coordinates)[0][i];
        (*converted_coordinates)[i][1] = (*this->coordinates)[1][i];

    };

    /* Deleting old coordinates pointer and updating with new one */
    (this->coordinates).reset();
    this->coordinates = converted_coordinates;

};


/*
 * namespace : dubins
 * class : Dubins
 * method : compute_trajectory()
 *
 */

void dubins::Dubins::compute_trajectory(){

    bool is_short, is_long;

    is_short = (this->path_type == 1);
    is_long = (this->path_type == -1);

    if (is_short){
        shortest_path();
    } else if (is_long){
        longest_path();
    };

    coordinate_conversor();

};
