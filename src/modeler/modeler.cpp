#include "modeler.hpp"

/*
 * namespace : modeler
 * class : Route
 * method : assembly_sparse_matrix()
 *
 */

std::shared_ptr<Eigen::MatrixXd> modeler::Route::assembly_sparse_matrix(){

    int col_offset;
    int mat_size {(int)(*this->control_waypoints_coords).size() * 3};
    std::shared_ptr<Eigen::MatrixXd> sparse_mat;
    sparse_mat = std::make_shared<Eigen::MatrixXd>(mat_size, mat_size);

    /* Filling matrix with 0 */
    *sparse_mat = Eigen::MatrixXd::Zero(mat_size, mat_size);

    /* Filling with 1 */
    for (int i = 0; i < mat_size; i++){
        col_offset = 3 * i;

        (*sparse_mat)(i, col_offset)     = 1.0;
        (*sparse_mat)(i, col_offset + 1) = 1.0;
        (*sparse_mat)(i, col_offset + 2) = 1.0;
        (*sparse_mat)(i, col_offset + 3) = 1.0;
        (*sparse_mat)(i, col_offset + 4) = 1.0;
        (*sparse_mat)(i, col_offset + 5) = 1.0;
    };

    /* Setting last element which have special location */
    (*sparse_mat)(mat_size - 1, 0) = 1.0;
    (*sparse_mat)(mat_size - 1, 1) = 1.0;
    (*sparse_mat)(mat_size - 1, 2) = 1.0;
    (*sparse_mat)(mat_size - 1, mat_size - 3) = 1.0;
    (*sparse_mat)(mat_size - 1, mat_size - 2) = 1.0;
    (*sparse_mat)(mat_size - 1, mat_size - 1) = 1.0;

    return sparse_mat;

};


/*
 * namespace : modeler
 * class : Route
 * method : assembly_control_waypoints()
 *
 */

std::shared_ptr<Eigen::VectorXd> modeler::Route::assembly_control_waypoints(){

    int i {0};
    int vec_size {(int)(*this->control_waypoints_coords).size() * 3};
    std::shared_ptr<Eigen::VectorXd> control_waypoints_vec;
    std::vector<std::vector<double>>::iterator wp_it;

    control_waypoints_vec = std::make_shared<Eigen::VectorXd>(vec_size);

    for (wp_it = (*this->control_waypoints_coords).begin() + 1; wp_it < (*this->control_waypoints_coords).end(); wp_it++){

        (*control_waypoints_vec)(i)     = *(wp_it->begin()) * 2.0;
        (*control_waypoints_vec)(1 + i) = *(wp_it->begin() + 1) * 2.0;
        (*control_waypoints_vec)(2 + i) = *(wp_it->begin() + 2) * 2.0;

        i += 3;
    };

    /* Adding the first control point at th end of vector */
    /* This control point can be considered as a virtual point */
    (*control_waypoints_vec)(i)     = (*this->control_waypoints_coords)[0][0] * 2.0;
    (*control_waypoints_vec)(1 + i) = (*this->control_waypoints_coords)[0][1] * 2.0;
    (*control_waypoints_vec)(2 + i) = (*this->control_waypoints_coords)[0][2] * 2.0;

    return control_waypoints_vec;

};


/*
 * namespace : modeler
 * class : Route
 * method : solve_bezier_control_points()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> modeler::Route::solve_bezier_control_points(){

    std::shared_ptr<std::vector<double>> bezier_converted_type;
    std::shared_ptr<std::vector<std::vector<double>>> bezier_return;
    std::shared_ptr<Eigen::VectorXd> bezier_control_points;
    std::shared_ptr<Eigen::VectorXd> control_waypoints_vec;
    std::shared_ptr<Eigen::MatrixXd> sparse_mat;

    /* Preparing data to solve system */
    sparse_mat = assembly_sparse_matrix();
    control_waypoints_vec = assembly_control_waypoints();

    /* Solve system of equations by applying the standard Cholesky decomposition (LLT) */
    (*bezier_control_points) = (*sparse_mat).llt().solve((*control_waypoints_vec));

    /* Converting from Eigen::VectorXd type to a std::vector<double> type */
    bezier_converted_type = std::make_shared<std::vector<double>>((*bezier_control_points).data(), (*bezier_control_points).data() + (*bezier_control_points).size());

    /* Store bezier control points into a vector of (x, y, z) format */
    bezier_return = std::make_shared<std::vector<std::vector<double>>>((*bezier_converted_type).size() / 3, std::vector<double>(3));
    for (unsigned i = 0; (*bezier_return).size(); i++){
        (*bezier_return)[i][0] = (*bezier_converted_type)[i];
        (*bezier_return)[i][1] = (*bezier_converted_type)[i + 1];
        (*bezier_return)[i][2] = (*bezier_converted_type)[i + 2];
    };

    return bezier_return;

};


/*
 * namespace : modeler
 * class : Route
 * method : calculate_dubins_route()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> modeler::Route::calculate_dubins_route(){

    int num_paths;
    std::shared_ptr<std::vector<std::vector<double>>> complete_route_points, short_route_points;
    std::shared_ptr<dubins::Dubins> dubins_single;
    std::shared_ptr<dubins::Member> member_initial, member_last;
    std::vector<std::shared_ptr<dubins::Member>> member;
    std::shared_ptr<dubins::Trajectory> trajectory;
    std::vector<std::vector<double>>::iterator coord_iterator;
    std::vector<double>::iterator waypoint;

    /* Set data and calculating single dubins path */
    num_paths = (*control_waypoints_coords).size() - 1;

    for (int i = 0; i < num_paths; i++){

        member_initial = std::make_shared<dubins::Member>((*this->headings)[i], (*this->turn_radius)[i]);
        member_last    = std::make_shared<dubins::Member>((*this->headings)[i + 1], (*this->turn_radius)[i + 1]);
        member.push_back(member_initial);
        member.push_back(member_last);
        trajectory = std::make_shared<dubins::Trajectory>(this->resolution, (*this->control_waypoints_coords)[i], (*this->control_waypoints_coords)[i + 1]);
        dubins_single = std::make_shared<dubins::Dubins>(1, member, trajectory);

        /* Calculating single dubins path route points */
        dubins_single->compute_trajectory();
        short_route_points = dubins_single->get_coordinates();

        /* Assembling single dubins path into complete trajectory */
        for (coord_iterator = (*short_route_points).begin(); coord_iterator < (*short_route_points).end(); coord_iterator++){

            (*complete_route_points).push_back(*coord_iterator);
        };

        member_initial.reset();
        member_last.reset();
        member.clear();
        trajectory.reset();
        dubins_single.reset();
        short_route_points.reset();
    };

    return complete_route_points;

};


/*
 * namespace : modeler
 * class : Route
 * method : calculate_bezier_curve()
 *
 */

std::shared_ptr<std::vector<std::vector<double>>> modeler::Route::calculate_bezier_curve(){

    int num_bezier_paths;
    std::shared_ptr<std::vector<std::vector<double>>> solved_bezier_control_points;
    std::shared_ptr<std::vector<std::vector<double>>> current_bezier_control_points;
    std::shared_ptr<std::vector<std::vector<double>>> current_bezier_path;
    std::shared_ptr<std::vector<std::vector<double>>> complete_bezier_path;
    std::shared_ptr<bezier::Bezier> bezier;

    /* Calculating Bezier control points  */
    solved_bezier_control_points = solve_bezier_control_points();

    /* Number of bezier curves that will be calculated and joined together */
    num_bezier_paths = (*solved_bezier_control_points).size() / 3;

    /* Calculate each bezier curve */
    complete_bezier_path = std::make_shared<std::vector<std::vector<double>>>();

    for (int i = 0; i < num_bezier_paths; i++){
        current_bezier_control_points = std::make_shared<std::vector<std::vector<double>>>(3, std::vector<double>(3));

        /* Get 3 control points */
        (*current_bezier_control_points)[0] = (*solved_bezier_control_points)[i];
        (*current_bezier_control_points)[1] = (*solved_bezier_control_points)[i + 1];
        (*current_bezier_control_points)[2] = (*solved_bezier_control_points)[i + 2];

        /* Calculate bezier path for each set of control waypoints */
        bezier = std::make_shared<bezier::Bezier>(this->resolution, current_bezier_control_points);
        current_bezier_path = std::make_shared<std::vector<std::vector<double>>>();
        current_bezier_path =  bezier->calculate_coordinates();

        /* Joining the specific bezier_path */
        complete_bezier_path->insert(complete_bezier_path->end(), current_bezier_path->begin(), current_bezier_path->end());

        current_bezier_control_points.reset();
        current_bezier_path.reset();

    };

    return complete_bezier_path;

};


/*
 * namespace : modeler
 * class : Route
 * method : compute_route()
 *
 */

void modeler::Route::compute_route(){

    std::shared_ptr<std::vector<std::vector<double>>> dubins_coords;
    std::shared_ptr<std::vector<std::vector<double>>> bezier_coords;
    std::shared_ptr<std::vector<std::vector<double>>> final_coords;

    /* Calculate 2D Dubins path which hold complete route in X-Y coordinates */
    dubins_coords = calculate_dubins_route();

    /* Calculate bezier curves for softening the dubins path */
    bezier_coords = calculate_bezier_curve();

    /* Combine X-Y coordinates of Dubins path with Z coordinates of Bezier path */
    final_coords = std::make_shared<std::vector<std::vector<double>>>((*dubins_coords).size(), std::vector<double>(3));
    for (unsigned i = 0; i < (*final_coords).size(); i++){
        (*final_coords)[i][0] = (*dubins_coords)[i][0];
        (*final_coords)[i][1] = (*dubins_coords)[i][1];
        (*final_coords)[i][2] = (*bezier_coords)[i][2];
    }

    this->route = final_coords;

};


/*
 * namespace : modeler
 * class : Modeler
 * method : load_route_modeling_data()
 *
 */

void modeler::Modeler::load_route_modeling_data(){

    std::shared_ptr<modeler::Route> route_data;

    this->route = route_data;

};


/*
 * namespace : modeler
 * class : Modeler
 * method : store_route_modeling_data()
 *
 */

void modeler::Modeler::store_route_modeling_data(){



};


/*
 * namespace : modeler
 * class : Modeler
 * method : generate_route()
 *
 */

void modeler::Modeler::generate_route(){

    load_route_modeling_data();

    route->compute_route();

    store_route_modeling_data();

};
