#ifndef MODELER_H_
#define MODELER_H

#include <math.h>
#include <memory>
#include <vector>

#include "bezier/bezier.hpp"
#include "../constrainer/constrainer.hpp"
#include "dubins/dubins.hpp"

#include "../../ext/Eigen/Dense"

namespace modeler{

/*
 * \class Waypoint
 *
 * \brief Holds data regarding to a waypoint and dynamics related aircraft data
 * over that waypoint
 *
 */

        class Waypoint{

                private:
                        double heading;
                        std::vector<double> coordinates;
                        std::shared_ptr<constrainer::AircraftDynamics> aircraft_dynamics;

                public:
                        Waypoint(double heading,
                                 std::vector<double> coordinates):
                                heading(heading),
                                coordinates(coordinates){};

                        ~Waypoint(){};

                        double get_heading() const {return heading;}
                        double get_turn_radius() const {return aircraft_dynamics->turn_radius();}
                        std::vector<double> get_coordinates() const {return coordinates;}

        };

/*
 * \class Route
 *
 * \brief Calculates a trajectory using a predefined tecnique (the default
 * combines 2D Dubins curves with 3D Bezier curves for softening)
 *
 */

        class Route{

                private:
                        int resolution;
                        std::shared_ptr<std::vector<double>> headings;
                        std::shared_ptr<std::vector<double>> turn_radius;
                        std::shared_ptr<std::vector<std::vector<double>>> control_waypoints_coords;/**<
                                                                                     *  Control points to
                                                                                     *  create the complete
                                                                                     *  route
                                                                                     *
                                                                                     */

                        std::shared_ptr<std::vector<std::vector<double>>> route; /**<
                                                                                  *   Complete route points
                                                                                  */


                        /* \fn assembly_sparse_matrix()
                         *
                         * \brief Generates the sparse matrix A of size
                         * (3 x control_waypoints_num) X (3 x control_waypoints_num)
                         * filled with 1's and 0's
                         *
                         */

                        std::shared_ptr<Eigen::MatrixXd> assembly_sparse_matrix();


                        /* \fn assembly_control_waypoints()
                         *
                         * \brief Generates a vector including all the waypoints of each
                         * heading for the Dubins path
                         *
                         */

                        std::shared_ptr<Eigen::VectorXd> assembly_control_waypoints();


                        /* \fn solve_bezier_control_points()
                         *
                         * \brief Calculate the linear system of equations Ax = b, where
                         * A is the sparse matrix, x are the bezier control points and b
                         * are the control waypoints
                         *
                         */

                        std::shared_ptr<std::vector<std::vector<double>>> solve_bezier_control_points();


                        /* \fn calculate_dubins_route()
                         *
                         * \brief Calculates multiple sets of 2D dubins paths by joining each
                         * control waypoint together
                         *
                         */

                        std::shared_ptr<std::vector<std::vector<double>>> calculate_dubins_route();


                        /* \fn calculate_bezier_curve()
                         *
                         * \brief Calculates the bezier curve by using the control points calculated
                         * in solve_bezier_control_points(). Each bezier curve is calculated by using
                         * 2 control points
                         *
                         */

                        std::shared_ptr<std::vector<std::vector<double>>> calculate_bezier_curve();

                public:
                        Route(){};

                        Route(int resolution,
                              std::shared_ptr<std::vector<double>> headings,
                              std::shared_ptr<std::vector<std::vector<double>>> control_waypoints_coords):
                                resolution(resolution),
                                headings(headings),
                                control_waypoints_coords(control_waypoints_coords){};

                        ~Route(){};


                        /* \fn compute_route()
                         *
                         * \brief Interface function to compute all the 3D route points
                         *
                         */

                        void compute_route();
                        std::shared_ptr<std::vector<std::vector<double>>> get_route_coordinates() const {return route;}

        };


/*
 * \class Modeler
 *
 * \brief Interface to calculate and model a trajectory
 *
 */

        class Modeler{

                private:
                        int generator; /**< Generation algorithm for the 3D path */
                        std::shared_ptr<Route> route;

                        /* \fn load_route_modeling_data()
                         *
                         * \brief load required data to feed into the route generation
                         * algorithms
                         *
                         */

                        void load_route_modeling_data();


                        /* \fn store_route_modeling_data()
                         *
                         * \brief stores generated route data
                         *
                         */

                        void store_route_modeling_data();



                public:
                        Modeler(){};

                        Modeler(int generator):
                                generator(generator){};

                        ~Modeler(){};

                        void generate_route();



        };

};

//TODO
//1. Refactor Route class into template, to include multiple route generation tecniques
//2. Waypoint class is well suited for dubins but maybe it will require future refactoring
//   when applying other route generation methods
//3. Refactor the assembly_sparse_matrix() method to consider the class SparseMatrix of
//   Eigen library

#endif // MODELER_H_
