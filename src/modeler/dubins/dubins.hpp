#ifndef DUBINS_H_
#define DUBINS_H_

#include <cstdlib>
#include <math.h>
#include <memory>
#include <vector>


namespace dubins{

/*
 * \class Member
 *
 * \brief Hold the data regarding to a member ready to perform
 * an specific trajectory
 *
 */

        class Member{

                private:
                        double heading;
                        double turn_radius;
                        int rotation; /**<
                                                        *   Clockwise (1),
                                                        *   Counterclockwise (-1)
                                                        *   Straight (0)
                                                        *
                                                        */

                public:
                        Member(double heading,
                               double turn_radius):
                                heading(heading),
                                turn_radius(turn_radius){};

                        Member(double heading,
                               double turn_radius,
                               int rotation):
                                heading(heading),
                                turn_radius(turn_radius),
                                rotation(rotation){};

                        Member(){};

                        ~Member(){};

                        void set_heading(double heading){this->heading = heading;}
                        void set_turn_radius(double turn_radius){this->turn_radius = turn_radius;}
                        void set_rotation(int rotation){this->rotation = rotation;}

                        double get_heading() const {return heading;}
                        double get_turn_radius() const {return turn_radius;}
                        int get_rotation() const {return rotation;}
        };



/*
 * \class Trajectory
 *
 * \brief Hold the data regarding to the design of an specific
 * trajectory.
 *
 */

        class Trajectory{

                private:
                        int coord_precision;
                        double length;
                        std::vector<double> initial_point;
                        std::vector<double> last_point;
                        std::shared_ptr<std::vector<std::vector<double>>> coordinates;

                public:
                        Trajectory(int coord_precision,
                                   std::vector<double> initial_point,
                                   std::vector<double> last_point):
                                coord_precision(coord_precision),
                                initial_point(initial_point),
                                last_point(last_point){};

                        Trajectory(){};

                        ~Trajectory(){};

                        void set_coord_precision(int coord_precision){this->coord_precision = coord_precision;}
                        void set_length(double length){this->length = length;}
                        void set_initial_point(std::vector<double> initial_point){
                                this->initial_point = initial_point;
                        }
                        void set_last_point(std::vector<double> last_point){
                                this->last_point = last_point;
                        }

                        void set_coordinates(std::shared_ptr<std::vector<std::vector<double>>> coordinates){
                                this->coordinates = coordinates;
                        }

                        int get_coord_precision() const {return coord_precision;}
                        double get_length() const {return length;}
                        std::vector<double> get_initial_point() const {return initial_point;}
                        std::vector<double> get_last_point() const {return last_point;}
                        std::shared_ptr<std::vector<std::vector<double>>> get_coordinates() const {
                                return coordinates;
                        }

        };

        
/* \class Segment
 *
 * \brief Calculates the elemental dubins path based on an analytical
 * 2 dimensional geometry
 *
 */

        class Segment{

                private:
                        std::vector<std::shared_ptr<Member>> member;
                        std::shared_ptr<Member> turn_member = NULL;
                        std::shared_ptr<Member> straight_member = NULL;
                        std::shared_ptr<Trajectory> trajectory;
                        std::vector<std::vector<double>> center_coords;


                        /* \fn classify_member_category()
                         *
                         * \brief Determine if the member struct corresponds
                         * to a turn or a straight segment
                         *
                         */

                        void classify_member_category();


                        /* \fn heading_conversor()
                         *
                         * \brief Performs a change in heading value as
                         * a function of cuadrant
                         *
                         */

                        double heading_conversor(double heading);


                        /* \fn heading_range()
                         *
                         * \brief Calculates the travelled range accounting
                         * the quadrant and the rotation direction
                         *
                         */

                        double heading_range();


                        /* \fn calculate_length_turn()
                         *
                         * \brief Calculates the length of the
                         * turn
                         *
                         */


                        void calculate_length_turn();


                       /* \fn calculate_length_straight()
                         *
                         * \brief Calculates the length of an
                         * straight segment

                         */

                        void calculate_length_straight();


                        /* \fn calculate_coordinates_turn()
                         *
                         * \brief Calculates the coordinates corresponding
                         * to a circular segment
                         *
                         */

                        void calculate_coordinates_turn();


                       /* \fn calculate_coordinates_straight()
                         *
                         * \brief Calculates the coordinates corresponding
                         * to a straight segment
                         */

                        void calculate_coordinates_straight();


                public:
                        Segment(){};

                        Segment(std::vector<std::shared_ptr<dubins::Member>> member,
                                std::shared_ptr<dubins::Trajectory> trajectory,
                                std::vector<std::vector<double>> center_coords):
                                member(member),
                                trajectory(trajectory),
                                center_coords(center_coords){};

                        ~Segment(){};

                        double calculate_length();
                        std::shared_ptr<std::vector<std::vector<double>>> calculate_coords();

        };


/* \class Dubins
 *
 * \brief Interface to calculate a Dubins trajectory
 *
 */
        class Dubins{

                private:
                        int path_type; /**< (-1): longest path
                                        *   (1): shortest_path */

                        std::vector<std::shared_ptr<dubins::Member>> member; /**< Member stats
                                                                      * along the planned
                                                                      * trajectory */
                        std::shared_ptr<dubins::Trajectory> trajectory; /**< Dubins path
                                                                              * waypoints */

                        double length;
                        std::shared_ptr<std::vector<std::vector<double>>> coordinates;





                        /* \fn circumference_center()
                         *
                         * \brief Calculates the coordinates of the
                         * turning circumference center position
                         *
                         */

                        std::vector<double> circumference_center(std::shared_ptr<dubins::Member> member,
                                                                 std::vector<double> point);


                        /* \fn transfer_angle()
                         *
                         * \brief Calculates the transfer angle between the
                         * two turns
                         *
                         */

                        double transfer_angle(std::vector<double>& start_turn_center_coords,
                                              std::vector<double>& end_turn_center_coords);


                        /* \fn length_single_trajectory()
                         *
                         * \brief Calculates the total length of a
                         * specific Dubins trajectory as a measure of
                         * performance of that trajectory
                         *
                         * \return length of trajectory
                         *
                         */

                        double length_single_trajectory(std::vector<int> rotations);


                        /* \fn coordinates_single_trajectory()
                         *
                         * \brief Calculates the coordinates of a
                         * specific Dubins trajectory
                         *
                         * \return coordinates of trajectory
                         *
                         */

                        std::shared_ptr<std::vector<std::vector<double>>> coordinates_single_trajectory(std::vector<int> rotations);


                        /* \fn longest_path()
                         *
                         * \brief Iteratively reviews all the possible Dubins paths
                         * and select the longest
                         *
                         */

                        void longest_path();


                        /* \fn shortest_path()
                         *
                         * \brief Iteratively reviews all the possible Dubins paths
                         * and select the shortest
                         *
                         */

                        void shortest_path();


                        /* \fn coordinate_conversor()
                         *
                         * \brief Rearranges the coordinates vector in order to return as
                         * rows for waypoints and columns for axes
                         *
                         */

                        void coordinate_conversor();


                public:
                        Dubins(int path_type,
                               std::vector<std::shared_ptr<dubins::Member>> member,
                               std::shared_ptr<dubins::Trajectory> trajectory):
                                path_type(path_type),
                                member(member),
                                trajectory(trajectory){};

                        ~Dubins(){};

                        void compute_trajectory();
                        double get_length() const {return length;}
                        std::shared_ptr<std::vector<std::vector<double>>> get_coordinates() const {
                                return coordinates;
                        }

        };

};

#endif // DUBINS_H_


// TODO:
// 1. Divide Segment class into a TurnSegment and StraightSegment classes
// 2. Add method "circumference_center()" into TurnSegment class
// 3. Add method "transfer_angle()" into StraightSegment class
// 4. Remove the shared_ptr option of rotation atribute in Member class
