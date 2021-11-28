#ifndef MAPPER_H_
#define MAPPER_H_

#include <math.h>
#include <memory>
#include <numeric>
#include <random>
#include <vector>

#include "../../ext/Custom/random_utils.hpp"

namespace mapper{

/*
 * \class Mesher
 *
 * \brief Generates a 2D mesh required to build the map
 *
 */

    class Mesher{
        private:
            double width;
            double height;
            int precision_x;
            int precision_y;
            std::shared_ptr<std::vector<double>> x_coords;
            std::shared_ptr<std::vector<double>> y_coords;


            /* \fn discretize_coordinate()
             *
             * \brief Performs a regular mesh division given specific axis size
             * and precision
             *
             * \param coords are the specific axis coordinates that will be inserted
             * and modified in the same function
             *
             * \param size is the size of the mesh in specific axis
             *
             * \param precision are the total subdivisions for each axis
             *
             */

            void discretize_coordinate(std::shared_ptr<std::vector<double>>& coords,
                                       double size,
                                       int precision);

        public:
            Mesher(double width,
                   double height,
                   int precision_x,
                   int precision_y):
                width(width),
                height(height),
                precision_x(precision_x),
                precision_y(precision_y){};

            Mesher(){};

            ~Mesher(){};


            /* \fn generate_mesh()
             *
             * \brief Interface function to generate the mesh
             *
             */

            void generate_mesh();


            /* \fn get_node_coordinates()
             *
             * \brief Get node coordinates (x, y) by setting the corresponding
             * indexes (i, j)
             *
             */

            std::vector<double> get_node_coordinates(int index_x,
                                                     int index_y);

    };


/*
 * \class DiamondSquare
 *
 * \brief Includes the Diamond Square algorithm related functions to model
 * a fractal landscape
 *
 */

    class DiamondSquare{

        private:
            std::vector<int> size_param; /**< (x_coord, y_coord)*/
            std::vector<int> precision; /**< (x_coord, y_coord)*/
            std::vector<double> elevation_contraints; /**< (min elevation, max elevation)*/
            double roughness;
            std::shared_ptr<Mesher> mesh;
            std::shared_ptr<std::vector<double>> elevation_profile;


            /* \fn size_dependent_parameter()
             *
             * \brief Calculates the size parameter (n or m) as a function
             * of the number of grid subdivisions
             *
             * \param size_dim is the grid subdivision of a particular axis
             *
             */

            double size_dependent_parameter(double size_dim);


            /* \fn point_elevation()
             *
             * \brief Calculates the elevation of a particular point in the grid
             * as a function of surrounding values plus a random factor
             *
             * \param enclosing_elevations are the surrounding values of a particular
             * point
             *
             * \param iteration is the current iteration being processed
             *
             */

            double point_elevation(std::vector<double>& enclosing_elevations,
                                   int iteration);


            /* \fn grid_to_elevation_index()
             *
             * \brief Calculates the elevation of a particular point in the grid
             * as a function of surrounding values plus a random factor
             *
             * \param grid_indexes are the corresponding indexes in both x and y
             * coordinates to be converted into a single value index for the
             * elevation_profile vector
             *
             */

            int grid_to_elevation_index(std::vector<int>& grid_indexes);


            /* \fn compute_diamond()
             *
             * \brief Calculates the diamond index (center index) elevation
             *
             * \param subgrid_elevations cotains the elevation of the subgrid
             * enclosing points
             *
             * \param iteration is the current iteration being processed
             *
             */

            void compute_diamond(std::vector<std::vector<int>>& subgrid_indexes,
                                 std::vector<double>& subgrid_elevations,
                                 int iteration);


            /* \fn compute_square()
             *
             * \brief Calculates the square sides index elevations
             *
             * \param subgrid_elevations cotains the elevation of the subgrid
             * enclosing points
             *
             * \param iteration is the current iteration being processed
             *
             */

            void compute_square(std::vector<std::vector<int>>& subgrid_indexes,
                                std::vector<double>& subgrid_elevations,
                                int iteration);


            /* \fn compute_subgrid()
             *
             * \brief Calculates the diamond and square in a particular
             * subgrid
             *
             * \param subgrid_indexes are the point indexes enclosing the subgrid.
             * Those points are set in counterclockwise direction starting from the
             * bottom left
             *
             */

            void compute_subgrid(std::vector<std::vector<int>>& subgrid_indexes,
                                 int iteration);


            /* \fn compute_iteration()
             *
             * \brief Creates a set of predefined regions (subgrids) of the main
             * grid according to an specific iteration. Tose subgrids are defined
             * according to mesh indexes.
             *
             * \param input_index_points are all the mesh indexes (coordinates x,y)
             * required to compute the square and diamond of an specific iteration
             *
             * \param iteration is the current iteration being processed
             *
             */

            void compute_iteration(int iteration_x,
                                   int iteration_y);


        public:

            /* \fn generate_profile()
             *
             * \brief Interface function to generate the elevation profile
             * of all the surface applying the Square Diamond algorithm
             *
             */

            std::shared_ptr<std::vector<double>> generate_profile();

            DiamondSquare(std::vector<int> precision,
                          std::vector<double> elevation_contraints,
                          double roughness,
                          std::shared_ptr<Mesher> mesh):
                precision(precision),
                elevation_contraints(elevation_contraints),
                roughness(roughness),
                mesh(mesh){};

            DiamondSquare(){};

            ~DiamondSquare(){};

    };


/*
 * \class Terrain
 *
 * \brief Interface class to model procedural terrain
 *
 */

    class Terrain{

        private:
            int generation_type;
            std::vector<int> size_grid; /**< (x_coord, y_coord)*/
            std::vector<int> precision; /**< (x_coord, y_coord)*/
            std::vector<double> elevation_contraints; /**< (min elevation, max elevation)*/
            double roughness;
            std::shared_ptr<std::vector<double>> elevation_profile;

        public:
            Terrain(int generation_type,
                    std::vector<int> size_grid,
                    std::vector<int> precision,
                    std::vector<double> elevation_contraints,
                    double roughness):
                generation_type(generation_type),
                size_grid(size_grid),
                precision(precision),
                elevation_contraints(elevation_contraints),
                roughness(roughness){};

            Terrain(){};

            ~Terrain(){};


            /* \fn generate_terrain()
             *
             * \brief Interface function to generate terrain
             *
             */

            void generate_terrain();

            std::shared_ptr<std::vector<double>> get_elevation_profile() const {return elevation_profile;}




    };


/*
 * \class Surface
 *
 * \brief Interface class to generate a surface with all its features (terrain, lakes,
 * rivers, vegetation)
 *
 */

    class Surface{
        private:
            std::shared_ptr<Terrain> terrain;


            /* \fn load_terrain_input_data()
             *
             * \brief loads the terrain data reading values from a GUI, console
             * or text file
             *
             */

            void load_terrain_input_data();

        public:

            Surface(){};

            ~Surface(){};


             /* \fn generate_surface()
             *
             * \brief Interface function to generate surface
             *
             */

            void generate_surface();

    };



/*
 * \class Mapper
 *
 * \brief Interface for map generation. Includes synthetic surface modelling,
 * real surface modelling, interfacing with GUI, renderings, etc.
 *
 */

    class Mapper{

        private:

        public:
            Mapper(){};

            ~Mapper(){};


             /* \fn generate_map()
             *
             * \brief Interface function to generate map
             *
             */

            void generate_map();

    };









};


#endif // MAPPER_H_


//TODO
//1. SurfaceGenerator only applies the Square-Diamond algorithm.
//   Refactor to include other surface generation techniques
//2. In the future use gmsh for meshing and model map with Unity API
//3. DiamondSquare can handle only gris of same size. Change logic so
//   iteration in x_axis and iteration in y_axis are independent. This
//   must be done by changing a bit the generate_profile() and the compute iteration
