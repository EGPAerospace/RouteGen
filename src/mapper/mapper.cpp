#include "mapper.hpp"

/*
 * namespace : mapper
 * class : Mesher
 * method : discretize_coordinate()
 *
 */

void mapper::Mesher::discretize_coordinate(std::shared_ptr<std::vector<double>>& coords,
                                           double size,
                                           int precision){

    (*coords).push_back(0.0);

    for (int i = 1; i < precision + 1; i++){
        (*coords).push_back((*coords)[i - 1] + (size / precision));
    };

};


/*
 * namespace : mapper
 * class : Mesher
 * method : generate_mesh()
 *
 */

void mapper::Mesher::generate_mesh(){

    x_coords = std::make_shared<std::vector<double>>(this->precision_x);
    y_coords = std::make_shared<std::vector<double>>(this->precision_y);

    discretize_coordinate(x_coords, width, precision_x);
    discretize_coordinate(y_coords, height, precision_y);

};


/*
 * namespace : mapper
 * class : Mesher
 * method : get_node_coordinates()
 *
 */

std::vector<double> mapper::Mesher::get_node_coordinates(int index_x,
                                                         int index_y){

    std::vector<double> coords;

    coords[0] = (*this->x_coords)[index_x];
    coords[1] = (*this->y_coords)[index_y];

    return coords;
};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : size_dependent_parameter()
 *
 */

double mapper::DiamondSquare::size_dependent_parameter(double size_dim){

    double parameter;

    parameter = log(size_dim - 1.0) / log(2.0);

    return parameter;

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : point_elevation()
 *
 */

double mapper::DiamondSquare::point_elevation(std::vector<double>& enclosing_elevations,
                                              int iteration){

    double mean_elevation;
    double random_value;
    double scale_factor;
    double point_elevation;
    double min_val;
    double max_val;

    /* Calculate average elevation of local point */
    mean_elevation = std::accumulate(enclosing_elevations.begin(), enclosing_elevations.end(), 0.0) / enclosing_elevations.size();

    /* Calculate random value */
    min_val = - pow(this->roughness, iteration);
    max_val = pow(this->roughness, iteration);
    random_value = random_utils::interval_random_generator(min_val, max_val);

    /* Calculate scale factor */
    scale_factor = this->roughness * this->size_param[0] * this->size_param[1];

    point_elevation = mean_elevation + random_value * scale_factor;

    return point_elevation;
};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : grid_to_elevation_index()
 *
 */

int mapper::DiamondSquare::grid_to_elevation_index(std::vector<int>& grid_indexes){

    int elevation_index;

    elevation_index = grid_indexes[0] + grid_indexes[1] * this->precision[1];

    return elevation_index;

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : compute_diamond()
 *
 */

void mapper::DiamondSquare::compute_diamond(std::vector<std::vector<int>>& subgrid_indexes,
                                            std::vector<double>& subgrid_elevations,
                                            int iteration){

    int index_x;
    int index_y;
    int elevation_index_diamond;
    double elevation_value;
    std::vector<int> diamond_index;

    /* Calculate diamond center value index*/
    /* x_axis index */
    index_x = subgrid_indexes[0][0] + (subgrid_indexes[1][0] - subgrid_indexes[0][0]) / 2;
    diamond_index.push_back(index_x);

    /* y_axis index */
    index_y = subgrid_indexes[0][1] + (subgrid_indexes[1][1] - subgrid_indexes[0][1]) / 2;
    diamond_index.push_back(index_y);
    elevation_index_diamond = grid_to_elevation_index(diamond_index);

    /* Calculate elevation of diamond */
    elevation_value = point_elevation(subgrid_elevations, iteration);

    /* Storing resulting diamond elevation value */
    (*this->elevation_profile)[elevation_index_diamond] = elevation_value;

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : compute_square()
 *
 */

void mapper::DiamondSquare::compute_square(std::vector<std::vector<int>>& subgrid_indexes,
                                           std::vector<double>& subgrid_elevations,
                                           int iteration){

    std::vector<int> index_x; /* index of x_coordinate corresponsing to a
                               * mid part of a specific square segment (left, mid, right) */

    std::vector<int> index_y; /* index of y_coordinate corresponding to a
                               * mid part of a specific square segment (bottom, mid, top) */

    int elevation_index_mid_segment; /* corresponding index of the elevation vector for each segment mid part */
    double elevation_mid_segment; /* elevation at mid part of the segment for each segment */
    std::vector<int> mid_segment_index; /* stores indexes (x, y) for each segment surrounding the square */
    std::vector<double> elevations_extremes; /* elevation at extremes of the segment for each segment */
    std::vector<std::vector<int>> mid_segment_index_list {{1, 0}, {2, 1}, {1, 2}, {0, 1}};
    std::vector<std::vector<int>> subgrid_segment_index_list {{0, 1}, {1, 2}, {2, 3}, {3, 0}};

    /* Compute indexes for each segment midpoint */
    /* x_axis index */
    index_x.push_back(subgrid_indexes[0][0]);
    index_x.push_back(subgrid_indexes[0][0] + (subgrid_indexes[1][0] - subgrid_indexes[0][0]) / 2);
    index_x.push_back(subgrid_indexes[1][0]);

    /* y_axis index */
    index_y.push_back(subgrid_indexes[0][1]);
    index_y.push_back(subgrid_indexes[0][1] + (subgrid_indexes[1][1] - subgrid_indexes[0][1]) / 2);
    index_y.push_back(subgrid_indexes[1][1]);


    /* Calculating each segment on the subgrid square (the order is bottom, right, top, left) */
    for (int i = 0; i < 4; i++){

        mid_segment_index.push_back(mid_segment_index_list[i][0]);
        mid_segment_index.push_back(mid_segment_index_list[i][1]);
        elevation_index_mid_segment = grid_to_elevation_index(mid_segment_index);
        elevations_extremes.push_back(subgrid_elevations[subgrid_segment_index_list[i][0]]);
        elevations_extremes.push_back(subgrid_elevations[subgrid_segment_index_list[i][1]]);
        elevation_mid_segment = point_elevation(elevations_extremes, iteration);

        /* Storing resulting diamond elevation value */
        (*this->elevation_profile)[elevation_index_mid_segment] = elevation_mid_segment;

        mid_segment_index.clear();
        elevations_extremes.clear();
    };

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : compute_subgrid()
 *
 */

void mapper::DiamondSquare::compute_subgrid(std::vector<std::vector<int>>& subgrid_indexes,
                                            int iteration){

    int elevation_indexes_subgrid;
    std::vector<double> subgrid_elevations;

    /* Get elevations of subgrid indexes */
    for (unsigned i = 0; i < subgrid_indexes.size(); i++){

        /* Calculating index in the elevation vector for specific grid point */
        elevation_indexes_subgrid = grid_to_elevation_index(subgrid_indexes[i]);

        /* Get the corresponding elevation value from index */
        subgrid_elevations[i] = (*this->elevation_profile)[elevation_indexes_subgrid];

    };

    /* Calculate diamond */
    compute_diamond(subgrid_indexes, subgrid_elevations, iteration);

    /* Calculate square */
    compute_square(subgrid_indexes, subgrid_elevations, iteration);

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : compute_iteration()
 *
 */

void mapper::DiamondSquare::compute_iteration(int iteration_x,
                                              int iteration_y){

    int subgrids_x;
    int subgrids_y;
    int start_x;
    int start_y;
    std::vector<std::vector<int>> subgrid_indexes(4, std::vector<int>(2));

    /* Calculate the number of subgrids of this iteration */
    subgrids_x = pow(2, iteration_x - 1);
    subgrids_y = pow(2, iteration_y - 1);

    start_x = 0;
    start_y = 0;

    /* Iterating for each subgrid y_coordinates */
    for (int j = 1; j == subgrids_y; j++){

        /* Bottom left corner */
        subgrid_indexes[0][1] = start_y;

        /* Bottom right corner */
        subgrid_indexes[1][1] = start_y;

        /* Top right corner */
        subgrid_indexes[2][1] = start_y + subgrids_y;

        /* Top left corner */
        subgrid_indexes[3][1] = start_y + subgrids_y;

        /* Iterating for each subgrid x_coordinates */
        for (int i = 1; i == subgrids_x; i++ ){

            /* Bottom left corner */
            subgrid_indexes[0][0] = start_x;

            /* Bottom right corner */
            subgrid_indexes[1][0] = start_x + subgrids_x;

            /* Top right corner */
            subgrid_indexes[2][0] = start_x + subgrids_x;

            /* Top left corner */
            subgrid_indexes[3][0] = start_x;

            /* Computing each subgrid properties */
            compute_subgrid(subgrid_indexes, iteration_x);

            start_x += subgrids_x;
        };

        start_x = 0;
        start_y += subgrids_y;

    };

};


/*
 * namespace : mapper
 * class : DiamondSquare
 * method : generate_profile()
 *
 */

std::shared_ptr<std::vector<double>> mapper::DiamondSquare::generate_profile(){

    int elevation_index;
    int size_elevations = this->precision[0] * this->precision[1];
    this->elevation_profile = std::make_shared<std::vector<double>>(size_elevations);
    std::vector<std::vector<int>> corner_indexes{ {0, 0},
                                                  {this->precision[0] - 1, 0},
                                                  {this->precision[0] - 1, this->precision[1] - 1},
                                                  {0, this->precision[1] - 1} };

    /* Calculate size dependent parameter for each axis */
    (this->size_param).push_back(size_dependent_parameter(this->precision[0]));
    (this->size_param).push_back(size_dependent_parameter(this->precision[1]));

    /* Set random values at the corner */
    for (unsigned i = 0; i < corner_indexes.size(); i++){
        elevation_index = grid_to_elevation_index(corner_indexes[i]);
        (*this->elevation_profile)[elevation_index] = random_utils::interval_random_generator(elevation_contraints[0], elevation_contraints[1]);
    };


    /* Computing each iteration */
    for (int j = 1; j < this->size_param[1] + 1; j++){
        for (int i = 1; i < this->size_param[0] + 1; i++){
            compute_iteration(i, j);
        }
    }

    return this->elevation_profile;

};



/*
 * namespace : mapper
 * class : Terrain
 * method : generation_terrain()
 *
 */

void mapper::Terrain::generate_terrain(){

    std::shared_ptr<Mesher> terrain_mesh;
    std::shared_ptr<DiamondSquare> diamond_square_algorithm;

    /* Generating mesh */
    terrain_mesh = std::make_shared<Mesher>(this->size_grid[0],
                                            this->size_grid[1],
                                            this->precision[0],
                                            this->precision[1]);

    terrain_mesh->generate_mesh();

    /* Run diamond-square algorithm */
    diamond_square_algorithm = std::make_shared<DiamondSquare>(this->precision,
                                                               this->elevation_contraints,
                                                               this->roughness,
                                                               terrain_mesh);

    this->elevation_profile = diamond_square_algorithm->generate_profile();

};


/*
 * namespace : mapper
 * class : Surface
 * method : load_terrain_input_data()
 *
 */

void mapper::Surface::load_terrain_input_data(){

    std::shared_ptr<Terrain> terrain_data;


    terrain_data = std::make_shared<Terrain>();


};


/*
 * namespace : mapper
 * class : Surface
 * method : generate_surface()
 *
 */

void mapper::Surface::generate_surface(){

    this->terrain->generate_terrain();



};
