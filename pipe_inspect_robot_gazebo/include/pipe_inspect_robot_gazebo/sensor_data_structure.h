#ifndef SENSOR_DATA_STRUCTURE_H
#define SENSOR_DATA_STRUCTURE_H

#include <ros/ros.h>
#include <std_msgs/Int8.h>
#include <cmath>
#include <vector>

class SLPointStructure{ //point contains information about a xyz point, and degree
    /*
    The SLPointStructure class holds information about a point used in the structured light sensor
    */
    public:
        float x_val; //x coordinate of point relative to camera
        float y_val; //y coordinate of point relative to camera
        float z_val; //z coordinate of point relative to camera
        int index_in_ring; // Index of point in the ring that it belongs to
        int degree_value; // degree value assocated with the location of the point in the ring.

        float get_radius(); //method to return the radius from the center of the camera frame using the coordinate locations
}; 

class SLRingStructure{
    /*
    The SLPointStructure class holds information about the ring structure, which holds points. 
    There can be several rings in a single structure light sensor object.
    */
    private:
        std::vector<SLPointStructure> ring_of_points; // Current ring of points, a vector of points
        std::vector<int> indexes_to_check; // a vector of the indexes in the ring whose points need to be searched
        int points_in_ring; // The number of points in the ring. 
        int index_search_start; // The starting index in the ring to check the radius
        int search_size;// the number of indexes that need to be searched on either side of the point that is currently being checked. 
        float ring_current_pipe_radius; // current pipe radius inferred from the ring of points. 
        float min_ring_radius; // minimum radius given from the point that is closest to the central axis of the camera frame.
        float max_ring_radius; // maximum radius for current ring.
        float search_radius_threshold; // Radius in meters. When a point has a radius above this minimum threshold, then that point is considered to be apart of a measurement of a new pipe direction.
        
        int wire_start_index; // Index of where the camera wire starts
        int wire_length_index; // Length in indices of the camera wire.  
        int sample_every_n_degree; //constant which determines how many degrees are between each sampled point
        

        void initialize_quick_search(); // Method to initialize the ring and its data to prepare it to quickly search for nearby pipe directions
        void quick_set_pipe_radius(); // Method to set the pipe radius given the set of points 
        void quick_search_for_max_radii_with_index(); // Method to search for the maximum radius and the index
        SLPointStructure find_max_point_from_surrounding_points(SLPointStructure point_at_index); // determines the larger radius point given a current point by checking the points around this point. 
        void add_point_to_max_radius_points(SLPointStructure point); //Method to add the point to the ring's maximum radius points. These points should represent distinct pipe directions
        

    public: 
        std::vector<SLPointStructure> max_radius_points; // Vector of points containing the maximum radius values
        bool point_near_eachother_check(SLPointStructure point_1, SLPointStructure point_2); // Method to check if two points within a certian degree threshold of eachother.
        SLRingStructure(int points_in_ring, float min_ring_radius, float max_ring_radius, float search_radius_threshold, int sample_every_n_degree, int wire_length_index, int wire_start_index); // Initailizer
        void add_point(SLPointStructure xyz_point); // Method to add a point to the ring 
        void determine_max_radius_and_index(); //Method to determine the max radius of the ring and the given index of that point
        int convert_from_index_to_degree(int index); //Method to convert from a points index in the ring to a degree value. This value is eventually published and used by the control nodes.
        
        

};

class SLSensorStructure{ 
    /*
    The class SLSensorStructure contains methods and attributes related to the entire structure light data structure. 
    The sensor consists of a set of rings held in a vector. 
    */
    
    private:
        std::vector<SLRingStructure> set_of_rings;//A vector holding the set of rings
        int determine_degree_average_of_points(std::vector<SLPointStructure> points_vector) ; //A method to average the maximum radius values (in other words pipe directions) from each ring 
    public:
        void add_ring(SLRingStructure ring); // Method to add ring to the data structure
        std::vector<float>  determine_direction_options(); //Method to determine the direction options give the data from the rings. 
        
};
#endif