#include "pipe_inspect_robot_gazebo/sensor_data_structure.h"


void SLSensorStructure::add_ring(SLRingStructure ring_of_points){
    /*
    Function to add a ring to the sesnsor structure

    Parameters: 
        ring_of_points: ring structure to be added
    Returns:
        None
    */
    this->set_of_rings.push_back(ring_of_points);
}

int SLRingStructure::convert_from_index_to_degree(int index){
    /*
    Function to convert the index of a point to a degree within the ring

    Parameters: 
        index: ring structure to be added
    Returns:
        integer representing the degree value
    */
  if (index <= (this->wire_start_index)){
    // Then can simply return the index as the degree after converting
    return index*this->sample_every_n_degree;

  }

  // Otherwise need to shift the index up by the amount of the length, and convert
  else {
    return (index + this->wire_length_index)*sample_every_n_degree;
  }
  return 0;
  
}

std::vector<float> SLSensorStructure::determine_direction_options(){
    /*
    Function to determine the direction options

    Parameters: 
        None
    Returns:
        A vector of floats containing the direction options
    */
    std::vector<SLPointStructure> direction_1;
    std::vector<SLPointStructure> direction_2;
    std::vector<SLPointStructure> direction_3;
    std::vector<SLPointStructure> direction_4;

    // For each ring
    for (SLRingStructure ring: set_of_rings){

        //Determine the set of max radius values from that ring
        ring.determine_max_radius_and_index();

        //Place each of those values into discrete buckets, adding similar values to the same bucket
        for (SLPointStructure point : ring.max_radius_points){
            if (direction_1.empty()){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_1.push_back(point);
            }
            else if (ring.point_near_eachother_check(direction_1.at(0), point)){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_1.push_back(point);
            }

            else if (direction_2.empty()){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_2.push_back(point);
            }
            else if (ring.point_near_eachother_check(direction_2.at(0), point)){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_2.push_back(point);
            }

            else if (direction_3.empty()){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_3.push_back(point);
            }
            else if (ring.point_near_eachother_check(direction_3.at(0), point)){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_3.push_back(point);
            }
            else if (direction_4.empty()){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_4.push_back(point);
            }
            else if (ring.point_near_eachother_check(direction_4.at(0), point)){
                point.degree_value = ring.convert_from_index_to_degree(point.index_in_ring);
                direction_4.push_back(point);
            }

        }

    }

    //Now we determine the average index for each of the buckets and add to a vector
    std::vector<float> average_values_for_each_location;
    if (!direction_1.empty()){
        average_values_for_each_location.push_back(this->determine_degree_average_of_points(direction_1));
    }
    if (!direction_2.empty()){
        average_values_for_each_location.push_back(this->determine_degree_average_of_points(direction_2));
    }
    if (!direction_3.empty()){
        average_values_for_each_location.push_back(this->determine_degree_average_of_points(direction_3));
    }
    if (!direction_4.empty()){
        average_values_for_each_location.push_back(this->determine_degree_average_of_points(direction_4));
    }

    if (average_values_for_each_location.empty()){
        average_values_for_each_location.push_back(-2);
    }

    //Return the final vector from the information from each ring
    return average_values_for_each_location;

}

int SLSensorStructure::determine_degree_average_of_points(std::vector<SLPointStructure> points_vector){
    /*
    Function to determine the average of points from a vector.

    Parameters: 
        points_vector, vector of point objects whose degree needs to be averaged
    Returns:
        Integer representing the average degree value
    */

    int sum = 0;
    int degree = 0;
    for (SLPointStructure point : points_vector ){
        degree = point.degree_value;
        sum = sum + degree;
    } 
    return sum/points_vector.size();

}

void SLRingStructure::add_point(SLPointStructure xyz_point){
    /*
    Function to add a point to a ring structure

    Parameters: 
        xyz_point: point object
    Returns:
        None
    */
    this->ring_of_points.push_back(xyz_point);
    this->points_in_ring++;
}
void SLRingStructure::determine_max_radius_and_index(){
    /*
    Function determine the max radius and index values for a given ring

    Parameters: 
        None
    Returns:
        None
    */
    this->initialize_quick_search();
    this->quick_set_pipe_radius();
    this->quick_search_for_max_radii_with_index();
}

SLRingStructure::SLRingStructure(int points_in_ring, float min_ring_radius, float max_ring_radius, float search_radius_threshold, int sample_every_n_degree, int wire_length_index, int wire_start_index)
: points_in_ring{points_in_ring}, 
min_ring_radius{min_ring_radius}, 
max_ring_radius{max_ring_radius}, 
search_radius_threshold{search_radius_threshold}, 
sample_every_n_degree{sample_every_n_degree}, 
wire_length_index{wire_length_index},
wire_start_index{wire_start_index}
{

}

void SLRingStructure::initialize_quick_search(){
    /*
    Function initializes the quick search algorithm. Creates the set of indices, where the point's radius at each index are eventually checked to determine the max radius

    Parameters: 
        None
    Returns:
        None
    */

    this->search_size = floor(this->points_in_ring/9);
    if (search_size%2 == 0){
        this->search_size = this->search_size - 1;  //make odd number 
    }
    this->index_search_start = floor(this->search_size / 2);
    this->indexes_to_check = {  (this->index_search_start),
                                (this->index_search_start + 1* this->search_size), 
                                (this->index_search_start + 2* this->search_size), 
                                (this->index_search_start + 3* this->search_size),
                                (this->index_search_start + 4* this->search_size),
                                (this->index_search_start + 5* this->search_size),
                                (this->index_search_start + 6* this->search_size),
                                (this->index_search_start + 7* this->search_size),
                                (this->index_search_start + 8* this->search_size),
                                (this->index_search_start + 8* this->search_size + this->index_search_start)
                              };

    
}

void SLRingStructure::quick_set_pipe_radius(){
    /*
    Function uses the set of indexes created by the initialze quick search function and accesses the points at the index and determines the max radius

    Parameters: 
        None
    Returns:
        None
    */
    int index_counter = 0;
    for ( int index : this->indexes_to_check)  //Looping over indexes to check
    {
        
        if (index_counter == 9){ //Last step is skipped for now
            break;
        }

        float current_radius_in_question = this->ring_of_points.at(index).get_radius();
        
        //Checking and updating radius
        if (current_radius_in_question < this->min_ring_radius)
        {
            this->ring_current_pipe_radius = current_radius_in_question;
            this->min_ring_radius = current_radius_in_question;
        }
        index_counter++;
    }
}

void SLRingStructure::quick_search_for_max_radii_with_index(){
    /*
    Function to check each of the 10 points to see if they are greater than the current pipe radius.
    If they are, then that means there is likely a joint that we are currently near. 

    Parameters: 
        None
    Returns:
        None
    */

    std::vector<SLPointStructure> max_radius_points_in_ring;

    for (auto index : this->indexes_to_check)  //Looping over indexes to check
    {
        SLPointStructure point_at_index = this->ring_of_points.at(index);
        SLPointStructure point_with_max_radius_in_search_zone;
        float current_radius_in_question = point_at_index.get_radius();

        //Check radius against the threshold
        if ( current_radius_in_question > this->ring_current_pipe_radius * this->search_radius_threshold)
        {
            point_with_max_radius_in_search_zone = this->find_max_point_from_surrounding_points(point_at_index);
            this->add_point_to_max_radius_points(point_with_max_radius_in_search_zone);
        }
    }
}

void SLRingStructure::add_point_to_max_radius_points(SLPointStructure point){
    /*
    Function adds the point to the set of max radius points for the ring. 
    Checks if the point is already near the point within the threshold, and updates the point in the max radius points if a larger radius is found

    Parameters: 
        point: Point object that should be added
    Returns:
        None
    */

    //If structure is empty, add the point
    if (this->max_radius_points.empty()){
        this->max_radius_points.push_back(point);
        return;
    }

    //Check if the index is near the points in the vector already.
    int near_eachother_check = 0;
    int index_of_point_thats_near = -1;
    for (SLPointStructure max_radius_point : this->max_radius_points){
        index_of_point_thats_near++;
        if (this->point_near_eachother_check(max_radius_point, point)){
            //if they are near eachother, then we need to replace the one with the max radius.
            near_eachother_check = 1;
            break;
        }
        
    } 

    if (near_eachother_check == 1){
        //Then we need to compare radii to see which one is the better candidate for their being a pipe route
        if (point.get_radius() > this->max_radius_points.at(index_of_point_thats_near).get_radius()){
            this->max_radius_points.at(index_of_point_thats_near) = point;
        }
    }
    else{
        this->max_radius_points.push_back(point);
    }
}

bool SLRingStructure::point_near_eachother_check(SLPointStructure point_1, SLPointStructure point_2){
    /*
    Function checks to see if two points are near eachother by first relating their degree values and seeing if they are within a 120 degree threshold

    Parameters: 
        point_1: Point object to be compared
        point_2: Point object to be compared
    Returns:
        Bool based on whether the points are near eachother (True) or not (False)
    */

    if (point_1.index_in_ring < 60/this->sample_every_n_degree && point_2.index_in_ring > (300/this->sample_every_n_degree -this->wire_length_index)){
        return true;
    }
    else if (point_2.index_in_ring < 60/this->sample_every_n_degree && point_1.index_in_ring > (300/this->sample_every_n_degree -this->wire_length_index)){
        return true;
    }
    else if (abs(point_1.index_in_ring - point_2.index_in_ring) < this->ring_of_points.size()/3){
        return true;
    }
    else{
        return false;
    }
    return false;
}

SLPointStructure SLRingStructure::find_max_point_from_surrounding_points(SLPointStructure point_at_index){
    /*
    Function Checks the surrounding points of the input point to determine if there is a larger radius within that set of points

    Parameters: 
        point_at_index: point whose surrounding will be checked
    Returns:
        the maximum radius point object from the set or checked points
    */


    SLPointStructure max_radius_point;
    int index = point_at_index.index_in_ring;
    float max_radius = point_at_index.get_radius();
    for (int index_to_check = index - this->index_search_start; index_to_check <= index + this->index_search_start; index_to_check++){

        SLPointStructure point_at_index = this->ring_of_points.at(index_to_check);
        float current_radius_in_question = point_at_index.get_radius();
        if (current_radius_in_question >= max_radius){
            // Then this point is larger, and should be added to the map, but only if there is no other point within 45 degrees. 
            //Update max_ring_radius and associated point
            this->max_ring_radius = current_radius_in_question;
            max_radius = current_radius_in_question;
            max_radius_point = point_at_index;
        }
    }
    return max_radius_point;
}


float SLPointStructure::get_radius(){
    /*
    Function returns the radius of a specific point from the central axis of the camera frame(Z)

    Parameters: 
        None
    Returns:
        float representing the radius value of the point
    */

    return std::hypot(this->x_val, this->y_val);
}