
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cmath>
#include <random>
#include "std_msgs/String.h"
#include "std_msgs/Float64MultiArray.h"
#include "pipe_inspect_robot_gazebo/pipe_direction_options.h" //Custom Message File
#include <vector>
#include <map>
#include "pipe_inspect_robot_gazebo/sensor_data_structure.h"

// Create noise generator
std::default_random_engine generator;
std::normal_distribution<double> distribution(0,1.75);

class NoisySLSensor{
    /*
    Class NoisySLSensor contains methods and objects to create a simulated noisy (or not noisy) structured light data frame that published as a new ROS message
    */
    private:
    ros::Subscriber default_depth_image_subscriber; // ROS subscriber responsible for reading the default depth image from the default sensor declared in the URDF file
    ros::Publisher pclcloud_sensor_readings; // ROS publisher responbisble for outputting the raw data of the simulated depth sensor
    ros::Publisher pipe_direction_options_pub; // ROS publisher responsible for publishing the vector message containing the pipe direction options
    
    int wire_end_index; // Index in ring where wire ends
    int wire_length_index; //Length of wire 



    public:
    int use_noise; //Whether to use Gaussian noise in the sensor readings. Default is 1
    const int num_rings; // Number of rings in the data frame
    float wire_start; // Wire start location in radians
    float wire_length; //Wire length in radians
    int sample_every_n_degree; //Integer for how many degrees should be between each point that is sampled from the initial depth sensor image 
    std::vector<float> ring_location_factors; // Vector of floats for where the rings are located relative to the center of the depth image
    std::vector<float> direction_options; // Vector of floats for where there might be a new pipe direction 

    //Constructor:
    NoisySLSensor(ros::NodeHandle *node_handle, int num_rings, std::vector<float> ring_location_factors,float wire_start,float wire_length,int sample_every_n_degree) 
        : num_rings{num_rings}, ring_location_factors{ring_location_factors}, wire_start{wire_start}, wire_length{wire_length}, sample_every_n_degree{sample_every_n_degree}
    {
        default_depth_image_subscriber = node_handle->subscribe("/camera/depth/points", 1000, &NoisySLSensor::receive_default_depth_image_callback, this);
        pclcloud_sensor_readings = node_handle->advertise<sensor_msgs::PointCloud2>("/pipe_inspect_robot/raw_sl_sensor_readings_pcl", 1000);
        pipe_direction_options_pub = node_handle->advertise<pipe_inspect_robot_gazebo::pipe_direction_options>("/pipe_inspect_robot/pipe_direction_options", 1000);
    }

    //Callback function
    void receive_default_depth_image_callback(const sensor_msgs::PointCloud2ConstPtr& msg) {
        /*
        Function receives a raw depth sensor image frame, creates a structured light object from the frame, publishes the new strcutured light sensor data, and publishes the pipe direction options message

        Parameters: 
            msg: initial depth image from the default depth sensor defined in the URDF
        Returns:
            None
        */

        // Create SensorStructure Object. 
        pcl::PointCloud<pcl::PointXYZ> cloud;
        pcl::fromROSMsg(*msg, cloud); 
        pcl::PointCloud<pcl::PointXYZ> adjusted_cloud;
        sensor_msgs::PointCloud2 output;  

        //Create data structure to hold the data
        SLSensorStructure sl_sensor_structure;

        //Set initial conditions for previous points
        int prev_point_x =  1;
        int prev_point_y =  10000;
        
        //Get centers of image
        int c_x = cloud.height/2;
        int c_y = cloud.height/2;

        int wire_start_index;
        int wire_length_index;
        int point_counter;
        float noise = 0;
        this->use_noise = 0;
        std::vector<float> ring_radii;
        for (int x = 0; x < this->num_rings; x++){
            ring_radii.push_back(cloud.height * this->ring_location_factors[x]);
        }
        
        for (auto ring_radius : ring_radii){
            //For each ring factor, extract those points from that radius from the cloud image.
            float wire_length_index_f;
            wire_length_index_f = (this->wire_length * 180.0 / 3.14159) / this-> sample_every_n_degree ; 
            wire_length_index = round(wire_length_index_f);
            wire_start_index = this->sample_every_n_degree * this->wire_start;

            //Create a ring object
            SLRingStructure current_ring(0, 1000, 0, 1.5, this->sample_every_n_degree, wire_length_index, wire_start_index);
            int point_index_in_ring = 0;
            for (float angle = 0; angle < 6.28318; angle = angle + 0.01745*this->sample_every_n_degree){
                
                //Account for the wire length and wire start
                if (angle > this->wire_start && angle < this->wire_start + this->wire_length){
                    wire_end_index = point_counter;
                    wire_length_index++;
                    point_counter++;
                    continue;
                }

                //Create noise
                if (this->use_noise == 1){
                    noise = distribution(generator); //rand() % 11 + (-5);
                }
                else{
                    noise = 0;
                }

                int point_x = round((ring_radius+noise)*cos(angle));
                int point_y = round((ring_radius+noise)*sin(angle));

                if (point_x == prev_point_x && point_y == prev_point_y){
                    continue;
                }

                else{
                    adjusted_cloud.points.push_back(cloud.at(c_x + point_x, c_y + point_y));

                    //Create point
                    SLPointStructure current_point;
                    current_point.x_val = cloud.at(c_x + point_x, c_y + point_y).x;
                    current_point.y_val = cloud.at(c_x + point_x, c_y + point_y).y;
                    current_point.z_val = cloud.at(c_x + point_x, c_y + point_y).z;
                    current_point.index_in_ring = point_index_in_ring;
                    point_index_in_ring++;

                    current_ring.add_point(current_point);

                    prev_point_x =  point_x;
                    prev_point_y =  point_y;
                }
            }
            sl_sensor_structure.add_ring(current_ring);
        }

        pcl::toROSMsg(adjusted_cloud, output);
        output.header.frame_id = "SL_camera_link";

        //Output the simulated sensor data
        this->pclcloud_sensor_readings.publish(output);

        //Find and output the direction options vector
        this->direction_options = sl_sensor_structure.determine_direction_options();
        pipe_inspect_robot_gazebo::pipe_direction_options direction_message; 
        direction_message.pipe_direction_options_msg_vector = this->direction_options;
        this->pipe_direction_options_pub.publish(direction_message);
    }
};


int main (int argc, char **argv)
{
    //Create ROS Node
    ros::init(argc, argv, "simulated_sl_sensor");
    ros::NodeHandle node_handle;
    ros::Rate loop_rate(20);

    // Define simulated sensor parameters: 
    const int num_rings = 4;
    std::vector<float> ring_location_factors = {0.2 , 0.24, 0.3, 0.36, 0.4};;//  // {0.2 , 0.24, 0.3, 0.36, 0.4}; //must match num_rings 0.4 
    float wire_start = 1.57; //radians from positive x axis
    float wire_length = 0.1745*2; //"length" in radians of the wire that blocks the sl sensor projector light
    int sample_every_n_degree_deg = 4; //samples a point every x degrees around a ring. 

    //Initialize simulated sensor object
    NoisySLSensor noisy_sl_sensor = NoisySLSensor(&node_handle, num_rings, ring_location_factors, wire_start, wire_length, sample_every_n_degree_deg);

    //Repeat at 20Hz
    while (ros::ok())
    {
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    return 0;
}



