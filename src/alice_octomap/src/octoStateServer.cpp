#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>
#include <alice_octomap/octomap.h>

bool convertOcTree(alice_octomap::octomap::Request &req, alice_octomap::octomap::Response &res)
{

    ROS_INFO("Creating client to send a request to octomap_binary");

    // Create a service call to retrieve octomap information
    octomap_msgs::GetOctomap srv;

    //Create a NodeHandle for the client
    ros::NodeHandle nh;

    //Create a client that calls the octomap_binary server
    ros::ServiceClient client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");

    // Call the GetOctomap service
    if (client.call(srv))
    {
        // If the call succeeded
        ROS_INFO("Service call succeeded");

        // Service call succeeded, process the received octomap
        // octomap now contains the (full) octomap from the service call
        octomap_msgs::Octomap octomap = srv.response.map;

        // Define the bounding box
        octomap::point3d min(0, -1, 0);
        octomap::point3d max(2, 1, 1); // Coordinates are in meters

        // Convert the octomap into an octree for easier processing
        // octree is a pointer variable. Dynamic_cast casts the result of msgToMap(octomap) which is of type AbstractOcTree to type Octree
        octomap::AbstractOcTree *abstract_octree = octomap_msgs::msgToMap(octomap);
        octomap::OcTree *octree = dynamic_cast<octomap::OcTree *>(abstract_octree);

        // Create vectors to store the XYZ coordinates + occupancy information
        std::vector<double> x_values;
        std::vector<double> y_values;
        std::vector<double> z_values;

        //It's called unsigned char but it actually is <bool> kinda
        std::vector<unsigned char> occupancy;

        // Checks if the pointer variable octree is not a NULL pointer (so conversion is )
        if (octree)
        {
            int totalVoxels = 0;
            int occupiedVoxels = 0;
            int maxDepth = 5;

            for (octomap::OcTree::leaf_bbx_iterator it = octree->begin_leafs_bbx(min,max), end = octree->end_leafs_bbx(); it != end; ++it)
            {

                double x = it.getX();
                double y = it.getY();
                double z = it.getZ();

                //Check if node is occupied
                bool is_occupied = octree->isNodeOccupied(*it);

                //For each node, push the data into the vectors
                // x_values.push_back(x);
                // y_values.push_back(y);
                // z_values.push_back(z);
                // occupancy.push_back(is_occupied);

                std::cout << "X-Coordinate: " << x << std::endl;
                // std::cout << "Y-Coordinate: " << y << std::endl;
                // std::cout << "Z-Coordinate: " << z << std::endl;

                //To count total voxels
                totalVoxels++;

                //If it is occupied, add it to the occupiedVoxels vector
                if (is_occupied)
                {
                    occupiedVoxels++;
                }
            }

            // std::cout << "amount of voxels: " << totalVoxels << std::endl;
            // std::cout << "amount of occupied voxels: " << occupiedVoxels << std::endl;

            //res is the response from calling the server, what information do I get?
            res.total_voxels = totalVoxels;
            res.occupied_voxels = occupiedVoxels;
            res.x_values = x_values;
            res.y_values = y_values;
            res.z_values = z_values;
            res.occupancy = occupancy;

            delete octree;
        }

    //Return true if the service call succeeded
    return true;

    }

    else
    {

        // Service call failed
        ROS_ERROR("Failed to call octomap_binary service");

        //return false if there was an error
        return false;
    }

}

int main(int argc, char **argv) {

    ROS_INFO("Starting octoStateServer");

    // This node is a server that once called will send a client call to the octomap_binary server.
    // It returns the state of the octomap
    ros::init(argc, argv, "octoStateServer");

    // Create a global ROS handle (nh)
    ros::NodeHandle nh;

    // Create a server to send the information from the client to another node.
    // It takes as input the result of convertOcTree
    ros::ServiceServer service = nh.advertiseService("octoStateServer", convertOcTree);

    //spin is always necessary
    ros::spin();

    return 0;

}
