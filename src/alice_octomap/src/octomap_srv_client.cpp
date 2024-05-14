#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/Octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "octomap_service_client");

    // Create a global ROS handle (nh)
    ros::NodeHandle nh;

    // Create a client that calls to service node "Octomap_binary"
    ros::ServiceClient octomap_binary_client = nh.serviceClient<octomap_msgs::GetOctomap>("octomap_binary");

    // Create a service call to retrieve octomap information
    octomap_msgs::GetOctomap srv;

    ROS_INFO("Attempting to call service");

    // Call the GetOctomap service
    if (octomap_binary_client.call(srv))
    {
        // If the call succeeded
        ROS_INFO("Service call succeeded");

        // Service call succeeded, process the received octomap
        // octomap now contains the (full) octomap from the service call
        octomap_msgs::Octomap octomap = srv.response.map;

        // Convert the octomap into an octree for easier processing
        // octree is a pointer variable. Dynamic_cast casts the result of msgToMap(octomap) which is of type AbstractOcTree to type Octree
        octomap::AbstractOcTree *abstract_octree = octomap_msgs::msgToMap(octomap);
        octomap::OcTree *octree = dynamic_cast<octomap::OcTree *>(abstract_octree);

        // Checks if the pointer variable octree is not a NULL pointer (so conversion is )
        if (octree)
        {
            int totalVoxels = 0;
            int seenVoxels = 0;

            for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
            {

                // access node, eg:
                // std::cout << "Node center: " << it.getCoordinate();
                // std::cout << "value: " << it->getValue() << "\n";

                totalVoxels++;

                if (octree->isNodeOccupied(*it))
                {
                    seenVoxels++;
                }
            }

            std::cout << "amount of voxels: " << totalVoxels;
            std::cout << "amount of occupied voxels: " << seenVoxels;

            // double seenPercentage = (seenVoxels / (totalVoxels)) * 100.0;
            // std::cout << "Percentage of voxels seen: " << seenPercentage << "%" << std::endl;

            delete octree;
        }
    }

    else
    {

        // Service call failed
        ROS_ERROR("Failed to call GetOctomap service. Error: %s", octomap_binary_client.getService().c_str());
    }

    ros::spin();

    return 0;
}