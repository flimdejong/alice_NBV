#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/GetOctomap.h>

int main(int argc, char **argv)
{
    // Initialize node
    ros::init(argc, argv, "octomap_client");

    // Create a global ROS handle (nh)
    ros::NodeHandle nh;

    //Create a client that calls to service node "Octomap_binary"
    ros::ServiceClient octomap_binary_client = nh.serviceClient<octomap_msgs::GetOctomap> ("octomap_binary");

    // Createa a service call to retrieve octomap information
    octomap_msgs::GetOctomap srv;

    // Call the GetOctomap service
    if (octomap_binary_client.call(srv))
    {
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

            for (octomap::OcTree::iterator it = octree->begin(), end = octree->end(); it != end; ++it)
            {
                totalVoxels++;
                if (octree->isNodeOccupied(*it))
                {
                    seenVoxels++;
                }
            }

            double seenPercentage = (seenVoxels / (totalVoxels)) * 100.0;
            std::cout << "Percentage of voxels seen: " << seenPercentage << "%" << std::endl;

            delete octree;
        }
    }

    else {
        
        // Service call failed
        ROS_ERROR("Failed to call GetOctomap service");
    }

    ros::spin();

    return 0;
}
