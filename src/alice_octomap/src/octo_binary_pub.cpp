#include <ros/ros.h>
#include <octomap/octomap.h>
#include <octomap_msgs/conversions.h>
#include <octomap_msgs/Octomap.h>
#include <std_msgs/Int32.h>

ros::Publisher seen_voxels_pub;

void octomapCallback(const octomap_msgs::Octomap::ConstPtr& octomap_msg)
{
    // Process the received Octomap message
    octomap::AbstractOcTree* abstract_octree = octomap_msgs::msgToMap(*octomap_msg);
    octomap::OcTree* octree = dynamic_cast<octomap::OcTree*>(abstract_octree);

    if (octree)
    {
        int totalVoxels = 0;
        int seenVoxels = 0;
        for (octomap::OcTree::leaf_iterator it = octree->begin_leafs(), end = octree->end_leafs(); it != end; ++it)
        {
            totalVoxels++;
            if (octree->isNodeOccupied(*it))
            {
                seenVoxels++;
            }
        }

        std::cout << "Amount of voxels: " << totalVoxels << std::endl;
        std::cout << "Amount of occupied voxels: " << seenVoxels << std::endl;

        // Publish the amount of seen voxels
        std_msgs::Int32 seen_voxels_msg;
        seen_voxels_msg.data = seenVoxels;
        seen_voxels_pub.publish(seen_voxels_msg);

        delete octree;
    }
}

int main(int argc, char** argv)
{
    // Initialize node
    ros::init(argc, argv, "octomap_subscriber");

    // Create a global ROS handle (nh)
    ros::NodeHandle nh;

    // Create a subscriber to the octomap_binary topic
    ros::Subscriber octomap_sub = nh.subscribe<octomap_msgs::Octomap>("octomap_binary", 1, octomapCallback);

    // Create a publisher to send the data to the publisher node
    seen_voxels_pub = nh.advertise<std_msgs::Int32>("seen_voxels", 1);

    // Spin and wait for Octomap messages
    ros::spin();

    return 0;
}