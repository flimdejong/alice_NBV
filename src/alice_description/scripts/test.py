import rospy


if __name__ == '__main__':
    rospy.init_node("test_node") #A node is inside an executable

    rospy.loginfo("Hello!")


    rospy.sleep(1)

    rospy.loginfo("End of program!")