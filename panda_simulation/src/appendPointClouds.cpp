/*Referenced move_group_interface_tutorial.cpp
and https://github.com/strawlab/perception_pcl/blob/master/pcl_ros/src/tools/pointcloud_to_pcd.cpp
*/

#include <string>
#include <ros/ros.h>
#include <pcl/io/io.h>
#include <pcl/io/pcd_io.h>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <sensor_msgs/PointCloud2.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <pcl_conversions/pcl_conversions.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>
#include <pcl/point_cloud.h>
#include <moveit_visual_tools/moveit_visual_tools.h>
#include <pcl/point_types.h>

using namespace std;

// The circle constant tau = 2*pi. One tau is one rotation in radians.
const double tau = 2 * M_PI;
bool saveImage = false;
string imName;
sensor_msgs::PointCloud2 cloud1;
sensor_msgs::PointCloud2 cloud2;
sensor_msgs::PointCloud2 out1;
int numTimes;
std::string prefix_;
  int moveToPose(float x, float y, float z, float roll, float pitch, float yaw){
    // Setup
    static const std::string PLANNING_GROUP = "panda_arm";
    moveit::planning_interface::MoveGroupInterface move_group_interface(PLANNING_GROUP);
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  
    // Visualization
    namespace rvt = rviz_visual_tools;
    moveit_visual_tools::MoveItVisualTools visual_tools("panda_link0");
    visual_tools.deleteAllMarkers();
    visual_tools.loadRemoteControl();
    Eigen::Isometry3d text_pose = Eigen::Isometry3d::Identity();
    text_pose.translation().z() = 1.0;
    visual_tools.publishText(text_pose, "MoveRobot Demo", rvt::WHITE, rvt::XLARGE);
    visual_tools.trigger();

    // Getting Basic Information
    //Print the name of the reference frame for this robot.
    ROS_INFO_NAMED("tutorial", "Planning frame: %s", move_group_interface.getPlanningFrame().c_str());

    //Print the name of the end-effector link for this group.
    ROS_INFO_NAMED("tutorial", "End effector link: %s", move_group_interface.getEndEffectorLink().c_str());

    //Get a list of all the groups in the robot:
    ROS_INFO_NAMED("tutorial", "Available Planning Groups:");
    std::copy(move_group_interface.getJointModelGroupNames().begin(),
            move_group_interface.getJointModelGroupNames().end(), std::ostream_iterator<std::string>(std::cout, ", "));

    //Planning to a Pose goal
    geometry_msgs::Pose target_pose1;
    tf2::Quaternion q;
    q.setRPY(roll, pitch, yaw);
    q = q.normalize();
    target_pose1.orientation.x = q[0];
    target_pose1.orientation.y = q[1];
    target_pose1.orientation.z = q[2];
    target_pose1.orientation.w = q[3];
    target_pose1.position.x = x;
    target_pose1.position.y = y;
    target_pose1.position.z = z;
    move_group_interface.setPoseTarget(target_pose1);

    //Call the planner to compute the plan and visualize it.
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;

    bool success = (move_group_interface.plan(my_plan) == moveit::planning_interface::MoveItErrorCode::SUCCESS);

    //Execute the trajectory stored in my_plan
    ROS_INFO_NAMED("tutorial", "Executing trajectory");
    move_group_interface.execute(my_plan);
    return 0;
  }

  void camCallBack(const sensor_msgs::PointCloud2ConstPtr& cloud){
    if(saveImage){
      if ((cloud->width * cloud->height) == 0)
        return;

      ROS_INFO_NAMED("tutorial", "Received %d data points in frame %s with the following fields: %s",
                (int)cloud->width * cloud->height,
                cloud->header.frame_id.c_str (),
                pcl::getFieldsList (*cloud).c_str ());

      ROS_INFO_NAMED("tutorial", "Data saved to %s", imName.c_str());

      pcl::io::savePCDFile (imName, *cloud, Eigen::Vector4f::Zero (),
                            Eigen::Quaternionf::Identity (), false);
      saveImage = false;
      if(numTimes == 1){
        cloud1.width = cloud->width;
        cloud1.height = cloud->height;
        cloud1.header.frame_id = cloud->header.frame_id;
        cloud1.data = cloud->data;
      }
    }
  }

  void capturePCImage(int num){
    saveImage = true;
    string numString = to_string(num);
    imName = "PCD_Image" + numString + ".pcd";
    numTimes = num;
    }
  void combineClouds(sensor_msgs::PointCloud2 in1, sensor_msgs::PointCloud2 in2, sensor_msgs::PointCloud2 out){
     ROS_INFO_NAMED("tutorial", "FrameID: %s", in1.header.frame_id.c_str());
  }

int main(int argc, char** argv)
{
  ros::init(argc, argv, "move_robot");
  ros::NodeHandle node_handle;
  ros::Subscriber cam_sub = node_handle.subscribe("panda_camera/depth/points", 1, camCallBack);

  // ROS spinning must be running for the MoveGroupInterface to get information
  // about the robot's state. One way to do this is to start an AsyncSpinner
  // beforehand.
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  //Move to initial pose facing the robot
  moveToPose(0, -0.2, 0.9, 1.5, -0.8, 0);
  
  //Capture image
  capturePCImage(1);
  
  //Move to the left of the object
  moveToPose(-0.4, -0.2, 0.9, 1.5, -0.8, 0.5);
  
  //Capture image
  capturePCImage(2);
  
  //Move to the right of the object
  moveToPose(0.4, -0.2, 0.9, 1.5, -0.8, -0.5);
  
  //Caputre image
  capturePCImage(3);
  
  //Combine clouds 1 and 2
  //pcl::io::loadPCDFile ("PCD_Image1.pcd", cloud1);
  pcl::io::loadPCDFile ("PCD_Image2.pcd", cloud2);
  combineClouds(cloud1, cloud2, out1);
  ros::shutdown();
  return 0;
}
