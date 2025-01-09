#include <ros/ros.h>
#include <move_base_msgs/MoveBaseAction.h>
#include <actionlib/client/simple_action_client.h>
#include "Server.hpp"
#include "Eigen/Geometry"

struct dataPos {
    // Translation components
    float tx, ty, tz, distance;
    int direction;

    // Quaternion components for rotation
    //float qx, qy, qz, qw;

    // Constructor to initialize the Pose
    //qx(qx), qy(qy), qz(qz), qw(qw)
    dataPos(float tx, float ty, float tz, float distance, int direction)
        : tx(tx), ty(ty), tz(tz), distance(distance), direction(direction) {}
};

typedef actionlib::SimpleActionClient<move_base_msgs::MoveBaseAction> MoveBaseClient;

int main(int argc, char** argv){
  ros::init(argc, argv, "simple_navigation_goals");

  //tell the action client that we want to spin a thread by default
  MoveBaseClient ac("move_base", true);

  float MsgBuffer[100];
  int clientSocket = startServer();
  int bytes_received;
  ROS_INFO("clientSocket inside main: %i", clientSocket);
  ROS_INFO("Waiting for goal info from client...");
  while((bytes_received = recv(clientSocket, MsgBuffer, sizeof(MsgBuffer), 0)) > 0){
      move_base_msgs::MoveBaseGoal goal;
      dataPos dPos = dataPos(MsgBuffer[0], MsgBuffer[1], MsgBuffer[2], MsgBuffer[3], (int) MsgBuffer[4]);
      
      goal.target_pose.header.frame_id = "base_link";
      goal.target_pose.header.stamp = ros::Time::now();
      goal.target_pose.pose.position.x = 0;
      goal.target_pose.pose.position.y = 0;
      goal.target_pose.pose.position.z = 0;
      if(dPos.distance < 1.75 && dPos.distance > 0.0){
        //robot is close, only fix orientation
        double angle = 0.0;
          if(dPos.direction == 1){
            angle = (1.0/(dPos.distance * dPos.distance)) * -0.85;
          }
          else if(dPos.direction == 0){
            angle = (1.0/(dPos.distance * dPos.distance)) * 0.85;
          }
          Eigen::MatrixXd zAxis(3,1);
          zAxis.setZero();
          zAxis(2,0) = 1;
          Eigen::Quaterniond q(Eigen::AngleAxisd(angle, zAxis));
          goal.target_pose.pose.orientation.x = q.x();
          goal.target_pose.pose.orientation.y = q.y();
          goal.target_pose.pose.orientation.z = q.z();
          goal.target_pose.pose.orientation.w = q.w();
      }
      else{
        goal.target_pose.pose.position.x = dPos.tz - 1.25;
        goal.target_pose.pose.position.y = -1.0 * dPos.tx;
        goal.target_pose.pose.position.z = 0;
        goal.target_pose.pose.orientation.w = 1.0;
      }

      ROS_INFO("Received goal position: [%f,%f,%f] Distance:%f, Direction: %i", goal.target_pose.pose.position.x
        , goal.target_pose.pose.position.y, goal.target_pose.pose.position.z, dPos.distance, dPos.direction);
      ROS_INFO("Sending goal");
      ac.sendGoal(goal);
  }
  ROS_INFO("Closing socket");
  ac.cancelGoal();
  close(clientSocket);

  ROS_INFO("Exited while loop.");
  if(ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED)
    ROS_INFO("Hooray, all goals accmoplished");
  else ROS_INFO("The base failed to accomplish latest goal");
  
  return 0;
}


