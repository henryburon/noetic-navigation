#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <cmath>

// Global variables
ros::Publisher cmd_vel_pub;
ros::Subscriber laser_sub;
double goal_x, goal_y, current_angle, current_x, current_y;
double front_scan, right_scan, left_scan;
double orient_tolerance = 10.0, goal_tolerance = 0.3;
bool facing_goal = false;


// Function declarations
void publishCmdVelocity(double linear_x, double angular_z);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
bool orientToGoal(double goal_x, double goal_y, double current_x, double current_y, double current_angle, double orient_tolerance);

enum State {
   IDLE,
   DRIVING_TO_GOAL,
   FOLLOWING_PERIMETER,
   GOAL_REACHED
};

State current_state = IDLE;

int main(int argc, char **argv) {
   ros::init(argc, argv, "turtlebot_navigate");
   ros::NodeHandle n;
   ros:: Rate loop_rate(10);

   // Publishers
   cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

   // Subscribers
   laser_sub = n.subscribe("scan", 1000, laserCallback);

   // Parameters
   n.getParam("/goal_x", goal_x);
   n.getParam("/goal_y", goal_y);

   // Log initialization values
   ROS_INFO("Goal: (%f, %f)", goal_x, goal_y);
   ROS_INFO("Initial position: (%f, %f)", current_x, current_y);

   current_state = DRIVING_TO_GOAL;

   while (ros::ok()) {

      // Get the current position and orientation of the robot each loop
      // ###################################################
      tf::TransformListener listener;
      tf::StampedTransform transform;
      try {
         listener.waitForTransform("/odom", "/base_footprint", ros::Time(0), ros::Duration(3.0) );
         listener.lookupTransform("/odom", "/base_footprint", ros::Time(0), transform);
      } catch (tf::TransformException &ex) {
         ROS_ERROR("%s", ex.what());
         ros::Duration(1.0).sleep();
         continue;
      }
      tf::Quaternion q = transform.getRotation();
      current_angle = tf::getYaw(q);
      current_x = transform.getOrigin().x();
      current_y = transform.getOrigin().y();
      // ###################################################

      // State machine
      switch (current_state) {
         case IDLE:
            // Do nothing
            publishCmdVelocity(0.0, 0.0);
            break;

         case DRIVING_TO_GOAL:

            // Check if the goal is reached
               // If yes, change state to GOAL_REACHED
               // If no, continue
            // Check if facing the goal
               // If yes, drive forward
               // If no, rotate to face the goal
            


            
            facing_goal = orientToGoal(goal_x, goal_y, current_x, current_y, current_angle, 8.0);
            
            if (facing_goal) {
               // Drive forward
               publishCmdVelocity(0.20, 0.0);
            }

            


            break;

         case FOLLOWING_PERIMETER:
            break;

         case GOAL_REACHED:
            // Stop the robot
            publishCmdVelocity(0.0, 0.0);
            ROS_INFO("Goal reached within a tolerance of %f meters", goal_tolerance);

            break;
      }




      ros::spinOnce();
      loop_rate.sleep();
   }
   return 0;
}

void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

   // Get the distance to the wall on each side of the robot
   front_scan = scan_msg->ranges[0];
   right_scan = scan_msg->ranges[270];
   left_scan = scan_msg->ranges[90];

   // Check if the path to the goal from the current position is clear (checks 1 meter in front of the robot)
   double angle_to_goal = atan2(goal_y - current_y, goal_x - current_x) * 180 / M_PI;
   double robot_angle = current_angle * 180 / M_PI;
   double angle_diff = angle_to_goal - robot_angle;
   int rounded_angle_diff = static_cast<int>(std::round(angle_diff));
   if (rounded_angle_diff < 0) {
      rounded_angle_diff += 360;
   }

   bool clear_path = true;
   for (int i = rounded_angle_diff - 6; i <= rounded_angle_diff + 6; i++) {
      int index = (i + 360) % 360; // Ensure the index is within [0, 359]
      if (scan_msg->ranges[index] <= 1.0) { // Checks 1 meter in front of the robot
         clear_path = false;
         break;
      }
   }

   // If 1 meter in the direction of the goal is clear, DRIVE TO GOAL
   if (clear_path) {
      current_state = DRIVING_TO_GOAL;
   }




   }

   // Check if the path to the goal is clear
      // If yes, change state to DRIVING_TO_GOAL



   // Check if the path to the goal is clear
      // If yes, change state to DRIVING_TO_GOAL
      // If no, change state to FOLLOWING_PERIMETER

   

bool orientToGoal(double goal_x, double goal_y, double current_x, double current_y, double current_angle, double orient_tolerance) {
   
   // Calculate the angle the robot needs to rotate to face the goal
   double angle_to_goal = atan2(goal_y - current_y, goal_x - current_x) * 180 / M_PI;
   double robot_angle = current_angle * 180 / M_PI;
   double angle_diff = angle_to_goal - robot_angle;

   // Normalize the angle difference to be within [-180, 180]
   if (angle_diff > 180) {
      angle_diff -= 360;
   } else if (angle_diff < -180) {
      angle_diff += 360;
   }

   // log angle difference
   ROS_INFO("Angle difference: %f", angle_diff);

   // Publish the appropriate cmd_vel based on the angle difference
   if (angle_diff > orient_tolerance) {
      // Rotate CCW
      publishCmdVelocity(0.0, 0.20);
      return false;
   }
   else if (angle_diff < -orient_tolerance) {
      // Rotate CW
      publishCmdVelocity(0.0, -0.20);
      return false;
   }
   else {
      // Stop rotating
      // publishCmdVelocity(0.0, 0.0);
      return true;
   }
}

void publishCmdVelocity(double linear_x, double angular_z) {
      geometry_msgs::Twist msg;

      // Set linear velocity
      msg.linear.x = linear_x;
      msg.linear.y = 0.0;
      msg.linear.z = 0.0;

      // Set angular velocity
      msg.angular.x = 0.0;
      msg.angular.y = 0.0;
      msg.angular.z = angular_z;

      // Publish the message on /cmd_vel topic
      cmd_vel_pub.publish(msg);
}