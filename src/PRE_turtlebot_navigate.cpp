#include "ros/ros.h"
#include "geometry_msgs/Twist.h"
#include "sensor_msgs/LaserScan.h"
#include <tf/transform_listener.h>
#include <cmath>

// Global variables
ros::Publisher cmd_vel_pub;
ros::Subscriber laser_sub;
double goal_x, goal_y;
double angle, x, y;
double front_scan, right_scan, left_scan, back_scan;
bool facing_goal, orientation_fixed, wall_ahead = false;
bool goal_reached, clear_goal_path = false;

// Function prototypes
bool orientToGoal(double tolerance_angle, tf::StampedTransform transform);
void publishCmdVelocity(double linear_x, double angular_z);
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg);
double measureDistance(double x1, double y1, double x2, double y2);
void checkGoalReached();

/// \brief Enum to represent the state of the robot
enum State {
   IDLE,
   DRIVING_TO_GOAL,
   FOLLOWING_PERIMETER,
   GOAL_REACHED
};

// State variables
State current_state = IDLE;

// Main function
int main(int argc, char **argv)
{
   ros::init(argc, argv, "turtlebot_navigate");
   ros::NodeHandle n;
   ros::Rate loop_rate(10);

   // Publishers
   cmd_vel_pub = n.advertise<geometry_msgs::Twist>("cmd_vel", 1000);

   // Subscribers
   laser_sub = n.subscribe("scan", 1000, laserCallback);

   // Parameters
   n.getParam("/goal_x", goal_x);
   n.getParam("/goal_y", goal_y);

   // Set the initial state
   current_state = DRIVING_TO_GOAL;

   // Log the goal and initial position
   ROS_INFO("Goal: (%f, %f)", goal_x, goal_y);
   ROS_INFO("Initial position: (%f, %f)", x, y);
   ROS_INFO("Navigation started.");


   // Main timer loop
   while (ros::ok()) 
   {
      // Get the current position and orientation of the robot each loop
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
      angle = tf::getYaw(q);
      x = transform.getOrigin().x();
      y = transform.getOrigin().y();
      
      // State Machine
      switch (current_state) {

         // Idle state, do nothing
         case IDLE:
            break;

         // Drive to goal
         case DRIVING_TO_GOAL:
            checkGoalReached();
            facing_goal = orientToGoal(0.35, transform);
            if (facing_goal) {
               if (measureDistance(x, y, goal_x, goal_y) > 1.0) {
                  publishCmdVelocity(0.20, 0.0);
               } else {
                  publishCmdVelocity(0.05, 0.0);
               }
            }
            break;

         // Maintain left scan distance from the wall and navigate around obstacles
         case FOLLOWING_PERIMETER:

            // If wall in front, and orientation not fixed, fit it
            if (front_scan < 0.5 && orientation_fixed == false) {
               publishCmdVelocity(0.0, -0.20);
               break;
            }
            else {
               orientation_fixed = true;
            }

            // If wall is detected on the left...
            if (left_scan < 0.8) {
               ROS_INFO("Following the left wall...");

               // Maintain left scan distance from the wall as robot moves along the wall
               if (orientation_fixed && front_scan > 0.5 && wall_ahead == false) {
                  ROS_INFO("Maintaining distance...");
                  if (left_scan < 0.35) {
                     publishCmdVelocity(0.13, -0.09);
                  }
                  else if (left_scan >= 0.35) {
                     publishCmdVelocity(0.13, 0.09);
                  }
                  else {
                     publishCmdVelocity(0.13, 0.0);
                  }
               }

               // If obstacle detected ahead while moving along the wall, stop
               if (orientation_fixed && front_scan < 0.5 && wall_ahead == false) {
                  publishCmdVelocity(0.0, 0.0);
                  wall_ahead = true;
               }

               // If obstacle detected ahead while moving along the wall, stop and turn
               if (wall_ahead && front_scan < 2.0) {
                  publishCmdVelocity(0.0, -0.20);
               }
               // If reoriented, mark wall as not ahead anymore
               else if (wall_ahead && front_scan > 2.0) {
                  wall_ahead = false;
                  orientation_fixed = false;
                  publishCmdVelocity(0.0, 0.0);
               }
            }
            else {
               // If lost the wall, turn left until left scan detects the wall
               ROS_INFO("Lost the wall, turning left...");
               orientation_fixed = false;
               wall_ahead = false;
               publishCmdVelocity(0.10, 0.75);
            }
            break;

         // Goal reached
         case GOAL_REACHED:
            publishCmdVelocity(0.0, 0.0);
            if (!goal_reached) {
            ROS_INFO("Goal reached.");
            goal_reached = true;
            }
            break;
               }

   ros::spinOnce();
   loop_rate.sleep();
   }
   return 0;
}

/// \brief Callback function for the laser scan data
/// \param scan_msg The message containing the laser scan data
/// \return void
/// \details This function is called whenever a new laser scan message is received. It checks if the path to the goal is clear
///          and sets the current state to DRIVING_TO_GOAL if the path is clear.
void laserCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {

   // Find the scan to look at for checking if the path is clear
   double abs_angle_to_goal = atan2(goal_y - y, goal_x - x) * 180 / M_PI;
   double robot_angle = angle * (180/M_PI);
   double angle_diff_1 = abs_angle_to_goal - robot_angle;
   int rounded_angle_diff_1 = static_cast<int>(std::round(angle_diff_1));
   // Sometimes the rounded angle difference is negative, so add 360 to it
   if (rounded_angle_diff_1 < 0) {
      rounded_angle_diff_1 += 360;
   }

   // Check if the path to the goal is clear
   bool is_path_clear = true;
   for (int i = rounded_angle_diff_1 - 6; i <= rounded_angle_diff_1 + 6; i++) {
      int index = (i + 360) % 360; // Ensure the index is within [0, 359]
      if (scan_msg->ranges[index] <= 3.0) {
         is_path_clear = false;
         break;
      }
   }

   // If the path to the goal is clear, set the current state to DRIVING_TO_GOAL
   if (is_path_clear) {
      if (current_state != GOAL_REACHED) {
         ROS_INFO("Path to goal appears clear.");
         current_state = DRIVING_TO_GOAL;
      }
   }

   // Get the distance to the wall in front of the robot
   front_scan = scan_msg->ranges[0];
   right_scan = scan_msg->ranges[270];
   left_scan = scan_msg->ranges[90];
   back_scan = scan_msg->ranges[180];
   
   // If the robot is not following the perimeter and a wall is detected in front of the robot, set the current state to FOLLOWING_PERIMETER
   if (front_scan < 0.5 && current_state != FOLLOWING_PERIMETER) {
      ROS_INFO("Perimeter detected in front");
      publishCmdVelocity(0.0, 0.0);
      orientation_fixed = false;
      current_state = FOLLOWING_PERIMETER;
   }
}

/// \brief Function to orient the robot towards the goal
/// \param tolerance_angle The tolerance angle for the orientation
/// \param transform The transform containing the current position and orientation of the robot
/// \return bool True if the robot is oriented towards the goal, false otherwise
/// \details This function orients the robot towards the goal. It calculates the angle to the goal and the angle difference
///          between the robot's orientation and the angle to the goal. If the angle difference is greater than the tolerance
///          angle, the robot rotates towards the goal. The rotation speed is proportional to the angle difference. If the
///          distance to the goal is less than 1.0, the rotation speed is reduced. The function returns true if the robot is
///          oriented towards the goal, and false otherwise.
bool orientToGoal(double tolerance_angle, tf::StampedTransform transform) {

   double angle_to_goal = atan2(goal_y - transform.getOrigin().y(), goal_x - transform.getOrigin().x());
   double angle_diff = angle_to_goal - transform.getRotation().getAngle();
   angle_diff = atan2(sin(angle_diff), cos(angle_diff));

   // Rotation speed is proportional to angle_diff
   if (fabs(angle_diff) > tolerance_angle) {
      double rotation_speed = 0.25 * fabs(angle_diff);
      if (measureDistance(x, y, goal_x, goal_y) < 1.0) {
         rotation_speed = rotation_speed * 0.25;
      }
      if (angle_diff > 0) {
         publishCmdVelocity(0.0, rotation_speed);
      } else {
         publishCmdVelocity(0.0, -rotation_speed);
      }
      return false;
   }
   else {
      return true;
   }
}

/// \brief Function to publish the command velocity message
/// \param linear_x The linear velocity in the x-direction
/// \param angular_z The angular velocity in the z-direction
/// \details This function publishes the command velocity message on the /cmd_vel topic. The linear and angular velocities
///          are set in the message and published.
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

/// \brief Function to measure the distance between two points
/// \param x1 The x-coordinate of the first point
/// \param y1 The y-coordinate of the first point
/// \param x2 The x-coordinate of the second point
/// \param y2 The y-coordinate of the second point
/// \return double The distance between the two points
double measureDistance(double x1, double y1, double x2, double y2) {
    double dx = x2 - x1;
    double dy = y2 - y1;
    return std::sqrt(dx * dx + dy * dy);
}

/// \brief Function to check if the goal has been reached
/// \details This function checks if the goal has been reached. It calculates the distance to the goal and if the distance
///          is less than 0.3, the current state is set to GOAL_REACHED.
void checkGoalReached() {
   double distance_to_goal = measureDistance(x, y, goal_x, goal_y);
   if (distance_to_goal < 0.2) {
      current_state = GOAL_REACHED;
   }
}
