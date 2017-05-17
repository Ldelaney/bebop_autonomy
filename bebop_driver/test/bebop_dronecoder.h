#ifndef BEBOP_DRONECODER_H
#define BEBOP_DRONECODER_H

#include "bebop_codeblock_abstraction.h"
using namespace bebop_driver::test;

TEST_F(BebopInTheLoopTest, Piloting)
{
 //SETUP

  double alt_start, yaw_start, alt_start_odom, yaw_start_odom;
  boost::shared_ptr<bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateFlyingStateChanged> >
      flying_state(new bebop_driver:: util::ASyncSub<bebop_msgs::Ardrone3PilotingStateFlyingStateChanged>(
                      nh_, "states/ardrone3/PilotingState/FlyingStateChanged", 10));

  boost::shared_ptr<bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateSpeedChanged> >
      speed_state(new bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateSpeedChanged>(
                      nh_, "states/ardrone3/PilotingState/SpeedChanged", 10));

  boost::shared_ptr<bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateAltitudeChanged> >
      alt_state(new bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateAltitudeChanged>(
                      nh_, "states/ardrone3/PilotingState/AltitudeChanged", 10));

  boost::shared_ptr<bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> >
      att_state(new bebop_driver::util::ASyncSub<bebop_msgs::Ardrone3PilotingStateAttitudeChanged>(
                      nh_, "states/ardrone3/PilotingState/AttitudeChanged", 10));

  // The problem with the battery state is that it is only updated when its values changes,
  // this is more likely to happen when flying than sitting on the ground
  boost::shared_ptr<bebop_driver::util::ASyncSub<bebop_msgs::CommonCommonStateBatteryStateChanged> >
      bat_state(new bebop_driver::util::ASyncSub<bebop_msgs::CommonCommonStateBatteryStateChanged>(
                      nh_, "states/common/CommonState/BatteryStateChanged", 10));

  boost::shared_ptr<bebop_driver::util::ASyncSub<nav_msgs::Odometry> >
      odom(new bebop_driver::util::ASyncSub<nav_msgs::Odometry>(
             nh_, "odom", 10));

  boost::shared_ptr<bebop_driver::util::ASyncSub<sensor_msgs::NavSatFix> >
      gps(new bebop_driver::util::ASyncSub<sensor_msgs::NavSatFix>(
            nh_, "fix", 10));



  ros::Publisher takeoff_pub =  nh_.advertise<std_msgs::Empty>("takeoff", 1);
  ros::Publisher reset_pub = nh_.advertise<std_msgs::Empty>("reset", 1);

  // Wait 5s time for connections to establish
  TIMED_ASSERT(5.0, takeoff_pub.getNumSubscribers() > 0, "Waiting for takeoff subscription ...");
  TIMED_ASSERT(5.0, cmdvel_pub_.getNumSubscribers() > 0, "Waiting for cmd_vel subscription ...");
  TIMED_ASSERT(5.0, reset_pub.getNumSubscribers() > 0, "Waiting for reset subscription ...");
//END OF SETUP

  ROS_WARN("Taking off ...");
  takeoff_pub.publish(em);
  TIMED_ASSERT(10.0,
               flying_state->IsActive() && flying_state->GetMsgCopy().state ==
                 bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_hovering,
               "Waiting for takeoff to finish ..."
               );
   //ros::Duration(3.0).sleep();

   /*StopBebop();
   //movegod (0, -10, 0)
   tw.linear.x = 0.0; //plus is forward,
   tw.linear.y = 0.25; //negative is right, positive is left
   tw.linear.z = 0.000;
   tw.angular.z = 0.000;
   ROS_WARN("MoveGod(0, -10, 0) ...");
   cmdvel_pub_.publish(tw);
   ros::Duration(2.5).sleep();
*/

   StopBebop();
   ASSERT_LT(att_state->GetFreshness().toSec(), 0.5);
   ASSERT_LT(odom->GetFreshness().toSec(), 0.5);
   yaw_start = att_state->GetMsgCopy().yaw;
   tw.linear.x = 0.1;
   tw.linear.y = -0.05; //slight pull into the circle
   tw.linear.z = 0.0;
   tw.angular.z = -0.5;
   ROS_WARN("Rotating CCW for 90 degrees ...");
   cmdvel_pub_.publish(tw);

   TIMED_ASSERT(10.0,
                angles::normalize_angle(att_state->GetMsgCopy().yaw - yaw_start) >= 0.5 * 3.141596,
                "Measuring Yaw");
   StopBebop();

/*
   //movegod (10, 0, 0)
   tw.linear.x = 0.25; //plus is forward,
   tw.linear.y = 0.00; //negative is right, positive is left
   tw.linear.z = 0.000;
   tw.angular.z = 0.000;
   ROS_WARN("MoveGod(10, 0, 0) ...");
   cmdvel_pub_.publish(tw);
   ros::Duration(2.5).sleep();
*/
  StopBebop();

  //TIMED_ASSERT(5.0, speed_state->IsActive(), "Waiting for Speed measurements ...");
  //TIMED_ASSERT(5.0, alt_state->IsActive(), "Waiting for alt measurements ...");
  //TIMED_ASSERT(5.0, att_state->IsActive(), "Waiting for attitude measurements ...");
  //TIMED_ASSERT(5.0, odom->IsActive(), "Waiting for odom ...");
  //TIMED_ASSERT(5.0, gps->IsActive(), "Waiting for GPS ...");

  // TODO(mani-monaj): Use proper values for pitch/roll test thresholds based on cmd_vel
/*
  tw.linear.x = 0.4;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving forwared ...");
  cmdvel_pub_.publish(tw);

  TIMED_ASSERT(3.0, angles::to_degrees(att_state->GetMsgCopy().pitch) < -2.5 , "Measuring pitch ...");
  TIMED_ASSERT(3.0, (odom->GetMsgCopy().twist.twist.linear.x > 0.2),
               "Measuring the forward and lateral velocities from odom ...");

  StopBebop();
  TIMED_ASSERT(5.0,
               (fabs(speed_state->GetMsgCopy().speedX) < 0.05) &&
               (fabs(speed_state->GetMsgCopy().speedY) < 0.05) &&
               (fabs(speed_state->GetMsgCopy().speedZ) < 0.05)
               , "Measuring the speed vector ...");

  TIMED_ASSERT(5.0,
               (fabs(odom->GetMsgCopy().twist.twist.linear.x) < 0.05) &&
               (fabs(odom->GetMsgCopy().twist.twist.linear.y) < 0.05) &&
               (fabs(odom->GetMsgCopy().twist.twist.linear.z) < 0.05)
               , "Measuring the speed vector from odom...");


  tw.linear.x = -0.4;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving Backward ...");
  cmdvel_pub_.publish(tw);

  TIMED_ASSERT(3.0, angles::to_degrees(att_state->GetMsgCopy().pitch) > 2.5, "Measuring pitch ...");
  TIMED_ASSERT(3.0, (odom->GetMsgCopy().twist.twist.linear.x < -0.2),
               "Measuring the forward and lateral velocities from odom ...");
  StopBebop();

  tw.linear.x = 0.0;
  tw.linear.y = 0.4;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving left ...");
  cmdvel_pub_.publish(tw);
  TIMED_ASSERT(3.0, angles::to_degrees(att_state->GetMsgCopy().roll) < -2.5, "Measuring roll ...");
  TIMED_ASSERT(3.0, (odom->GetMsgCopy().twist.twist.linear.y > 0.1),
               "Measuring the forward and lateral velocities from odom ...");

  StopBebop();

  tw.linear.x = 0.0;
  tw.linear.y = -0.4;
  tw.linear.z = 0.0;
  tw.angular.z = 0.0;
  ROS_WARN("Moving right ...");
  cmdvel_pub_.publish(tw);
  TIMED_ASSERT(3.0, angles::to_degrees(att_state->GetMsgCopy().roll) > 2.5, "Measuring roll ...");
  TIMED_ASSERT(3.0, (odom->GetMsgCopy().twist.twist.linear.y < -0.1),
               "Measuring the forward and lateral velocities from odom ...");
  StopBebop();

  // Make sure altitude is fresh
  ASSERT_LT(alt_state->GetFreshness().toSec(), 0.5);
  ASSERT_LT(odom->GetFreshness().toSec(), 0.5);

  alt_start = alt_state->GetMsgCopy().altitude;
  alt_start_odom = odom->GetMsgCopy().pose.pose.position.z;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.2;
  tw.angular.z = 0.0;
  ROS_WARN("Ascending for 0.5m ...");
  cmdvel_pub_.publish(tw);
  TIMED_ASSERT(10.0,
               ((alt_state->GetMsgCopy().altitude - alt_start) >= 0.5) &&
               ((odom->GetMsgCopy().pose.pose.position.z - alt_start_odom) >= 0.5) &&
               (speed_state->GetMsgCopy().speedZ < -0.05) &&
               (odom->GetMsgCopy().twist.twist.linear.z > 0.05),
               "Measuring altitude, speed.z and vertical velocity from odom ...");

  StopBebop();

  // Make sure altitude is fresh
  ASSERT_LT(alt_state->GetFreshness().toSec(), 0.5);
  ASSERT_LT(odom->GetFreshness().toSec(), 0.5);
  alt_start = alt_state->GetMsgCopy().altitude;
  alt_start_odom = odom->GetMsgCopy().pose.pose.position.z;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = -0.2;
  tw.angular.z = 0.0;
  ROS_WARN("Descending for 0.5m ...");
  cmdvel_pub_.publish(tw);
  TIMED_ASSERT(10.0,
               ((alt_state->GetMsgCopy().altitude - alt_start) <= -0.5) &&
               ((odom->GetMsgCopy().pose.pose.position.z - alt_start_odom) <= -0.5) &&
               (speed_state->GetMsgCopy().speedZ > 0.05) &&
               (odom->GetMsgCopy().twist.twist.linear.z < -0.05),
               "Measuring altitude, speed.z and vertical velocity from odom ...");

  StopBebop();

  // Make sure the attitude is fresh
  ASSERT_LT(att_state->GetFreshness().toSec(), 0.5);
  ASSERT_LT(odom->GetFreshness().toSec(), 0.5);
  yaw_start = att_state->GetMsgCopy().yaw;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = -0.5;
  ROS_WARN("Rotating CW for 90 degrees ...");
  cmdvel_pub_.publish(tw);

  // TODO(mani-monaj): Add yaw
  TIMED_ASSERT(10.0,
               angles::normalize_angle(att_state->GetMsgCopy().yaw - yaw_start) >= 0.5 * 3.141596,
               "Measuring Yaw");

  StopBebop();

  // Make sure the attitude is fresh
  ASSERT_LT(att_state->GetFreshness().toSec(), 0.5);
  yaw_start = att_state->GetMsgCopy().yaw;
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = 0.0;
  tw.angular.z = 0.5;
  ROS_WARN("Rotating CCW for 90 degrees ...");
  cmdvel_pub_.publish(tw);
  TIMED_ASSERT(15.0,
               angles::normalize_angle(att_state->GetMsgCopy().yaw - yaw_start) <= -0.8 * 3.141596,
               "Measuring Yaw");

  StopBebop();
*/
  /* By this time, battery state must have been changed (even on Bebop 2) */
  //TIMED_ASSERT(20.0, bat_state->IsActive(), "Measuring battery ...");
  //const uint8_t bat_percent = bat_state->GetMsgCopy().percent;

  //BEGIN LANDING
  StopBebop();
  tw.linear.x = 0.0;
  tw.linear.y = 0.0;
  tw.linear.z = -0.05;
  tw.angular.z = 0.0;
  ROS_WARN("Stop ...");
  cmdvel_pub_.publish(tw);

  ROS_WARN("Landing ...");
  land_pub_.publish(em);
  TIMED_ASSERT(20.0,
               flying_state->IsActive() && flying_state->GetMsgCopy().state ==
                 bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_landed,
               "Waiting for land to finish..."
               );
  //END OF LANDING



  // emergency state is transient (unlike ardrone), we may miss the signal
//  ROS_WARN("Emergency ...");
//  reset_pub.publish(em);

//  TIMED_ASSERT(5.0,
//               flying_state->IsActive() && flying_state->GetMsgCopy().state ==
//                 bebop_msgs::Ardrone3PilotingStateFlyingStateChanged::state_emergency,
//               "Waiting for reset to happen..."
//               );

  //ASSERT_GE(bat_percent - bat_state->GetMsgCopy().percent, 0);
}


#endif // BEBOP_DRONECODER_H
