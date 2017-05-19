#ifndef BEBOP_CODEBLOCK_ABSTRACTION_H
#define BEBOP_CODEBLOCK_ABSTRACTION_H


#include <string>
#include <numeric>
#include <boost/shared_ptr.hpp>
#include <boost/bind.hpp>
#include <boost/thread/mutex.hpp>
#include <boost/thread/locks.hpp>
#include <boost/function.hpp>

#include <ros/ros.h>
#include <std_msgs/Empty.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/Image.h>
#include <angles/angles.h>
#include <sensor_msgs/CameraInfo.h>
#include <nav_msgs/Odometry.h>
#include <sensor_msgs/NavSatFix.h>
#include <tf2_ros/transform_listener.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>

#include <gtest/gtest.h>

#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAttitudeChanged.h>
#include <bebop_msgs/Ardrone3CameraStateOrientation.h>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateSpeedChanged.h>
#include <bebop_msgs/Ardrone3PilotingStateAltitudeChanged.h>
#include <bebop_msgs/CommonCommonStateBatteryStateChanged.h>

#define TIMED_ASSERT(TIMEOUT, WAIT_UNTIL_TRUE, WAITING_TEXT)                      \
do                                                                                \
{                                                                                 \
  const ros::Time start = ros::Time::now();                                       \
  while (((ros::Time::now() - start).toSec() < TIMEOUT) && (!(WAIT_UNTIL_TRUE)))  \
  {                                                                               \
    ROS_INFO_ONCE(WAITING_TEXT);                                                  \
    ros::Rate(5.0).sleep();                                                       \
  }                                                                               \
  ASSERT_TRUE(WAIT_UNTIL_TRUE);                                                   \
}                                                                                \
while (0)                                                                         \

namespace bebop_driver
{
namespace util
{

template<typename T>
class ASyncSub
{
private:
  typedef boost::function<void (const boost::shared_ptr<T const>& data)> callback_t;

  ros::NodeHandle nh;
  bool active_;
  ros::Time last_updated_;
  std::string topic_;
  std::size_t queue_size_;
  callback_t user_callback_;
  ros::Subscriber sub_;
  boost::shared_ptr<T const> msg_cptr_;
  mutable boost::mutex mutex_;

  void cb(const boost::shared_ptr<T const> &msg_cptr)
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = true;
    last_updated_ = ros::Time::now();
    msg_cptr_ = msg_cptr;
    if (user_callback_) user_callback_(msg_cptr_);
  }

public:
  ASyncSub(ros::NodeHandle& nh,
           const std::string& topic,
           const std::size_t queue_size,
           callback_t user_callback = 0)
    : nh(nh), active_(false), last_updated_(0), topic_(topic), user_callback_(user_callback)
  {
    sub_ = nh.subscribe<T>(topic_, queue_size, boost::bind(&ASyncSub<T>::cb, this, _1));
  }

  T GetMsgCopy() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return *msg_cptr_;
  }

  const boost::shared_ptr<T const>& GetMsgConstPtr() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return msg_cptr_;
  }

  // T operator ()() const {return GetMsgCopy();}

  const boost::shared_ptr<T const>& operator()() const
  {
    // no need to acquire the lock here, since the underlying function does that
    return GetMsgConstPtr();
  }

  void Deactivate()
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    active_ = false;
  }

  void DeactivateIfOlderThan(const double seconds)
  {
    if (!IsActive()) return;
    if (GetFreshness().toSec() > seconds)
    {
      ROS_WARN("Information on topic (%s) is older than (%4.2lf) seconds. Deactivating.", topic_.c_str(), seconds);
      Deactivate();
    }
  }

  bool IsActive() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return active_;
  }

  const ros::Time GetLastUpdated() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return last_updated_;
  }

  const ros::Duration GetFreshness() const
  {
    boost::lock_guard<boost::mutex> lock(mutex_);
    return ros::Time::now() - last_updated_;
  }
};

}  // namespace util

namespace test
{



void dummy_cb(const bebop_msgs::Ardrone3PilotingStateAttitudeChanged::ConstPtr& msg_ptr)
{
  ROS_INFO("Roll: %0.2lf, Pitch: %0.2lf, Yaw: %0.2lf",
           msg_ptr->roll, msg_ptr->pitch, msg_ptr->yaw);
}

class BebopInTheLoopTest: public ::testing::Test
{
protected:
  ros::NodeHandle nh_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_ptr_;

  ros::Publisher land_pub_;
  ros::Publisher cmdvel_pub_;

  std_msgs::Empty em;
  geometry_msgs::Twist tw;

  boost::shared_ptr<util::ASyncSub<sensor_msgs::Image> > image_;

//  boost::shared_ptr<util::ASyncSub<bebop_msgs::Ardrone3PilotingStateAttitudeChanged> > attitude_state_;

  virtual void SetUp()
  {
    ros::NodeHandle priv_nh("~");

    ROS_INFO("In SetUp()");

    // Common Publishers
    land_pub_= nh_.advertise<std_msgs::Empty>("land", 1);
    cmdvel_pub_ = nh_.advertise<geometry_msgs::Twist>("cmd_vel", 1);

    // Common Subs
    image_.reset(new util::ASyncSub<sensor_msgs::Image>(
                   nh_, "image_raw", 10));

    spinner_ptr_.reset(new ros::AsyncSpinner(4));
    spinner_ptr_->start();

    // Wait 10s for Bebop to come up
    ros::Time start = ros::Time::now();

    // Check image_ (when image_ is being published, everything else is ready)
    TIMED_ASSERT(15.0, image_->IsActive(), "Waiting for Bebop ...");
    TIMED_ASSERT(5.0, land_pub_.getNumSubscribers() > 0, "Waiting for land subscription ...");

    ROS_INFO("End SetUp()");
  }

  void StopBebop()
  {
    tw.linear.x = 0.0;
    tw.linear.y = 0.0;
    tw.linear.z = 0.0;
    tw.angular.z = 0.0;
    ROS_WARN("Stopping ...");
    cmdvel_pub_.publish(tw);
    ros::Duration(1.0).sleep();
  }

  virtual void TearDown()
  {
    ROS_INFO("In teardown()");
    StopBebop();
    std_msgs::Empty em;
    land_pub_.publish(em);
    ros::Rate(ros::Duration(2.0)).sleep();
    spinner_ptr_->stop();
  }

};

/*
 * Parrot's coordinate system for velocities:
 * +x: forward
 * +y: right
 * +z: downward
 *
 * (not ROS compatible)
 *
 * odom on the other hand is REP-103 compatible
 * */

}  // namespace test
}  // namespace bebop_driver




#endif // BEBOP_CODEBLOCK_ABSTRACTION_H
