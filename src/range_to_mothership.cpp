#include "ros/ros.h"
#include "mdt_msgs/Gps.h"
#include "std_msgs/Float32.h"
#include "project11/utils.h"
#include "project11_msgs/RangeBearing.h"

ros::Publisher bearing_pub;
ros::Publisher range_pub;
ros::Publisher range_bearing_pub;

ros::Time last_pub_time;

mdt_msgs::Gps last_mothership_fix;

void mothershipGPSCallback(const mdt_msgs::Gps::ConstPtr& msg)
{
  last_mothership_fix = *msg;
  //std::cerr << "M: " << msg->latitude << ", " << msg->longitude << std::endl;
}

void drixGPSCallback(const mdt_msgs::Gps::ConstPtr& msg)
{
  //std::cerr << "D: " << msg->latitude << ", " << msg->longitude << std::endl;

  if(msg->header.stamp - last_mothership_fix.header.stamp < ros::Duration(5.0) && msg->header.stamp - last_pub_time >= ros::Duration(1.0))
  {
    project11::LatLongDegrees drix(msg->latitude, msg->longitude);
    project11::LatLongDegrees mothership(last_mothership_fix.latitude, last_mothership_fix.longitude);
    auto azimuth_distance = project11::WGS84::inverse(drix,mothership);
    //std::cerr << "a: " << project11::AngleDegrees(azimuth_distance.first) << " d: " << azimuth_distance.second << std::endl;

    std_msgs::Float32 bearing;
    bearing.data = project11::AngleDegrees(azimuth_distance.first).value();
    bearing_pub.publish(bearing);

    std_msgs::Float32 range;
    range.data = azimuth_distance.second;
    range_pub.publish(range);

    project11_msgs::RangeBearing range_bearing;
    range_bearing.stamp = msg->header.stamp;
    range_bearing.range = range.data;
    range_bearing.bearing = bearing.data;

    range_bearing_pub.publish(range_bearing);

    last_pub_time = msg->header.stamp;
  }

}


int main(int argc, char **argv)
{
  ros::init(argc, argv, "range_to_mothership");
  ros::NodeHandle nh;

  ros::Subscriber drix_sub = nh.subscribe("/gps", 5, &drixGPSCallback);
  ros::Subscriber mothership_sub = nh.subscribe("/mothership_gps", 5, &mothershipGPSCallback);

  bearing_pub = nh.advertise<std_msgs::Float32>("to_mothership/bearing", 5);
  range_pub = nh.advertise<std_msgs::Float32>("to_mothership/range", 5);
  range_bearing_pub = nh.advertise<project11_msgs::RangeBearing>("to_mothership/range_bearing", 5);

  ros::spin();
}
