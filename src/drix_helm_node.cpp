#include "ros/ros.h"
#include "mdt_msgs/Gps.h"
#include "mdt_msgs/GeoPath.h"
#include "drix_msgs/DrixOutput.h"
#include "mdt_msgs/StampedString.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "project11_msgs/Heartbeat.h"
#include "project11_msgs/Helm.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include "nmea_msgs/Sentence.h"
#include <vector>
#include "project11/utils.h"

namespace p11 = project11;

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;
ros::Publisher heartbeat_pub;
ros::Publisher backseat_path_pub;
ros::Publisher display_pub;
ros::Publisher ais_pub;

mdt_msgs::Gps last_gps;

double rudder;
double throttle;
ros::Time last_helm_time;

bool standby;

struct LatLong
{
    double latitude;
    double longitude;
};

std::vector<LatLong> current_path;
LatLong current_position;

double current_speed;

std::string frame_id;

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void helmCallback(const project11_msgs::Helm::ConstPtr& msg)
{
    throttle = msg->throttle;
    rudder = msg->rudder;
    
    last_helm_time = msg->header.stamp;
}

void standbyCallback(const std_msgs::Bool::ConstPtr& msg)
{
  standby = msg->data;
}

void vehicleSatusCallback(const drix_msgs::DrixOutput::ConstPtr& inmsg)
{
  project11_msgs::Heartbeat hb;
  hb.header.stamp = ros::Time::now();

  project11_msgs::KeyValue kv;

  kv.key = "RPM";
  std::stringstream rpm_str;
  rpm_str << inmsg->thruster_RPM;
  kv.value = rpm_str.str();
  hb.values.push_back(kv);
    
  kv.key = "rudder_angle";
  std::stringstream rudder_str;
  rudder_str << inmsg->rudderAngle_deg;
  kv.value = rudder_str.str();
  hb.values.push_back(kv);

  kv.key = "fuel_level";
  std::stringstream fuel_level_str;
  fuel_level_str << int(inmsg->gasolineLevel_percent);
  kv.value = fuel_level_str.str();
  hb.values.push_back(kv);

  kv.key = "drix_mode";
  switch (inmsg->drix_mode)
  {
  case drix_msgs::DrixOutput::MODE_DOCKING_:
    kv.value = "Docking";
    break;
  case drix_msgs::DrixOutput::MODE_MANUAL_:
    kv.value = "Manual";
    break;
  case drix_msgs::DrixOutput::MODE_AUTO_:
    kv.value = "Auto";
    break;
  default:
    kv.value = "Unknown";
  }
  hb.values.push_back(kv);

  kv.key = "emergency";
  if(inmsg->emergency_mode)
    kv.value = "True";
  else
    kv.value = "False";
  hb.values.push_back(kv);

  kv.key = "low_power_mode";
  if(inmsg->low_power_mode)
    kv.value = "True";
  else
    kv.value = "False";
  hb.values.push_back(kv);

  kv.key = "clutch";
  switch (inmsg->drix_clutch)
  {
  case drix_msgs::DrixOutput::CLUTCH_ERROR_:
    kv.value = "Error";
    break;
  case drix_msgs::DrixOutput::CLUTCH_BACKWARD_:
    kv.value = "Backward";
    break;
  case drix_msgs::DrixOutput::CLUTCH_NEUTRAL_:
    kv.value = "Neutral";
    break;
  case drix_msgs::DrixOutput::CLUTCH_FORWARD_:
    kv.value = "Forward";
    break;
  default:
    kv.value = "Unknown";
  }
  hb.values.push_back(kv);

  if(inmsg->error_code != 0)
  {
    kv.key = "error_code";
    std::stringstream error_code_ss;
    error_code_ss << inmsg->error_code;
    kv.value = error_code_ss.str();
    hb.values.push_back(kv);

    kv.key = "error_string";
    kv.value = inmsg->error_string;
    hb.values.push_back(kv);
  }
    
  kv.key = "keel_state";
  switch (inmsg->keel_state)
  {
  case drix_msgs::DrixOutput::KEEL_ERROR_:
    kv.value = "Error";
    break;
  case drix_msgs::DrixOutput::KEEL_UP_:
    kv.value = "Up";
    break;
  case drix_msgs::DrixOutput::KEEL_MIDDLE_:
    kv.value = "Middle";
    break;
  case drix_msgs::DrixOutput::KEEL_DOWN_:
    kv.value = "Down";
    break;
  case drix_msgs::DrixOutput::KEEL_GOING_DOWN_ERROR_:
    kv.value = "Going down error";
    break;
  case drix_msgs::DrixOutput::KEEL_GOING_UP_ERROR_:
    kv.value = "Going up error";
    break;
  case drix_msgs::DrixOutput::KEEL_UP_DOWN_ERROR_:
    kv.value = "Up down error";
    break;
  default:
    kv.value = "Unknown";
  }
  hb.values.push_back(kv);

  if(inmsg->rc_emergency_stop)
  {
    kv.key = "rc_emergency_stop";
    kv.value = "True";
    hb.values.push_back(kv);
  }

  if(inmsg->cable_emergency_stop)
  {
    kv.key = "cable_emergency_stop";
    kv.value = "True";
    hb.values.push_back(kv);
  }

  if(inmsg->hmi_emergency_stop)
  {
    kv.key = "hmi_emergency_stop";
    kv.value = "True";
    hb.values.push_back(kv);
  }

  kv.key = "shutdown_status";
  switch (inmsg->shutdown_status)
  {
  case drix_msgs::DrixOutput::NO_SHUTDOWN_:
    kv.value = "No Shutdown";
    break;
  case drix_msgs::DrixOutput::SHUTDOWN_REQUESTED_:
    kv.value = "Shutdown Requested";
    break;
  case drix_msgs::DrixOutput::REBOOT_REQUESTED_:
    kv.value = "Reboot Requested";
    break;
  default:
    kv.value = "Unknown";
  }
  hb.values.push_back(kv);

  heartbeat_pub.publish(hb);
}


void currentSpeedCallback(const std_msgs::Float32::ConstPtr& inmsg)
{
    current_speed = inmsg->data;
}

void sendPath()
{
    mdt_msgs::GeoPath gpath;
    gpath.stamp = ros::Time::now();
    gpath.is_first_point_start_point = true;
    if (!last_helm_time.isZero()&&gpath.stamp-last_helm_time>ros::Duration(.5))
    {
        throttle = 0.0;
        rudder = 0.0;
    }
    
    geographic_visualization_msgs::GeoVizItem vizItem;
    vizItem.id = "drix_helm";
    geographic_visualization_msgs::GeoVizPointList plist;
    plist.color.r = 1.0;
    plist.color.g = 0.5;
    plist.color.b = 0.0;
    plist.color.a = 1.0;
    plist.size = 3;

    if(!standby)
    {
        p11::LatLongDegrees vehicle_position, p2;
        vehicle_position[0] = current_position.latitude;
        vehicle_position[1] = current_position.longitude;

        p11::AngleRadians desired_heading = p11::AngleDegrees(last_gps.heading) + p11::AngleRadians(rudder);
        p2 = p11::WGS84::direct(vehicle_position,desired_heading,20);
        mdt_msgs::GeoPathPoint gpoint1,gpoint2;
        gpoint1.speed = throttle*current_speed;
        gpoint1.lat = vehicle_position[0];
        gpoint1.lon = vehicle_position[1];
        gpath.points.push_back(gpoint1);

        gpoint2.speed = throttle*current_speed;
        gpoint2.lat = p2[0];
        gpoint2.lon = p2[1];
        gpath.points.push_back(gpoint2);

        geographic_msgs::GeoPoint gp;
        gp.latitude = vehicle_position[0];
        gp.longitude = vehicle_position[1];
        plist.points.push_back(gp);

        geographic_msgs::GeoPoint gp2;
        gp2.latitude = p2[0];
        gp2.longitude = p2[1];
        plist.points.push_back(gp2);
    }
    else
    {
        mdt_msgs::GeoPathPoint gpoint;
        gpoint.speed = 0.0;
        gpath.points.push_back(gpoint);
        gpath.points.push_back(gpoint);
    }

    vizItem.lines.push_back(plist);
    display_pub.publish(vizItem);

    backseat_path_pub.publish(gpath);
}

void gpsCallback(const mdt_msgs::Gps::ConstPtr& inmsg)
{
    last_gps = *inmsg;

    sensor_msgs::NavSatFix nsf;
    nsf.header = inmsg->header;
    nsf.header.frame_id = frame_id;
    nsf.latitude = inmsg->latitude;
    nsf.longitude = inmsg->longitude;
    nsf.altitude = inmsg->altitude;
    position_pub.publish(nsf);

    current_position.latitude = inmsg->latitude;
    current_position.longitude = inmsg->longitude;

    sensor_msgs::Imu imu;
    imu.header = inmsg->header;
    imu.header.frame_id = frame_id;
    tf2::Quaternion q;
    double yaw = M_PI*(90-inmsg->heading)/180.0;
    q.setRPY(0, 0, yaw);
    tf2::convert(q, imu.orientation);
    imu.angular_velocity_covariance[0] = -1;
    imu.linear_acceleration_covariance[0] = -1;
    orientation_pub.publish(imu);

    geometry_msgs::TwistWithCovarianceStamped twcs;
    twcs.header = inmsg->header;
    twcs.header.frame_id =  frame_id;
    twcs.twist.twist.linear.x = inmsg->sog*cos(yaw);
    twcs.twist.twist.linear.y = inmsg->sog*sin(yaw);
    velocity_pub.publish(twcs);

    sendPath();
}

void aisCallback(const mdt_msgs::StampedString::ConstPtr& msg)
{
  nmea_msgs::Sentence sentence;
  sentence.header = msg->header;
  sentence.sentence = msg->data;
  ais_pub.publish(sentence);
}


int main(int argc, char **argv)
{
  current_speed = 5.0;
  throttle = 0.0;
  rudder = 0.0;
  standby = true;

  ros::init(argc, argv, "drix_helm");
  ros::NodeHandle n;
  
  frame_id = ros::param::param<std::string>("~frame_id", "base_link");

  orientation_pub = n.advertise<sensor_msgs::Imu>("project11/nav/oem/orientation",1);
  position_pub = n.advertise<sensor_msgs::NavSatFix>("project11/nav/oem/position",1);
  velocity_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("project11/nav/oem/velocity",1);
  heartbeat_pub = n.advertise<project11_msgs::Heartbeat>("project11/status/helm", 10);

  backseat_path_pub = n.advertise<mdt_msgs::GeoPath>("/autopilot/guidance_manager/backseat_path", 10);
  display_pub = n.advertise<geographic_visualization_msgs::GeoVizItem>("project11/display",5);
  ais_pub = n.advertise<nmea_msgs::Sentence>("sensors/ais/nmea", 10);

  ros::Subscriber asv_helm_sub = n.subscribe("project11/control/helm", 5, helmCallback);
  ros::Subscriber standby_sub = n.subscribe("project11/piloting_mode/standby/active", 10,standbyCallback);

  ros::Subscriber vehicle_state_sub =  n.subscribe("/hardware/drix_status",10,vehicleSatusCallback);
  ros::Subscriber gps_sub = n.subscribe("/pos/gps",10,gpsCallback);
  ros::Subscriber ais_sub = n.subscribe("/sensors/ais/ais_receiver/raw_ais", 5, aisCallback);
    
  ros::spin();
  
  return 0;
}

    
