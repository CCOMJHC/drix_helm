#include "ros/ros.h"
#include "mdt_msgs/Gps.h"
#include "mdt_msgs/GeoPath.h"
#include "drix_msgs/DrixOutput.h"
#include "geometry_msgs/TwistWithCovarianceStamped.h"
#include "marine_msgs/Heartbeat.h"
#include "marine_msgs/Helm.h"
#include "sensor_msgs/NavSatFix.h"
#include "sensor_msgs/Imu.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32.h"
#include "geographic_visualization_msgs/GeoVizItem.h"
#include <vector>
#include "project11/utils.h"

namespace p11 = project11;

ros::Publisher position_pub;
ros::Publisher orientation_pub;
ros::Publisher velocity_pub;
ros::Publisher heartbeat_pub;
ros::Publisher backseat_path_pub;
ros::Publisher display_pub;

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

void helmCallback(const marine_msgs::Helm::ConstPtr& msg)
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
    marine_msgs::Heartbeat hb;
    hb.header.stamp = ros::Time::now();

    marine_msgs::KeyValue kv;

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
    
    kv.key = "drix_mode";
    kv.value = inmsg->drix_mode;
    hb.values.push_back(kv);
    
    kv.key = "clutch";
    kv.value = inmsg->drix_clutch;
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
    kv.value = inmsg->keel_state;
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

int main(int argc, char **argv)
{
    current_speed = 4.0;
    throttle = 0.0;
    rudder = 0.0;
    standby = true;

    ros::init(argc, argv, "drix_helm");
    ros::NodeHandle n;
    
    frame_id = ros::param::param<std::string>("~frame_id", "base_link");


    orientation_pub = n.advertise<sensor_msgs::Imu>("nav/oem/orientation",1);
    position_pub = n.advertise<sensor_msgs::NavSatFix>("nav/oem/position",1);
    velocity_pub = n.advertise<geometry_msgs::TwistWithCovarianceStamped>("nav/oem/velocity",1);
    heartbeat_pub = n.advertise<marine_msgs::Heartbeat>("project11/status/helm", 10);

    backseat_path_pub = n.advertise<mdt_msgs::GeoPath>("/backseat_path", 10);
    display_pub = n.advertise<geographic_visualization_msgs::GeoVizItem>("project11/display",5);

    ros::Subscriber asv_helm_sub = n.subscribe("control/helm", 5, helmCallback);
    ros::Subscriber standby_sub = n.subscribe("piloting_mode/standby/active", 10,standbyCallback);

    ros::Subscriber vehicle_state_sub =  n.subscribe("/drix_status",10,vehicleSatusCallback);
    ros::Subscriber gps_sub = n.subscribe("/gps",10,gpsCallback);
    
    ros::spin();
    
    return 0;
}

    
