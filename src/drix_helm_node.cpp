#include "ros/ros.h"
#include "mdt_msgs/Gps.h"
#include "mdt_msgs/GeoPath.h"
#include "drix_msgs/DrixOutput.h"
#include "marine_msgs/Heartbeat.h"
#include "geographic_msgs/GeoPointStamped.h"
#include "geographic_msgs/GeoPath.h"
#include "geometry_msgs/TwistStamped.h"
#include "std_msgs/Bool.h"
#include "std_msgs/String.h"
#include "marine_msgs/NavEulerStamped.h"
#include <vector>
#include "project11/gz4d_geo.h"

ros::Publisher position_pub;
ros::Publisher heading_pub;
ros::Publisher speed_pub;
ros::Publisher heartbeat_pub;
ros::Publisher backseat_path_pub;

bool active;
std::string helm_mode;

mdt_msgs::Gps last_gps;

double js_turn_rate;
double js_speed;
ros::Time last_js_time;
bool joystick_override = false;

struct LatLong
{
    double latitude;
    double longitude;
};

std::vector<LatLong> current_path;
LatLong current_position;

std::string boolToString(bool value)
{
    if(value)
        return "true";
    return "false";
}

void twistCallback(const geometry_msgs::TwistStamped::ConstPtr& msg)
{
    js_speed = msg->twist.linear.x;
    js_turn_rate = msg->twist.angular.z;
    
    last_js_time = msg->header.stamp;
}

void vehicleSatusCallback(const drix_msgs::DrixOutput::ConstPtr& inmsg)
{
    marine_msgs::Heartbeat hb;
    hb.header.stamp = ros::Time::now();

    marine_msgs::KeyValue kv;

    kv.key = "active";
    kv.value = boolToString(active);
    hb.values.push_back(kv);
    
    kv.key = "helm_mode";
    kv.value = helm_mode;
    hb.values.push_back(kv);
    
    kv.key = "js_override";
    kv.value = boolToString(joystick_override);
    hb.values.push_back(kv);

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

    kv.key = "remote_control_lost";
    kv.value = boolToString(inmsg->remoteControlLost);
    hb.values.push_back(kv);
    
    kv.key = "keel_state";
    kv.value = inmsg->keel_state;
    hb.values.push_back(kv);

    heartbeat_pub.publish(hb);
}

void activeCallback(const std_msgs::Bool::ConstPtr& inmsg)
{
    active = inmsg->data;
}

void helmModeCallback(const std_msgs::String::ConstPtr& inmsg)
{
    helm_mode = inmsg->data;
}

void sendPath()
{
    mdt_msgs::GeoPath gpath;
    gpath.stamp = ros::Time::now();
    joystick_override = false;
    if(active)
    {
        bool doDesired = true;
        if (!last_js_time.isZero())
        {
            if(ros::Time::now()-last_js_time > ros::Duration(.5))
            {
                js_speed = 0.0;
                js_turn_rate = 0.0;
            }
            else
                doDesired = false;
        }

        if(doDesired)
        {
            if(!current_path.empty())
            {
                gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> p1, p2,vehicle_position;
                p1[0] = current_path[0].latitude;
                p1[1] = current_path[0].longitude;
                p2[0] = current_path[1].latitude;
                p2[1] = current_path[1].longitude;
                std::cerr << "p1: " << p1[0] << "," << p1[1] << " p2: " << p2[0] << "," << p2[1] << std::endl;
                
                vehicle_position[0] = current_position.latitude;
                vehicle_position[1] = current_position.longitude;
                        
                auto path_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,p2);
                auto vehicle_azimuth_distance = gz4d::geo::WGS84::Ellipsoid::inverse(p1,vehicle_position);

                std::cerr << "path azimuth: " << path_azimuth_distance.first << " distance: " << path_azimuth_distance.second << std::endl;
                
                double error_azimuth = vehicle_azimuth_distance.first - path_azimuth_distance.first;
                double sin_error_azimuth = sin(error_azimuth*M_PI/180.0);
                double cos_error_azimuth = cos(error_azimuth*M_PI/180.0);
                
                double progress = vehicle_azimuth_distance.second*cos_error_azimuth;
                std::cerr << "progress: " << progress << std::endl;
                auto startPoint = gz4d::geo::WGS84::Ellipsoid::direct(p1,path_azimuth_distance.first,progress);
                
                mdt_msgs::GeoPathPoint gpoint1,gpoint2;
                gpoint1.speed = 4.0;
                gpoint1.lat = startPoint[0];
                gpoint1.lon = startPoint[1];
                gpath.points.push_back(gpoint1);

                gpoint2.speed = 4.0;
                gpoint2.lat = p2[0];
                gpoint2.lon = p2[1];
                gpath.points.push_back(gpoint2);
            }
            else
            {
                mdt_msgs::GeoPathPoint gpoint;
                gpoint.speed = 0.0;
                gpath.points.push_back(gpoint);
                gpath.points.push_back(gpoint);
            }
        }
        else
        {
            // gen path segment from joystick
            joystick_override = true;
            gz4d::geo::Point<double,gz4d::geo::WGS84::LatLon> vehicle_position,p2;
            vehicle_position[0] = current_position.latitude;
            vehicle_position[1] = current_position.longitude;

            double desired_heading = last_gps.heading - (js_turn_rate*180.0/M_PI);
            p2 = gz4d::geo::WGS84::Ellipsoid::direct(vehicle_position,desired_heading,20);
            mdt_msgs::GeoPathPoint gpoint1,gpoint2;
            gpoint1.speed = js_speed;
            gpoint1.lat = vehicle_position[0];
            gpoint1.lon = vehicle_position[1];
            gpath.points.push_back(gpoint1);

            gpoint2.speed = js_speed;
            gpoint2.lat = p2[0];
            gpoint2.lon = p2[1];
            gpath.points.push_back(gpoint2);
        }
    }
    else
    {
        mdt_msgs::GeoPathPoint gpoint;
        gpoint.speed = 0.0;
        gpath.points.push_back(gpoint);
        gpath.points.push_back(gpoint);
    }
    backseat_path_pub.publish(gpath);
}

void gpsCallback(const mdt_msgs::Gps::ConstPtr& inmsg)
{
    last_gps = *inmsg;
    
    geographic_msgs::GeoPointStamped gps;
    gps.header = inmsg->header;
    gps.position.latitude = inmsg->latitude;
    gps.position.longitude = inmsg->longitude;
    position_pub.publish(gps);
    
    current_position.latitude = inmsg->latitude;
    current_position.longitude = inmsg->longitude;

    geometry_msgs::TwistStamped ts;
    ts.header = inmsg->header;
    ts.twist.linear.x = inmsg->sog;
    speed_pub.publish(ts);

    marine_msgs::NavEulerStamped nes;
    nes.header = inmsg->header;
    nes.orientation.heading = inmsg->heading;
    heading_pub.publish(nes);
    
    sendPath();
}

void currentPathCallback(const geographic_msgs::GeoPath::ConstPtr& inmsg)
{
    current_path.clear();
    for(auto pose: inmsg->poses)
    {
        LatLong ll;
        ll.latitude = pose.pose.position.latitude;
        ll.longitude = pose.pose.position.longitude;
        current_path.push_back(ll);
    }
}

int main(int argc, char **argv)
{
    js_speed = 0.0;
    js_turn_rate = 0.0;
    
    ros::init(argc, argv, "drix_helm");
    ros::NodeHandle n;
    
    heading_pub = n.advertise<marine_msgs::NavEulerStamped>("/heading",1);
    position_pub = n.advertise<geographic_msgs::GeoPointStamped>("/position",1);
    speed_pub = n.advertise<geometry_msgs::TwistStamped>("/sog",1);
    heartbeat_pub = n.advertise<marine_msgs::Heartbeat>("/heartbeat", 10);
    backseat_path_pub = n.advertise<mdt_msgs::GeoPath>("/backseat_path", 10);

    ros::Subscriber asv_helm_sub = n.subscribe("/remote/0/cmd_vel",5,twistCallback);
    ros::Subscriber vehicle_state_sub =  n.subscribe("/drix_status",10,vehicleSatusCallback);
    ros::Subscriber gps_sub = n.subscribe("/gps",10,gpsCallback);
    ros::Subscriber active_sub = n.subscribe("/active",10,activeCallback);
    ros::Subscriber helm_mode_sub = n.subscribe("/helm_mode",10,helmModeCallback);
    ros::Subscriber current_path_sub = n.subscribe("/project11/mission_manager/current_path",10,currentPathCallback);
    
    ros::spin();
    
    return 0;
}

    
