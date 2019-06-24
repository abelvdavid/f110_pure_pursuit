// ros stuff
#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/Point.h>
#include <ackermann_msgs/AckermannDriveStamped.h>
// standard
#include <math.h>
#include <stdlib.h>
#include <vector>
#include <array>
#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <algorithm>
#include <boost/algorithm/string.hpp>

// Eigen
#include <Eigen/Dense>
#include <Eigen/Geometry>
using namespace Eigen;

class PurePursuit {
public:
    PurePursuit(ros::NodeHandle nh);
    virtual ~PurePursuit();
private:
    // ros stuff
    // node handles
    ros::NodeHandle nh_;
    
    // subscribers
    ros::Subscriber pf_sub_;
    ros::Subscriber waypt_sub_;
    
    // publishers
    ros::Publisher drive_pub_;
    ros::Publisher waypt_pub_;

    // tf stuff
    tf::TransformListener listener;
    
    // params
    geometry_msgs::Point current_wpt;
    double velocity, lookahead_distance, checking_angle, wheelbase, steer_p;
    bool use_csv, use_sim;
    std::vector< std::array<double,2> > waypoints;

    // methods
    void wpt_callback(const geometry_msgs::Point::ConstPtr& wpt_msg);
    void callback_csv(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    void callback_pt(const geometry_msgs::PoseStamped::ConstPtr& pose_msg);
    double dist(std::array<double,2> p1, std::array<double,2> p2);
    bool dir(std::array<double,2> pos, std::array<double,2> wpt, double theta);
};


class CSVReader {
public:
    CSVReader(std::string file, std::string delim=",");
    virtual ~CSVReader();
    // attr
    std::string file_name;
    std::string delimeter;

    // methods
    std::vector< std::array<double,2> > getWpt();
};