#include "f110_pure_pursuit/pure_pursuit.h"

using namespace Eigen;

PurePursuit::~PurePursuit() {
    ROS_INFO("Pure pursuit shutting down");
} // End ScanMatcher destructor
PurePursuit::PurePursuit(ros::NodeHandle nh)
   :nh_(nh) {
    // getting params
    // topics
    std::string pose_topic, drive_topic, sim_drive_topic, wpt_topic, wpt_viz_topic;
    nh_.getParam("pose_topic", pose_topic);
    nh_.getParam("drive_topic", drive_topic);
    nh_.getParam("sim_drive_topic", sim_drive_topic);
    nh_.getParam("wpt_topic", wpt_topic);
    nh_.getParam("wpt_viz_topic", wpt_viz_topic);

    // files
    std::string wpt_file;
    nh_.getParam("waypoint_file", wpt_file);

    // params
    nh_.getParam("use_csv", use_csv);
    nh_.getParam("use_sim", use_sim);
    nh_.getParam("velocity", velocity);
    nh_.getParam("lookahead_distance", lookahead_distance);
    nh_.getParam("checking_angle", checking_angle);
    nh_.getParam("wheelbase", wheelbase);
    nh_.getParam("steer_p", steer_p);

    if (use_csv) {
        std::string pkg_path = ros::package::getPath("f110_pure_pursuit");
        CSVReader csvreader = CSVReader(pkg_path+wpt_file);
        waypoints = csvreader.getWpt();
        pf_sub_ = nh_.subscribe(pose_topic, 10, &PurePursuit::callback_csv, this);
    } else {
        waypt_sub_ = nh_.subscribe(wpt_topic, 10, &PurePursuit::wpt_callback, this);
        pf_sub_ = nh_.subscribe(pose_topic, 10, &PurePursuit::callback_pt, this);
    }

    if (use_sim) {
        drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(sim_drive_topic, 10);
    } else {
        drive_pub_ = nh_.advertise<ackermann_msgs::AckermannDriveStamped>(drive_topic, 10);
    }

    waypt_pub_ = nh_.advertise<geometry_msgs::Point>(wpt_viz_topic, 10);

    
} // end PurePursuit constructor

void PurePursuit::wpt_callback(const geometry_msgs::Point::ConstPtr& wpt_msg) {
    current_wpt.x = wpt_msg->x;
    current_wpt.y = wpt_msg->y;
    current_wpt.z = wpt_msg->z;
}

void PurePursuit::callback_pt(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    double radius = 1/(2.0*current_wpt.y/pow(lookahead_distance, 2));
    double steering_angle = steer_p * atan(wheelbase/radius);
    if (steering_angle < -0.4189) steering_angle = -0.4189;
    if (steering_angle > 0.4189) steering_angle = 0.4189;
    ackermann_msgs::AckermannDriveStamped msg;
    if (std::abs(steering_angle) > 0.35) msg.drive.speed = velocity - 0.4;
    else msg.drive.speed = velocity;
    msg.drive.steering_angle = steering_angle;
    drive_pub_.publish(msg);
    ROS_INFO("steering angle: %f", steering_angle);
    // publishing waypoints for debugging
    waypt_pub_.publish(current_wpt);
}

void PurePursuit::callback_csv(const geometry_msgs::PoseStamped::ConstPtr& pose_msg) {
    // get current pose
    double x = pose_msg->pose.position.x;
    double y = pose_msg->pose.position.y;
    double z = pose_msg->pose.position.z;
    double qx = pose_msg->pose.orientation.x;
    double qy = pose_msg->pose.orientation.y;
    double qz = pose_msg->pose.orientation.z;
    double qw = pose_msg->pose.orientation.w;
    std::array<double,2> pos{ {x, y} };//2d position
    tf::Quaternion q(qx, qy, qz, qw);
    tf::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);
    // Quaterniond q = Quaterniond(qx, qy, qz, qw);
    // auto euler = q.toRotationMatrix().eulerAngles(2,1,0);
    // double theta = euler(0); //yaw
    double theta = yaw;
    // ROS_INFO("theta: %f", theta);
    // get best waypoint
    std::vector< std::array<double,2> > valid_points;
    std::vector<double> distances;
    for (int i=0; i<waypoints.size(); i++) {
        std::array<double,2> point = waypoints[i];
        double dist = this->dist(point, pos);
        bool same_dir = this->dir(pos, point, theta);
        if (dist < 3.0*lookahead_distance && dist >= lookahead_distance && same_dir) {
            valid_points.push_back(point);
            distances.push_back(dist);
        }
    }
    if (valid_points.empty()) {
        ackermann_msgs::AckermannDriveStamped zero_msg;
        zero_msg.drive.speed = 0.0;
        zero_msg.drive.steering_angle = 0.0;
        drive_pub_.publish(zero_msg);
        return;
    }
    auto min_iter = std::min_element(distances.begin(), distances.end());
    int min_ind = min_iter - distances.begin();
    std::array<double,2> best_point = valid_points[min_ind];
    // transform goal point into car frame
    geometry_msgs::PointStamped p;
    geometry_msgs::PointStamped best_point_car;
    best_point_car.header.frame_id = "/laser";
    p.header.frame_id = "/map";
    p.point.x = best_point[0];
    p.point.y = best_point[1];
    p.point.z = 0.0;
    // auto t = listener.getLatestCommonTime("/laser", "/map", ros::Time::now(), "lol?");
    listener.waitForTransform("/laser", "/map", ros::Time::now(), ros::Duration(0.1));
    listener.transformPoint("/laser", p, best_point_car);
    double radius = 1/(2.0*best_point_car.point.y/pow(lookahead_distance, 2));
    double steering_angle = steer_p * atan(wheelbase/radius);
    if (steering_angle < -0.4189) steering_angle = -0.4189;
    if (steering_angle > 0.4189) steering_angle = 0.4189;
    ackermann_msgs::AckermannDriveStamped msg;
    if (std::abs(steering_angle) > 0.35) msg.drive.speed = velocity - 0.4;
    else msg.drive.speed = velocity;
    msg.drive.steering_angle = steering_angle;
    drive_pub_.publish(msg);
    // publishing waypoints for debugging
    geometry_msgs::Point waypoint_car;
    waypoint_car = best_point_car.point;
    waypt_pub_.publish(waypoint_car);
} // end scan callback

double PurePursuit::dist(std::array<double, 2> p1, std::array<double,2> p2) {
    return sqrt(pow((p1[0]-p2[0]),2) + pow((p1[1]-p2[1]),2));
} // end dist calc

bool PurePursuit::dir(std::array<double,2> pos, std::array<double,2> wpt, double theta) {
    Vector3d vec = Vector3d((wpt[0]-pos[0]), (wpt[1]-pos[1]), 0);
    Vector3d unit_vec = vec/vec.norm();
    // std::array<double,2> vec{ {(wpt[0]-pos[0]), (wpt[1]-pos[1])} };
    // unit vec?
    Vector3d rotated_x = Vector3d(cos(theta), sin(theta), 0);
    // std::array<double,2> rotated_x{ {cos(theta), sin(theta)} };
    double cosang = unit_vec.dot(rotated_x);
    double sinang = (unit_vec.cross(rotated_x)).norm();
    double angle = atan2(sinang, cosang);
    return (std::abs(angle) <= checking_angle);
} // end dir 

CSVReader::~CSVReader() {
    ROS_INFO("CSVReader shutting down");
} // end csvreader destructor

CSVReader::CSVReader(std::string file,
                     std::string delim)
    :file_name(file),
     delimeter(delim) {
    ROS_INFO("Starting csv reader");
} // end CSVReader constructor

std::vector< std::array<double,2> > CSVReader::getWpt() {
    ROS_INFO("%s", file_name.c_str());
    std::ifstream file(file_name);
    std::vector< std::array<double,2> > waypoints;
    std::string line = "";
    std::string::size_type sz;
    while(getline(file, line)) {
        std::vector<std::string> vec;
        boost::algorithm::split(vec, line, boost::is_any_of(delimeter));
        double x = std::stod(vec[0], &sz);
        double y = std::stod(vec[1], &sz);
        std::array<double,2> point{{x, y}};
        waypoints.push_back(point);
    }
    file.close();
    ROS_INFO("waypoints in list, size: %zd", waypoints.size());
    return waypoints;
}