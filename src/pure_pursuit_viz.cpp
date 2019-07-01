#include "f110_pure_pursuit/pure_pursuit_viz.h"

PurePursuitViz::~PurePursuitViz() {
    ROS_INFO("Pure Pursuit Viz shutting down");
}

PurePursuitViz::PurePursuitViz(ros::NodeHandle &nh) : nh_(nh) {
    std::string wpt_topic, wpt_marker_topic;
    nh_.getParam("wpt_viz_topic", wpt_topic);
    nh_.getParam("wpt_marker_topic", wpt_marker_topic);
    nh_.getParam("scan_frame", scan_frame);

    wpt_pub = nh_.advertise<visualization_msgs::Marker>(wpt_marker_topic, 10);
    wpt_sub = nh_.subscribe(wpt_topic, 10, &PurePursuitViz::wpt_callback, this);
}

void PurePursuitViz::wpt_callback(const geometry_msgs::Point::ConstPtr& msg) {
    visualization_msgs::Marker marker;
    marker.header.frame_id = scan_frame;
    marker.type = marker.SPHERE;
    marker.scale.x = 0.3;
    marker.scale.y = 0.3;
    marker.scale.z = 0.3;
    marker.color.a = 1.0;
    marker.color.r = 0.0;
    marker.color.g = 1.0;
    marker.color.b = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.pose.position.x = msg->x;
    marker.pose.position.y = msg->y;
    marker.pose.position.z = 0.0;
    wpt_pub.publish(marker);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit_viz");
    ros::NodeHandle nh;
    // class init here
    PurePursuitViz PurePursuitViz(nh);
    ros::spin();
    return 0;
}