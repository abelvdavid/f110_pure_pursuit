#include <ros/ros.h>
#include <std_msgs/Float64MultiArray.h>
#include <std_msgs/ColorRGBA.h>
#include <geometry_msgs/Point.h>
#include <visualization_msgs/Marker.h>

#include <vector>

class PurePursuitViz {
public:
    PurePursuitViz(ros::NodeHandle &nh);
    virtual ~PurePursuitViz();
private:
    ros::NodeHandle nh_;

    ros::Publisher wpt_pub;

    ros::Subscriber wpt_sub;

    void wpt_callback(const geometry_msgs::Point::ConstPtr& msg);
};