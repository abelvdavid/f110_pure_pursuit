#include "f110_pure_pursuit/pure_pursuit.h"

int main(int argc, char** argv) {
    ros::init(argc, argv, "pure_pursuit");
    ros::NodeHandle nh;
    // class init here
    PurePursuit PurePursuit(nh);
    ros::spin();
    return 0;
}