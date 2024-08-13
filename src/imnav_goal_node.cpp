#include <imnav_goal/imnav_goal.h>

ImnavGoal::ImnavGoal() : origin_received_(false), ac("move_base", true) {
    origin_sub_ = nh_.subscribe("/origin_gps", 1, &ImnavGoal::originCallback, this);
    goal_sub_ = nh_.subscribe("/gps_goal_fix", 1, &ImnavGoal::goalCallback, this);
    while(!ac.waitForServer(ros::Duration(5.0))){
        ROS_INFO("Waiting for the move_base action server to come up");
    }
    ROS_INFO("Connected to move_base");
}

void ImnavGoal::originCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!origin_received_) {
        origin_gps_ = *msg;
        origin_received_ = true;
        ROS_INFO("Origin GPS received: Lat=%f, Lon=%f", origin_gps_.latitude, origin_gps_.longitude);
    }
}

void ImnavGoal::goalCallback(const sensor_msgs::NavSatFix::ConstPtr& msg) {
    if (!origin_received_) {
        ROS_WARN("Origin GPS not received yet. Ignoring goal.");
        return;
    }

    last_goal_ = *msg;
    processAndPublishGoal();
}

void ImnavGoal::processAndPublishGoal() {
    auto [origin_x, origin_y, origin_zone] = gpsToUTM(origin_gps_.latitude, origin_gps_.longitude);
    auto [goal_x, goal_y, goal_zone] = gpsToUTM(last_goal_.latitude, last_goal_.longitude);

    if (origin_zone != goal_zone) {
        ROS_WARN("Origin and goal are in different UTM zones. This may cause inaccuracies.");
    }

    double x_star = goal_x - origin_x;
    double y_star = goal_y - origin_y;

    goal.target_pose.header.frame_id = "map";
    goal.target_pose.header.stamp = ros::Time::now();

    goal.target_pose.pose.position.x = x_star;
    goal.target_pose.pose.position.y = y_star;

    ac.sendGoal(goal);

    ROS_INFO("Published new goal: x=%f, y=%f", x_star, y_star);

    ac.waitForResult();

    if (ac.getState() == actionlib::SimpleClientGoalState::SUCCEEDED) {
        ROS_INFO("Goal achieved, proceeding with next goal");
    }
    else {
        ROS_INFO("Failed to achieve goal");
    }
}

std::tuple<double, double, int> ImnavGoal::gpsToUTM(double lat, double lon) {
    int zone;
    bool northp;
    double x, y;
    GeographicLib::UTMUPS::Forward(lat, lon, zone, northp, x, y);
    return std::make_tuple(x, y, zone);
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "imnav_goal_node");
    ImnavGoal imnav_goal;
    ros::spin();
    return 0;
}
