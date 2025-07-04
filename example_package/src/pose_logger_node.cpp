#include <ros/ros.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <nav_msgs/Odometry.h>
#include <fstream>
#include <iomanip>
#include <string>

class PoseLogger
{
public:
    PoseLogger(ros::NodeHandle& nh)
    {
        // Öffne Datei zum Schreiben
        log_file_.open("pose_log.csv", std::ios::out | std::ios::trunc);
        log_file_ << "timestamp,filter,x,y,theta,P_xx,P_yy,P_tt\n";

        ekf_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
        "/pose_ekf", 10,
        [this](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
            this->poseCallback(msg, "EKF");
        });

        kf_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/pose_kf", 10,
            [this](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
                this->poseCallback(msg, "KF");
            });

        pf_sub_ = nh.subscribe<geometry_msgs::PoseWithCovarianceStamped>(
            "/pose_pf", 10,
            [this](const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg) {
                this->poseCallback(msg, "PF");
        });
 
        odom_sub_ = nh.subscribe<nav_msgs::Odometry>(
        "/odom", 10,
        [this](const nav_msgs::Odometry::ConstPtr& msg) {
            this->odomCallback(msg);
        });


    }

    ~PoseLogger()
    {
        if (log_file_.is_open()) log_file_.close();
    }

private:
    std::ofstream log_file_;
    ros::Subscriber ekf_sub_, kf_sub_, pf_sub_, odom_sub_;

    void poseCallback(const geometry_msgs::PoseWithCovarianceStamped::ConstPtr& msg, const std::string& source)
    {
        double timestamp = msg->header.stamp.toSec();
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);

        double P_xx = msg->pose.covariance[0];
        double P_yy = msg->pose.covariance[7];
        double P_tt = msg->pose.covariance[14];

        logPose(timestamp, source, x, y, yaw, P_xx, P_yy, P_tt);

    }

    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
    {
        double timestamp = msg->header.stamp.toSec();
        double x = msg->pose.pose.position.x;
        double y = msg->pose.pose.position.y;

        tf2::Quaternion q;
        tf2::fromMsg(msg->pose.pose.orientation, q);
        double roll, pitch, yaw;
        tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
        double P_xx = msg->pose.covariance[0];
        double P_yy = msg->pose.covariance[7];
        double P_tt = msg->pose.covariance[14];

        logPose(timestamp, "Odom", x, y, yaw, P_xx, P_yy, P_tt);

    }

    void logPose(double time, const std::string& source, double x, double y, double theta,
             double P_xx, double P_yy, double P_tt)
    {
        if (!log_file_.is_open()) return;

        log_file_ << std::fixed << std::setprecision(6)
          << time << "," << source << ","
          << x << "," << y << "," << theta << ","
          << P_xx << "," << P_yy << "," << P_tt << "\n";

    }
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "pose_logger_node");
    ros::NodeHandle nh;

    PoseLogger logger(nh);
    ros::spin();
    return 0;
}