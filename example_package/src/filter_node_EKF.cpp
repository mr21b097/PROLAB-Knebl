#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/Twist.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <map>

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class EKFNode {
public:
    EKFNode(ros::NodeHandle &nh)
    : gen_(ros::Time::now().toNSec())
    {
        imu_sub_ = nh.subscribe("/imu", 10, &EKFNode::imuCallback, this);
        cmd_sub_ = nh.subscribe("/cmd_vel", 10, &EKFNode::cmdVelCallback, this);
        scan_sub_ = nh.subscribe("/scan", 10, &EKFNode::scanCallback, this);

        pub_ = nh.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_ekf", 10);

        // Params
        nh.param("scan_noise_std", scan_noise_std_, 0.05);
        nh.param("imu_yaw_noise_std", imu_yaw_noise_std_, 0.02);
        nh.param("landmark_match_threshold", lm_threshold_, 0.7);

        // Zustand & Unsicherheit
        x_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6);
        P_(0,0) = P_(1,1) = 1.0;  // große Anfangsunsicherheit in x, y
        P_(2,2) = 0.5;

        H_ = Eigen::MatrixXd::Zero(2, 6);
        H_(0, 2) = 1.0;
        H_(1, 5) = 1.0;

        Q_ = Eigen::MatrixXd::Identity(6, 6) * 0.01;
        R_ = Eigen::MatrixXd::Identity(2, 2) * 0.1;

        landmark_map_[1] = Eigen::Vector2d(2.0, 1.0);
        landmark_map_[2] = Eigen::Vector2d(4.0, 3.0);
        landmark_map_[3] = Eigen::Vector2d(1.0, -2.0);

        scan_noise_dist_ = std::normal_distribution<double>(0.0, scan_noise_std_);
        imu_yaw_noise_dist_ = std::normal_distribution<double>(0.0, imu_yaw_noise_std_);

        last_time_ = ros::Time(0);
    }

private:
    ros::Subscriber imu_sub_, cmd_sub_, scan_sub_;
    ros::Publisher pub_;
    ros::Time last_time_;
    geometry_msgs::Twist last_cmd_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, H_, Q_, R_;
    std::map<int, Eigen::Vector2d> landmark_map_;

    double scan_noise_std_, imu_yaw_noise_std_, lm_threshold_;
    std::default_random_engine gen_;
    std::normal_distribution<double> scan_noise_dist_;
    std::normal_distribution<double> imu_yaw_noise_dist_;

    void cmdVelCallback(const geometry_msgs::Twist::ConstPtr& msg)
    {
        last_cmd_ = *msg;
    }

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg)
    {
        ros::Time current_time = imu_msg->header.stamp;
        if (last_time_.isZero()) {
            last_time_ = current_time;
            return;
        }

        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        if (dt <= 0.0) return;

        double vx = last_cmd_.linear.x;
        double vy = 0.0;
        double omega = imu_msg->angular_velocity.z;

        double yaw = x_(2);
        double vx_world = std::cos(yaw) * vx - std::sin(yaw) * vy;
        double vy_world = std::sin(yaw) * vx + std::cos(yaw) * vy;

        Eigen::Vector3d u;
        u << vx_world, vy_world, omega;

        tf2::Quaternion quat;
        tf2::fromMsg(imu_msg->orientation, quat);
        double roll, pitch, yaw_imu;
        tf2::Matrix3x3(quat).getRPY(roll, pitch, yaw_imu);

        Eigen::VectorXd z(2);
        z << yaw_imu + imu_yaw_noise_dist_(gen_), omega;

        predict(u, dt);
        correct(z);

        geometry_msgs::PoseWithCovarianceStamped pred_msg;
        pred_msg.header.stamp = current_time;
        pred_msg.header.frame_id = "odom";
        pred_msg.pose.pose.position.x = x_(0);
        pred_msg.pose.pose.position.y = x_(1);

        tf2::Quaternion q_out;
        q_out.setRPY(0, 0, x_(2));
        pred_msg.pose.pose.orientation = tf2::toMsg(q_out);

        for (int i = 0; i < 6; ++i)
            for (int j = 0; j < 6; ++j)
                pred_msg.pose.covariance[i * 6 + j] = P_(i, j);

        pub_.publish(pred_msg);
    }

    void predict(const Eigen::Vector3d& u, double dt)
    {
        double vx = u(0), vy = u(1), omega = u(2);
        x_(0) += vx * dt;
        x_(1) += vy * dt;
        x_(2) = normalize_angle(x_(2) + omega * dt);
        x_(3) = vx;
        x_(4) = vy;
        x_(5) = omega;

        Eigen::MatrixXd F = Eigen::MatrixXd::Identity(6, 6);
        Eigen::MatrixXd V = Eigen::MatrixXd::Zero(6, 3);
        V(0, 0) = dt;
        V(1, 1) = dt;
        V(2, 2) = dt;
        V(3, 0) = 1.0;
        V(4, 1) = 1.0;
        V(5, 2) = 1.0;

        Eigen::MatrixXd M = Eigen::MatrixXd::Zero(3, 3);
        M(0,0) = 0.02;  // vx noise
        M(1,1) = 0.02;  // vy noise
        M(2,2) = 0.01;  // omega noise

        P_ = F * P_ * F.transpose() + V * M * V.transpose() + Q_;
    }

    void correct(const Eigen::VectorXd& z)
    {
        Eigen::VectorXd y = z - H_ * x_;
        y(0) = normalize_angle(y(0));

        Eigen::MatrixXd S = H_ * P_ * H_.transpose() + R_;
        Eigen::MatrixXd K = P_ * H_.transpose() * S.inverse();

        x_ = x_ + K * y;
        P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_) * P_;
        x_(2) = normalize_angle(x_(2));
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg)
    {
        int center_index = scan_msg->ranges.size() / 2;
        double raw_range = scan_msg->ranges[center_index];
        double measured_range = raw_range + scan_noise_dist_(gen_);

        for (const auto& lm_pair : landmark_map_) {
            int id = lm_pair.first;
            const Eigen::Vector2d& lm = lm_pair.second;

            double dx = lm(0) - x_(0);
            double dy = lm(1) - x_(1);
            double expected_range = std::sqrt(dx * dx + dy * dy);
            double y_resid = measured_range - expected_range;

            ROS_INFO_STREAM("Landmark " << id << ": expected=" << expected_range 
                << ", measured=" << measured_range << ", resid=" << y_resid);

            if (std::abs(y_resid) < lm_threshold_) {
                double q = dx * dx + dy * dy;
                if (q < 1e-6) continue;

                Eigen::MatrixXd H_lm = Eigen::MatrixXd::Zero(1, 6);
                H_lm(0, 0) = -dx / std::sqrt(q);
                H_lm(0, 1) = -dy / std::sqrt(q);

                Eigen::MatrixXd R_lm(1,1);
                R_lm(0,0) = 0.1;

                Eigen::MatrixXd S = H_lm * P_ * H_lm.transpose() + R_lm;
                Eigen::MatrixXd K = P_ * H_lm.transpose() * S.inverse();

                x_ = x_ + K * y_resid;
                P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H_lm) * P_;
                x_(2) = normalize_angle(x_(2));

                ROS_INFO_STREAM("→ Correction applied (id=" << id << ")");
            }
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "ekf_node");
    ros::NodeHandle nh;
    EKFNode node(nh);
    ros::spin();
    return 0;
}
