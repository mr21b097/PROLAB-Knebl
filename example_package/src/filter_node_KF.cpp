#include <ros/ros.h>
#include <eigen3/Eigen/Dense>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <map>
#include <tf2/utils.h> 

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class FilterNode {
public:
    FilterNode(ros::NodeHandle& nh) : nh_(nh), gen_(ros::Time::now().toNSec()) {
        imu_sub_ = nh_.subscribe("/imu", 10, &FilterNode::imuCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 10, &FilterNode::scanCallback, this);
        pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_kf", 10);

        x_ = Eigen::VectorXd::Zero(6);
        P_ = Eigen::MatrixXd::Identity(6, 6) * 0.05;
        Q_ = Eigen::MatrixXd::Zero(6, 6);
        Q_(0, 0) = Q_(1, 1) = 0.002;
        Q_(2, 2) = Q_(5, 5) = 1e-4;
        Q_(3, 3) = Q_(4, 4) = 0.005;

        scan_noise_std_ = 0.1;
        scan_noise_dist_ = std::normal_distribution<double>(0.0, scan_noise_std_);

        nh_.param("scan_match_angle_threshold_deg", scan_match_angle_threshold_deg_, 45.0);
        nh_.param("scan_match_residual_threshold", scan_match_residual_threshold_, 0.5);
        scan_match_angle_threshold_rad_ = scan_match_angle_threshold_deg_ * M_PI / 180.0;

        landmark_map_[1] = Eigen::Vector2d(2.0, 1.0);
        landmark_map_[2] = Eigen::Vector2d(4.0, 3.0);
        landmark_map_[3] = Eigen::Vector2d(1.0, -2.0);

        last_time_ = ros::Time(0);
    }

private:
    ros::NodeHandle nh_;
    ros::Subscriber imu_sub_, scan_sub_;
    ros::Publisher pub_;
    ros::Time last_time_;

    Eigen::VectorXd x_;
    Eigen::MatrixXd P_, Q_;
    std::map<int, Eigen::Vector2d> landmark_map_;

    std::default_random_engine gen_;
    std::normal_distribution<double> noise_dist_{0.0, 0.02};
    std::normal_distribution<double> scan_noise_dist_;
    double scan_noise_std_;

    double scan_match_angle_threshold_deg_;
    double scan_match_angle_threshold_rad_;
    double scan_match_residual_threshold_;

    void imuCallback(const sensor_msgs::Imu::ConstPtr& imu_msg) {
        ros::Time current_time = imu_msg->header.stamp;
        if (last_time_.isZero()) {
            last_time_ = current_time;
            return;
        }

        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        if (dt <= 0.0) return;

        double ax = imu_msg->linear_acceleration.x + noise_dist_(gen_);
        double ay = imu_msg->linear_acceleration.y + noise_dist_(gen_);
        double omega = imu_msg->angular_velocity.z + noise_dist_(gen_);

        Eigen::MatrixXd A = Eigen::MatrixXd::Identity(6, 6);
        A(0, 3) = dt;
        A(1, 4) = dt;
        A(2, 5) = dt;

        Eigen::MatrixXd B = Eigen::MatrixXd::Zero(6, 3);
        B(3, 0) = dt;
        B(4, 1) = dt;
        B(5, 2) = dt;

        double theta = x_(2);
        double ax_world = std::cos(theta) * ax - std::sin(theta) * ay;
        double ay_world = std::sin(theta) * ax + std::cos(theta) * ay;

        Eigen::Vector3d u(ax_world, ay_world, omega);
        x_ = A * x_ + B * u;
        x_(2) = normalize_angle(x_(2));
        P_ = A * P_ * A.transpose() + Q_;

        // --- YAW-KORREKTUR AUS IMU ---
        if (imu_msg->orientation_covariance[8] >= 0.0) {
            tf2::Quaternion q_imu;
            tf2::fromMsg(imu_msg->orientation, q_imu);
            double yaw_imu = tf2::getYaw(q_imu);
            double resid_theta = normalize_angle(yaw_imu - x_(2));

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
            H(0, 2) = 1.0;

            double R_theta_val = imu_msg->orientation_covariance[8];
            Eigen::MatrixXd R_theta = Eigen::MatrixXd::Constant(1, 1, R_theta_val);
            Eigen::MatrixXd S = H * P_ * H.transpose() + R_theta;
            Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

            x_ += K * resid_theta;
            x_(2) = normalize_angle(x_(2));
            P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;
        }

        // Begrenzung der Unsicherheit
        if (P_(2, 2) > 0.3) P_(2, 2) = 0.3;

        ROS_INFO_STREAM_THROTTLE(5.0, "Yaw estimate: " << x_(2) << " | Var: " << P_(2, 2));

        publishPose(current_time);
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& scan_msg) {
        if (scan_msg->ranges.empty()) return;

        int center_index = scan_msg->ranges.size() / 2;
        double raw_range = scan_msg->ranges[center_index];
        if (!std::isfinite(raw_range) || raw_range <= 0.0) return;

        double measured_range = raw_range + scan_noise_dist_(gen_);

        for (const auto& lm_pair : landmark_map_) {
            const Eigen::Vector2d& landmark = lm_pair.second;
            double dx = landmark(0) - x_(0);
            double dy = landmark(1) - x_(1);
            double q = dx * dx + dy * dy;
            if (q < 1e-6) continue;

            double expected_range = std::sqrt(q);
            double residual = measured_range - expected_range;

            double angle_to_lm = std::atan2(dy, dx);
            double angle_diff = normalize_angle(angle_to_lm - x_(2));
            if (std::abs(angle_diff) > scan_match_angle_threshold_rad_) continue;
            if (std::abs(residual) > scan_match_residual_threshold_) continue;

            Eigen::MatrixXd H = Eigen::MatrixXd::Zero(1, 6);
            H(0, 0) = -dx / expected_range;
            H(0, 1) = -dy / expected_range;

            Eigen::MatrixXd R = Eigen::MatrixXd::Constant(1, 1, scan_noise_std_ * scan_noise_std_);
            Eigen::MatrixXd S = H * P_ * H.transpose() + R;
            Eigen::MatrixXd K = P_ * H.transpose() * S.inverse();

            x_ += K * residual;
            P_ = (Eigen::MatrixXd::Identity(6, 6) - K * H) * P_;

            ROS_INFO_STREAM("KF correction from landmark " << lm_pair.first
                            << " | residual: " << residual
                            << " | angle_diff: " << angle_diff * 180.0 / M_PI
                            << " | P_trace: " << P_.trace());
        }
    }

    void publishPose(const ros::Time& stamp) {
        geometry_msgs::PoseWithCovarianceStamped msg;
        msg.header.stamp = stamp;
        msg.header.frame_id = "odom";
        msg.pose.pose.position.x = x_(0);
        msg.pose.pose.position.y = x_(1);

        tf2::Quaternion q;
        q.setRPY(0, 0, x_(2));
        msg.pose.pose.orientation = tf2::toMsg(q);

        for (int i = 0; i < 36; ++i) msg.pose.covariance[i] = 0.0;
        msg.pose.covariance[0]  = P_(0, 0);
        msg.pose.covariance[7]  = P_(1, 1);
        msg.pose.covariance[35] = P_(2, 2);

        pub_.publish(msg);
    }
};

int main(int argc, char **argv) {
    ros::init(argc, argv, "filter_node_kf");
    ros::NodeHandle nh("~");
    FilterNode node(nh);
    ros::spin();
    return 0;
}
