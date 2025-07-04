#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <vector>
#include <map>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>
#include <visualization_msgs/Marker.h>


struct Particle {
    double x;
    double y;
    double theta;
    double weight;
};

double normalize_angle(double angle) {
    while (angle > M_PI) angle -= 2.0 * M_PI;
    while (angle < -M_PI) angle += 2.0 * M_PI;
    return angle;
}

class ParticleFilter {
private:
    int num_particles_;
    double x_min_, x_max_, y_min_, y_max_;
    std::vector<Particle> particles_;
    std::default_random_engine gen_;
    std::map<int, Eigen::Vector2d> landmarks_;

public:
    ParticleFilter(int num_particles,
                   double x_min, double x_max,
                   double y_min, double y_max)
        : num_particles_(num_particles),
          x_min_(x_min), x_max_(x_max),
          y_min_(y_min), y_max_(y_max)
    {
        landmarks_[1] = Eigen::Vector2d(2.0, 1.0);
        landmarks_[2] = Eigen::Vector2d(4.0, 3.0);
        landmarks_[3] = Eigen::Vector2d(1.0, -2.0);
    }

    void initialize() {
        std::uniform_real_distribution<double> dist_x(x_min_, x_max_);
        std::uniform_real_distribution<double> dist_y(y_min_, y_max_);
        std::uniform_real_distribution<double> dist_theta(-M_PI, M_PI);

        particles_.clear();
        for (int i = 0; i < num_particles_; ++i) {
            Particle p;
            p.x = dist_x(gen_);
            p.y = dist_y(gen_);
            p.theta = dist_theta(gen_);
            p.weight = 1.0 / num_particles_;
            particles_.push_back(p);
        }

        ROS_INFO("Particle Filter initialized with %d particles", num_particles_);
    }

    void predict(double v, double omega, double dt) {
        std::normal_distribution<double> noise(0.0, 0.01);
        for (auto& p : particles_) {
            p.x += v * std::cos(p.theta) * dt + noise(gen_);
            p.y += v * std::sin(p.theta) * dt + noise(gen_);
            p.theta += omega * dt + noise(gen_);
        }
    }

    void updateWeights(const sensor_msgs::LaserScan::ConstPtr& scan_msg, double sensor_std) {
        double sigma2 = sensor_std * sensor_std;
        int center_idx = scan_msg->ranges.size() / 2;
        double measured_range = scan_msg->ranges[center_idx];

        if (!std::isfinite(measured_range)) return;

        for (auto& p : particles_) {
            double best_likelihood = 1e-5;

            for (const auto& [id, lm] : landmarks_) {
                double dx = lm(0) - p.x;
                double dy = lm(1) - p.y;
                double expected_range = std::sqrt(dx * dx + dy * dy);
                double angle_to_lm = std::atan2(dy, dx);
                double angle_diff = normalize_angle(angle_to_lm - p.theta);

                if (std::abs(angle_diff) > (scan_msg->angle_max - scan_msg->angle_min) / 2)
                    continue;

                double residual = measured_range - expected_range;
                double likelihood = std::exp(-0.5 * residual * residual / sigma2);

                if (likelihood > best_likelihood)
                    best_likelihood = likelihood;
            }

            p.weight = best_likelihood;
        }

        double sum_weights = 0.0;
        for (const auto& p : particles_) sum_weights += p.weight;
        for (auto& p : particles_) p.weight /= sum_weights;
    }

    void resample() {
        std::vector<Particle> new_particles;
        std::uniform_real_distribution<double> dist(0.0, 1.0);

        std::vector<double> cumulative;
        cumulative.push_back(particles_[0].weight);
        for (size_t i = 1; i < particles_.size(); ++i)
            cumulative.push_back(cumulative.back() + particles_[i].weight);

        for (int i = 0; i < num_particles_; ++i) {
            double r = dist(gen_);
            auto it = std::lower_bound(cumulative.begin(), cumulative.end(), r);
            int idx = std::distance(cumulative.begin(), it);
            new_particles.push_back(particles_[idx]);
            new_particles.back().weight = 1.0 / num_particles_;
        }

        particles_ = new_particles;
    }

    double estimateX() const {
        double sum = 0.0;
        for (const auto& p : particles_) sum += p.x * p.weight;
        return sum;
    }

    double estimateY() const {
        double sum = 0.0;
        for (const auto& p : particles_) sum += p.y * p.weight;
        return sum;
    }

    double estimateTheta() const {
        double sum_sin = 0.0, sum_cos = 0.0;
        for (const auto& p : particles_) {
            sum_sin += std::sin(p.theta) * p.weight;
            sum_cos += std::cos(p.theta) * p.weight;
        }
        return std::atan2(sum_sin, sum_cos);
    }

    const std::vector<Particle>& getParticles() const {
        return particles_;
    }

};

class ParticleFilterNode {
private:
    ros::NodeHandle nh_;
    ros::Subscriber cmd_sub_, scan_sub_;
    ros::Publisher pose_pub_;
    ros::Publisher marker_pub_;
    ParticleFilter pf_;
    geometry_msgs::Twist last_cmd_;
    ros::Time last_time_;

public:
    ParticleFilterNode(ros::NodeHandle& nh)
        : nh_(nh),
          pf_(nh.param("num_particles", 2000),
              nh.param("world_x_min", -10.0),
              nh.param("world_x_max", 10.0),
              nh.param("world_y_min", -10.0),
              nh.param("world_y_max", 10.0))
    {
        cmd_sub_ = nh_.subscribe("/cmd_vel", 10, &ParticleFilterNode::cmdCallback, this);
        scan_sub_ = nh_.subscribe("/scan", 10, &ParticleFilterNode::scanCallback, this);
        pose_pub_ = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/pose_pf", 10);
        marker_pub_ = nh_.advertise<visualization_msgs::Marker>("/particles_marker", 10);
        last_time_ = ros::Time(0);
        pf_.initialize();
    }

    void cmdCallback(const geometry_msgs::Twist::ConstPtr& msg) {
        last_cmd_ = *msg;
    }

    void scanCallback(const sensor_msgs::LaserScan::ConstPtr& msg) {
        ros::Time current_time = msg->header.stamp;
        if (last_time_.isZero()) {
            last_time_ = current_time;
            return;
        }


        double dt = (current_time - last_time_).toSec();
        last_time_ = current_time;
        if (dt <= 0.0) return;

        pf_.predict(last_cmd_.linear.x, last_cmd_.angular.z, dt);
        pf_.updateWeights(msg, 0.2);
        pf_.resample();

        geometry_msgs::PoseWithCovarianceStamped out;
        out.header.stamp = current_time;
        out.header.frame_id = "odom";
        out.pose.pose.position.x = pf_.estimateX();
        out.pose.pose.position.y = pf_.estimateY();

        tf2::Quaternion q;
        q.setRPY(0, 0, pf_.estimateTheta());
        out.pose.pose.orientation = tf2::toMsg(q);

        pose_pub_.publish(out);
        publishParticles(current_time);

    }

    void publishParticles(const ros::Time& stamp) {
        visualization_msgs::Marker marker;
        marker.header.stamp = stamp;
        marker.header.frame_id = "odom";
        marker.ns = "particles";
        marker.id = 0;
        marker.type = visualization_msgs::Marker::POINTS;
        marker.action = visualization_msgs::Marker::ADD;
        marker.scale.x = 0.05;
        marker.scale.y = 0.05;
        marker.color.r = 1.0;
        marker.color.g = 0.0;
        marker.color.b = 0.0;
        marker.color.a = 1.0;

        for (const auto& p : pf_.getParticles()) {
            geometry_msgs::Point pt;
            pt.x = p.x;
            pt.y = p.y;
            pt.z = 0.0;
            marker.points.push_back(pt);
        }

        marker_pub_.publish(marker);
    }


};

int main(int argc, char** argv) {
    ros::init(argc, argv, "filter_node_PF");
    ros::NodeHandle nh;
    ParticleFilterNode node(nh);
    ros::spin();
    return 0;
}
