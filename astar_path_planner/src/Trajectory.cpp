#include <iostream>
#include <vector>
#include <Eigen/Dense>
#include <osqp/osqp.h>
#include <ros/ros.h>
#include <OsqpEigen/OsqpEigen.h>
#include <visualization_msgs/Marker.h>
#include <visualization_msgs/MarkerArray.h>

#include <ros/ros.h>
#include <visualization_msgs/Marker.h>
#include <Eigen/Dense>
#include <vector>
#include <nav_msgs/Path.h>


typedef std::vector<Eigen::Vector2d> Path;

class TrajectoryGenerator {
public:
    TrajectoryGenerator() = default;

    std::vector<Eigen::Vector2d> generateTrajectory(const Path& path, double maxVelocity, double maxAcceleration, double maxJerk, double dt) {
        std::vector<Eigen::Vector2d> trajectory;
        if (path.size() < 2) return trajectory;

        // 对路径点进行降采样，但保留关键转弯点
        Path simplified_path = simplifyPath(path, 0.3);  // 降低阈值以保留更多转弯点

        // 为每个路径段生成控制点
        for (size_t i = 0; i < simplified_path.size() - 1; ++i) {
            const Eigen::Vector2d& current = simplified_path[i];
            const Eigen::Vector2d& next = simplified_path[i + 1];
            
            // 计算控制点
            Eigen::Vector2d control_point1, control_point2;
            if (i == 0) {
                // 起点段
                control_point1 = current + (next - current) * 0.25;
                control_point2 = current + (next - current) * 0.75;
            } else if (i == simplified_path.size() - 2) {
                // 终点段
                control_point1 = current + (next - current) * 0.25;
                control_point2 = current + (next - current) * 0.75;
            } else {
                // 中间段，考虑前后点以生成更平滑的曲线
                Eigen::Vector2d prev = simplified_path[i - 1];
                Eigen::Vector2d next_next = simplified_path[i + 2];
                
                Eigen::Vector2d dir1 = (next - prev).normalized();
                Eigen::Vector2d dir2 = (next_next - current).normalized();
                
                double segment_length = (next - current).norm();
                control_point1 = current + dir1 * segment_length * 0.25;
                control_point2 = next - dir2 * segment_length * 0.25;
            }

            // 生成贝塞尔曲线段
            double segment_time = std::max((next - current).norm() / maxVelocity, 
                                         std::sqrt((next - current).norm() / maxAcceleration));
            
            std::vector<Eigen::Vector2d> segment = generateBezierSegment(
                current, control_point1, control_point2, next, segment_time, dt);
            
            trajectory.insert(trajectory.end(), segment.begin(), segment.end());
        }

        return trajectory;
    }

private:
    Path simplifyPath(const Path& path, double threshold) {
        if (path.size() <= 2) return path;
        
        Path simplified;
        simplified.push_back(path.front());
        
        for (size_t i = 1; i < path.size() - 1; ++i) {
            Eigen::Vector2d prev = path[i - 1];
            Eigen::Vector2d curr = path[i];
            Eigen::Vector2d next = path[i + 1];
            
            // 计算转弯角度
            Eigen::Vector2d v1 = (curr - prev).normalized();
            Eigen::Vector2d v2 = (next - curr).normalized();
            double angle = std::acos(v1.dot(v2));
            
            // 如果转弯角度大于阈值，保留该点
            if (angle > threshold) {
                simplified.push_back(curr);
            }
        }
        
        simplified.push_back(path.back());
        return simplified;
    }

    std::vector<Eigen::Vector2d> generateBezierSegment(
        const Eigen::Vector2d& p0,
        const Eigen::Vector2d& p1,
        const Eigen::Vector2d& p2,
        const Eigen::Vector2d& p3,
        double T,
        double dt) {
        
        std::vector<Eigen::Vector2d> segment;
        
        for (double t = 0; t <= T; t += dt) {
            double u = t / T;  // 归一化时间参数
            double u2 = u * u;
            double u3 = u2 * u;
            double mu = 1.0 - u;
            double mu2 = mu * mu;
            double mu3 = mu2 * mu;
            
            // 三次贝塞尔曲线插值
            Eigen::Vector2d pos = mu3 * p0 +
                                3.0 * mu2 * u * p1 +
                                3.0 * mu * u2 * p2 +
                                u3 * p3;
            
            segment.push_back(pos);
        }
        
        return segment;
    }
};

class TrajectoryVisualizer {
public:
    // TrajectoryVisualizer(const Path& trajectory) : trajectory_(trajectory) {
    //     ros::NodeHandle nh;
    //     marker_pub_ = nh.advertise<visualization_msgs::Marker>("visualization_marker", 10);
    // }
    TrajectoryVisualizer(ros::NodeHandle& nh){
        marker_pub_ = nh.advertise<visualization_msgs::Marker>("smooth_trajectory", 10);
    }

    void publishTrajectory(const Path& trajectory) {
        visualization_msgs::Marker line_strip;
        line_strip.header.frame_id = "map";
        line_strip.header.stamp = ros::Time::now();
        line_strip.ns = "smooth_trajectory";
        line_strip.action = visualization_msgs::Marker::ADD;
        line_strip.pose.orientation.w = 1.0;
        line_strip.id = 0;
        line_strip.type = visualization_msgs::Marker::LINE_STRIP;

        line_strip.scale.x = 0.02;  // 线宽
        line_strip.color.b = 1.0;  // 蓝色
        line_strip.color.a = 1.0;  // 不透明

        for (const auto& point : trajectory) {
            geometry_msgs::Point p;
            p.x = point(0);
            p.y = point(1);
            p.z = 0;
            line_strip.points.push_back(p);
        }

        marker_pub_.publish(line_strip);
    }

private:
    ros::Publisher marker_pub_;
    // Path trajectory_;
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "trajectory_generator");
    ros::NodeHandle nh;

    // 读取参数
    double maxVelocity, maxAcceleration, maxJerk, dt;
    nh.param("trajectory/max_velocity", maxVelocity, 0.5);
    nh.param("trajectory/max_acceleration", maxAcceleration, 0.3);
    nh.param("trajectory/max_jerk", maxJerk, 0.1);
    nh.param("trajectory/dt", dt, 0.05);

    // 创建轨迹生成器和可视化器
    TrajectoryGenerator generator;
    TrajectoryVisualizer visualizer(nh);

    // 订阅 A* 路径
    ros::Subscriber path_sub = nh.subscribe<nav_msgs::Path>("path", 1,
        [&](const nav_msgs::Path::ConstPtr& msg){
            // 转换路径点格式
            Path path;
            for (const auto& pose : msg->poses) {
                path.emplace_back(pose.pose.position.x, pose.pose.position.y);
            }

            if (!path.empty()) {
                // 生成平滑轨迹
                Path smooth_trajectory = generator.generateTrajectory(
                    path, maxVelocity, maxAcceleration, maxJerk, dt);
                
                // 发布可视化
                visualizer.publishTrajectory(smooth_trajectory);
            }
        });

    ros::spin();
    return 0;
}