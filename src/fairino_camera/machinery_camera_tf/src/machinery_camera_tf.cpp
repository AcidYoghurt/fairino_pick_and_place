#include <fstream>
#include <yaml-cpp/yaml.h>
#include <rclcpp/rclcpp.hpp>
#include <geometry_msgs/msg/transform_stamped.hpp>
#include <tf2_ros/static_transform_broadcaster.h>
#include <ament_index_cpp/get_package_share_directory.hpp>

class MachineryCameraTF : public rclcpp::Node
{
public:
    MachineryCameraTF() : Node("machinery_camera_tf")
    {
        // 声明该节点所需要的参数
        this->declare_parameter("config_file_name", "eye_in_hand_calibrate.calib");
        this->get_parameter("config_file_name", config_file_name);
        std::string package_share_directory = ament_index_cpp::get_package_share_directory("machinery_camera_tf");

        // 打开 yaml文件
        std::string yaml_file_path = package_share_directory + "/config/"+config_file_name;
        std::ifstream file(yaml_file_path);
        if (file.is_open())
        {
            YAML::Node config = YAML::LoadFile(yaml_file_path);
            // 加载 parameters 组下的参数
            calibration_type = config["parameters"]["calibration_type"].as<std::string>();
            robot_base_frame = config["parameters"]["robot_base_frame"].as<std::string>();
            robot_effector_frame = config["parameters"]["robot_effector_frame"].as<std::string>();
            tracking_base_frame = config["parameters"]["tracking_base_frame"].as<std::string>();

            // 加载 transform 组下的参数
            trans_x = config["transform"]["translation"]["x"].as<double>();
            trans_y = config["transform"]["translation"]["y"].as<double>();
            trans_z = config["transform"]["translation"]["z"].as<double>();

            rot_x = config["transform"]["rotation"]["x"].as<double>();
            rot_y = config["transform"]["rotation"]["y"].as<double>();
            rot_z = config["transform"]["rotation"]["z"].as<double>();
            rot_w = config["transform"]["rotation"]["w"].as<double>();

            file.close();
        }
        else
        {
            RCLCPP_ERROR(this->get_logger(), "无法打开 YAML 文件: %s", yaml_file_path.c_str());
            return;
        }

        tf_broadcaster_ = std::make_shared<tf2_ros::StaticTransformBroadcaster>(this);
        this->publish_static_transform();
    }

private:
    void publish_static_transform()
    {
        if (calibration_type=="eye_in_hand")
        {
            geometry_msgs::msg::TransformStamped t;

            // 父坐标系 和 子坐标系
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = robot_effector_frame;
            t.child_frame_id = tracking_base_frame;

            // 平移
            t.transform.translation.x = trans_x;
            t.transform.translation.y = trans_y;
            t.transform.translation.z = trans_z;

            // 旋转
            t.transform.rotation.x = rot_x;
            t.transform.rotation.y = rot_y;
            t.transform.rotation.z = rot_z;
            t.transform.rotation.w = rot_w;

            // 发布tf
            RCLCPP_INFO(this->get_logger(),"当前是 %s ，正在发布 %s 到 %s 的静态tf",calibration_type.c_str(),robot_effector_frame.c_str(),tracking_base_frame.c_str());
            tf_broadcaster_->sendTransform(t);
        }
        else if (calibration_type=="eye_on_base")
        {
            geometry_msgs::msg::TransformStamped t;

            // 父坐标系 和 子坐标系
            t.header.stamp = this->get_clock()->now();
            t.header.frame_id = robot_base_frame;
            t.child_frame_id = tracking_base_frame;

            // 平移
            t.transform.translation.x = trans_x;
            t.transform.translation.y = trans_y;
            t.transform.translation.z = trans_z;

            // 旋转
            t.transform.rotation.x = rot_x;
            t.transform.rotation.y = rot_y;
            t.transform.rotation.z = rot_z;
            t.transform.rotation.w = rot_w;

            // 发布tf
            RCLCPP_INFO(this->get_logger(),"当前是 %s ，正在发布 %s 到 %s 的静态tf",calibration_type.c_str(),robot_base_frame.c_str(),tracking_base_frame.c_str());
            tf_broadcaster_->sendTransform(t);
        }
        else
            RCLCPP_ERROR(this->get_logger(),"calibration_type 参数违法");
    }

    // 所需参数
    std::string calibration_type,config_file_name,robot_effector_frame,tracking_base_frame,robot_base_frame;
    double trans_x, trans_y, trans_z,rot_x, rot_y, rot_z, rot_w;

    std::shared_ptr<tf2_ros::StaticTransformBroadcaster> tf_broadcaster_;
};


int main(int argc, char * argv[])
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<MachineryCameraTF>());
    rclcpp::shutdown();
    return 0;
}