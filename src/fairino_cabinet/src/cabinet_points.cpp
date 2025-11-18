#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

class cabinet_points : public rclcpp::Node
{
public:
    cabinet_points(): rclcpp::Node("cabinet_points_node")
    {
        // 柜子右下角原点
        this->declare_parameter<std::vector<double>>("cabinet_origin",{0.0,0.0,0.0});
        cabinet_origin = this->get_parameter("cabinet_origin").as_double_array();

        // 柜子数量
        this->declare_parameter("horizontal_cabinet_num",10);
        horizontal_cabinet_num = this->get_parameter("horizontal_cabinet_num").as_int();
        this->declare_parameter("vertical_cabinet_num",10);
        vertical_cabinet_num = this->get_parameter("vertical_cabinet_num").as_int();

        // 柜子的宽深高
        this->declare_parameter("cabinet_width",1.0);
        this->declare_parameter("cabinet_height",1.0);
        cabinet_width = this->get_parameter("cabinet_width").as_double();
        cabinet_height = this->get_parameter("cabinet_height").as_double();

        // 水平隔板
        this->declare_parameter("horizontal_partition_height",1.0);
        horizontal_partition_height = this->get_parameter("horizontal_partition_height").as_double();

        // 垂直隔板
        this->declare_parameter("vertical_partition_width",1.0);
        vertical_partition_width = this->get_parameter("vertical_partition_width").as_double();

        is_calculate = false;
        timer_ = this->create_wall_timer(std::chrono::seconds(2),std::bind(&cabinet_points::pubCabinetParam,this));
        cabinet_points_pub_ = this->create_publisher<std_msgs::msg::String>("cabinet/points",10);

        RCLCPP_INFO(this->get_logger(),"cabinet_points节点已启动");
    }

private:
    void pubCabinetParam()
    {
        if (!is_calculate)
        {
            cabinet_points_ = "{";
            for (int i=0;i<horizontal_cabinet_num;i++)
            {
                for (int j=0;j<vertical_cabinet_num;j++)
                {
                    cabinet_points_+="\"("+std::to_string(i)+", "+std::to_string(j)+")\": "+"["+std::to_string(cabinet_origin[0]+i*(cabinet_width+vertical_partition_width))+", "+std::to_string(cabinet_origin[1])+", "+std::to_string(cabinet_origin[2]+j*(cabinet_height+horizontal_partition_height))+"]";
                    if (i!=horizontal_cabinet_num-1 || j!=vertical_cabinet_num-1)
                        cabinet_points_+=",";
                }
            }
            cabinet_points_+="}";
            is_calculate=true;
        }
        std_msgs::msg::String str;
        str.data = cabinet_points_;
        cabinet_points_pub_->publish(str);
    }

    std::vector<double> cabinet_origin;
    int horizontal_cabinet_num;
    int vertical_cabinet_num;
    double cabinet_width;
    double cabinet_height;
    double horizontal_partition_height;
    double vertical_partition_width;
    std::string cabinet_points_;
    bool is_calculate;

    rclcpp::TimerBase::SharedPtr timer_;
    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr cabinet_points_pub_;
};


int main(int argc, char* argv[]) {
    rclcpp::init(argc, argv);
    auto node = std::make_shared<cabinet_points>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
