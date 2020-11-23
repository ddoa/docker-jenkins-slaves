#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

#include <memory>

#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/string.hpp>

using std::placeholders::_1;

namespace gazebo
{
class ModelPushRos : public ModelPlugin {

    public: void Load(physics::ModelPtr _parent, sdf::ElementPtr _sdf/*_sdf*/)
        {
            printf("Hello Model!\n");
            std::cout << "hello model !" <<std::endl;

            // Create a GazeboRos node instead of a common ROS node.
            // Pass it SDF parameters so common options like namespace and remapping
            // can be handled.
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            // The model pointer gives you direct access to the physics object,
            // for example:
            RCLCPP_INFO(ros_node_->get_logger(), _parent->GetName().c_str());

            // Subscribe to messages
            ros_node_ = gazebo_ros::Node::Get(_sdf);

            subscription_ = ros_node_->create_subscription<std_msgs::msg::String>(
                    "subscriber", 10, std::bind(&ModelPushRos::topic_callback, this, std::placeholders::_1));




            // Store the pointer to the model
            this->model = _parent;


//            // Check that the velocity element exists, then read the value
//            if (_sdf->HasElement("velocity"))
//                velocity = _sdf->Get<double>("velocity");


        // Listen to the update event. This event is broadcast every
        // simulation iteration.
        this->updateConnection = event::Events::ConnectWorldUpdateBegin(
                std::bind(&ModelPushRos::OnUpdate, this));


        }

    // Called by the world update start event
public: void OnUpdate()
    {
        // Apply a small linear velocity to the model.
        this->model->SetLinearVel(ignition::math::Vector3d(velocity, 0, 0));
    }

    void topic_callback(std_msgs::msg::String::SharedPtr msg)
    {
        RCLCPP_INFO(ros_node_->get_logger(), "I heard: '%s'", msg->data.c_str());
        std::cout << msg->data.c_str() << std::endl;
        double input = std::stof(msg->data.c_str());
        setVelocity(input);
    }

        // Pointer to the model
    private: physics::ModelPtr model;

        // Pointer to the update event connection
    private: event::ConnectionPtr updateConnection;
    /// Node for ROS communication.
private:gazebo_ros::Node::SharedPtr ros_node_;
private:rclcpp::Subscription<std_msgs::msg::String>::SharedPtr subscription_;
    // Default to zero velocity
    double velocity = 0;
private: void setVelocity(double aValue){
    velocity = aValue;
}
    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(ModelPushRos)
}
