#include <stdlib.h>
#include <stdio.h>
#include <schunk_wsg_driver/schunk_wsg_driver_common.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_ethernet.hpp>
#include <schunk_wsg_driver/schunk_wsg_driver_can.hpp>
#include <schunk_wsg_driver/WSGCommand.h>
#include <schunk_wsg_driver/WSGStatus.h>
// ROS
#include <ros/ros.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace schunk_wsg_driver
{
    class SchunkWSGDriver
    {
    protected:

        ros::NodeHandle nh_;
        ros::Subscriber command_sub_;
        ros::Publisher status_pub_;

        std::shared_ptr<WSGInterface> gripper_interface_ptr_;

    public:

        SchunkWSGDriver(const ros::NodeHandle& nh, const std::shared_ptr<WSGInterface>& gripper_interface, const std::string& command_topic, const std::string& status_topic) : nh_(nh), gripper_interface_ptr_(gripper_interface)
        {
            status_pub_ = nh_.advertise<WSGStatus>(status_topic, 1, false);
            command_sub_ = nh_.subscribe(command_topic, 1, &SchunkWSGDriver::CommandCB, this);
            const bool success = gripper_interface_ptr_->InitializeGripper();
            if (!success)
            {
                throw std::invalid_argument("Unable to initialize gripper");
            }
        }

        void Shutdown()
        {
            gripper_interface_ptr_->Shutdown();
            ros::requestShutdown();
        }

        void Loop(const double control_rate)
        {
            gripper_interface_ptr_->Log("Gripper interface running");;
            ros::Rate rate(control_rate);
            while (ros::ok())
            {
                PublishGripperStatus();
                ros::spinOnce();
                rate.sleep();
            }
            gripper_interface_ptr_->Log("Gripper interface shutting down");
        }

    protected:

        void CommandCB(WSGCommand command)
        {
            const double target_position = std::abs(command.target_position);
            const double max_effort = std::abs(command.max_effort);
            const bool sent = gripper_interface_ptr_->SetTargetPositionAndEffort(target_position, max_effort);
            if (!sent)
            {
                ROS_ERROR_NAMED(ros::this_node::getName(), "Failed to send command to gripper");
            }
        }

        void PublishGripperStatus()
        {
            gripper_interface_ptr_->RefreshGripperStatus();
            const GripperMotionStatus status = gripper_interface_ptr_->GetGripperStatus();
            WSGStatus status_msg;
            status_msg.actual_position = status.ActualPosition();
            status_msg.actual_velocity = status.ActualVelocity();
            status_msg.actual_effort = status.ActualEffort();
            status_msg.target_position = status.TargetPosition();
            status_msg.max_effort = status.MaxEffort();
            status_msg.header.stamp = ros::Time::now();
            status_pub_.publish(status_msg);
        }
    };
}

std::unique_ptr<schunk_wsg_driver::SchunkWSGDriver> g_gripper;

void SigIntHandler(int signal)
{
    (void)(signal);
    g_gripper->Shutdown();
}

void XmlRpcShutdownCB(XmlRpc::XmlRpcValue& params, XmlRpc::XmlRpcValue& result)
{
    const int num_params = (params.getType() == XmlRpc::XmlRpcValue::TypeArray) ? params.size() : 0;
    if (num_params > 1)
    {
        const std::string& reason = params[1];
        ROS_WARN("Shutdown request received. Reason: [%s]", reason.c_str());
        g_gripper->Shutdown();
    }
    result = ros::xmlrpc::responseInt(1, "", 0);
}

int main(int argc, char** argv)
{
    // Default ROS params
    const std::string DEFAULT_INTERFACE_TYPE("udp");
    const double DEFAULT_CONTROL_RATE = 10.0;
    const std::string DEFAULT_COMMAND_TOPIC("gripper_command");
    const std::string DEFAULT_STATUS_TOPIC("gripper_status");
    const std::string DEFAULT_GRIPPER_IP_ADDRESS("172.31.1.121");
    const int32_t DEFAULT_GRIPPER_PORT = 1500;
    const int32_t DEFAULT_LOCAL_PORT = 1501;
    const std::string DEFAULT_SOCKETCAN_INTERFACE("can0");
    const int32_t DEFAULT_GRIPPER_BASE_CAN_ID = 0x000;
    // Start ROS
    ros::init(argc, argv, "schunk_wsg_driver", ros::init_options::NoSigintHandler);
    signal(SIGINT, SigIntHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", XmlRpcShutdownCB);
    // Get params
    const std::string interface_type = nhp.param(std::string("interface_type"), DEFAULT_INTERFACE_TYPE);
    const double control_rate = std::abs(nhp.param(std::string("control_rate"), DEFAULT_CONTROL_RATE));
    const std::string command_topic = nhp.param(std::string("command_topic"), DEFAULT_COMMAND_TOPIC);
    const std::string status_topic = nhp.param(std::string("status_topic"), DEFAULT_STATUS_TOPIC);
    // Make the logging function
    std::function<void(const std::string&)> logging_fn = [] (const std::string& message) { ROS_INFO_NAMED(ros::this_node::getName(), "%s", message.c_str()); };
    if (interface_type == "udp")
    {
        const std::string gripper_ip_address = nhp.param(std::string("gripper_ip_address"), DEFAULT_GRIPPER_IP_ADDRESS);
        const uint16_t gripper_port = (uint16_t)nhp.param(std::string("gripper_port"), DEFAULT_GRIPPER_PORT);
        const uint16_t local_port = (uint16_t)nhp.param(std::string("local_port"), DEFAULT_LOCAL_PORT);
        std::shared_ptr<schunk_wsg_driver::WSGUDPInterface> gripper_interface(new schunk_wsg_driver::WSGUDPInterface(logging_fn, gripper_ip_address, gripper_port, local_port));
        g_gripper = std::unique_ptr<schunk_wsg_driver::SchunkWSGDriver>(new schunk_wsg_driver::SchunkWSGDriver(nh, gripper_interface, command_topic, status_topic));
        g_gripper->Loop(control_rate);
    }
    else if (interface_type == "can")
    {
        const std::string can_interface = nhp.param(std::string("socketcan_interface"), DEFAULT_SOCKETCAN_INTERFACE);
        const uint32_t gripper_send_can_id = (uint32_t)nhp.param(std::string("gripper_base_can_id"), DEFAULT_GRIPPER_BASE_CAN_ID);
        std::shared_ptr<schunk_wsg_driver::WSGCANInterface> gripper_interface(new schunk_wsg_driver::WSGCANInterface(logging_fn, can_interface, gripper_send_can_id));
        g_gripper = std::unique_ptr<schunk_wsg_driver::SchunkWSGDriver>(new schunk_wsg_driver::SchunkWSGDriver(nh, gripper_interface, command_topic, status_topic));
        g_gripper->Loop(control_rate);
    }
    else
    {
        ROS_FATAL_NAMED(ros::this_node::getName(), "Invalid interface option [%s], valid options are [udp] or [can]", interface_type.c_str());
    }
    return 0;
}
