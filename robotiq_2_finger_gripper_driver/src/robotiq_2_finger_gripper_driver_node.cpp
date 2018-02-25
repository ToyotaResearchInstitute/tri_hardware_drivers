#include <stdlib.h>
#include <stdio.h>
#include <memory>
#include <robotiq_2_finger_gripper_driver/robotiq_2_finger_gripper_driver.hpp>
// ROS
#include <ros/ros.h>
#include <robotiq_2_finger_gripper_driver/Robotiq2FingerCommand.h>
#include <robotiq_2_finger_gripper_driver/Robotiq2FingerStatus.h>
#include <ros/xmlrpc_manager.h>
#include <signal.h>

namespace robotiq_2_finger_gripper_driver
{
    class Robotiq2FingerDriver
    {
    protected:

        ros::NodeHandle nh_;
        ros::Publisher status_pub_;
        ros::Subscriber command_sub_;
        std::unique_ptr<Robotiq2FingerGripperModbusRtuInterface> gripper_interface_ptr_;

    public:

        Robotiq2FingerDriver(const ros::NodeHandle& nh, const std::string& status_topic, const std::string& command_topic, const std::string& modbus_rtu_interface, const uint16_t sensor_slave_id) : nh_(nh)
        {
            status_pub_ = nh_.advertise<Robotiq2FingerStatus>(status_topic, 1, false);
            command_sub_ = nh_.subscribe(command_topic, 1, &Robotiq2FingerDriver::CommandCB, this);
            // Make the logging function
            std::function<void(const std::string&)> logging_fn = [] (const std::string& message) { ROS_INFO_NAMED(ros::this_node::getName(), "%s", message.c_str()); };
            ROS_INFO_NAMED(ros::this_node::getName(), "Connecting to Robotiq 2-Finger gripper with sensor slave ID %hx on Modbus RTU interface %s...", sensor_slave_id, modbus_rtu_interface.c_str());
            gripper_interface_ptr_ = std::unique_ptr<Robotiq2FingerGripperModbusRtuInterface>(new Robotiq2FingerGripperModbusRtuInterface(logging_fn, modbus_rtu_interface, sensor_slave_id));
            const bool success = gripper_interface_ptr_->ActivateGripper();
            if (!success)
            {
                throw std::runtime_error("Unable to initialize gripper");
            }
        }

        void Shutdown()
        {
            gripper_interface_ptr_.reset();
            ros::requestShutdown();
        }

        void Loop(const double control_rate)
        {
            ROS_INFO_NAMED(ros::this_node::getName(), "Gripper interface running");;
            ros::Rate rate(control_rate);
            while (ros::ok())
            {
                PublishGripperStatus();
                ros::spinOnce();
                rate.sleep();
            }
            ROS_INFO_NAMED(ros::this_node::getName(), "Gripper interface shutting down");
        }

    protected:

        void CommandCB(Robotiq2FingerCommand command_msg)
        {
            const Robotiq2FingerGripperCommand gripper_command(command_msg.percent_closed, command_msg.percent_speed, command_msg.percent_effort);
            const bool sent = gripper_interface_ptr_->SendGripperCommand(gripper_command);
            if (!sent)
            {
                ROS_ERROR_NAMED(ros::this_node::getName(), "Failed to send command to gripper");
            }
        }

        void PublishGripperStatus()
        {
            const Robotiq2FingerGripperStatus status = gripper_interface_ptr_->GetGripperStatus();
            Robotiq2FingerStatus status_msg;
            status_msg.actual_percent_closed = status.ActualPosition();
            status_msg.actual_percent_current = status.ActualCurrent();
            status_msg.target_percent_closed = status.TargetPosition();
            status_msg.header.stamp = ros::Time::now();
            status_pub_.publish(status_msg);
        }
    };
}

std::unique_ptr<robotiq_2_finger_gripper_driver::Robotiq2FingerDriver> g_gripper;

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
    const double DEFAULT_POLL_RATE = 10.0;
    const std::string DEFAULT_STATUS_TOPIC("robotiq_2_finger_status");
    const std::string DEFAULT_COMMAND_TOPIC("robotiq_2_finger_command");
    const std::string DEFAULT_MODBUS_RTU_INTERFACE("/dev/ttyUSB0");
    const int32_t DEFAULT_GRIPPER_SLAVE_ID = 0x09;
    // Start ROS
    ros::init(argc, argv, "robotiq_2_finger_driver", ros::init_options::NoSigintHandler);
    signal(SIGINT, SigIntHandler);
    ros::NodeHandle nh;
    ros::NodeHandle nhp("~");
    // Override XMLRPC shutdown
    ros::XMLRPCManager::instance()->unbind("shutdown");
    ros::XMLRPCManager::instance()->bind("shutdown", XmlRpcShutdownCB);
    // Get params
    const std::string modbus_rtu_interface = nhp.param(std::string("modbus_rtu_interface"), DEFAULT_MODBUS_RTU_INTERFACE);
    const uint16_t gripper_slave_id = (uint16_t)nhp.param(std::string("gripper_slave_id"), DEFAULT_GRIPPER_SLAVE_ID);
    const double poll_rate = std::abs(nhp.param(std::string("poll_rate"), DEFAULT_POLL_RATE));
    const std::string status_topic = nhp.param(std::string("status_topic"), DEFAULT_STATUS_TOPIC);
    const std::string command_topic = nhp.param(std::string("command_topic"), DEFAULT_COMMAND_TOPIC);
    // Start the driver
    g_gripper = std::unique_ptr<robotiq_2_finger_gripper_driver::Robotiq2FingerDriver>(new robotiq_2_finger_gripper_driver::Robotiq2FingerDriver(nh, status_topic, command_topic, modbus_rtu_interface, gripper_slave_id));
    g_gripper->Loop(poll_rate);
    return 0;
}
