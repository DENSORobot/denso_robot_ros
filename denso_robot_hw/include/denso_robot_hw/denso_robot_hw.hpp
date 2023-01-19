#ifndef DENSO_ROBOT_HW_H__
#define DENSO_ROBOT_HW_H__

// std_h
#include <cstdint>
#include <memory>
#include <string>
#include <vector>
#include <bitset>

// hardware_interface
#include <hardware_interface/handle.hpp>
#include <hardware_interface/hardware_info.hpp>
#include <hardware_interface/system_interface.hpp>
#include <hardware_interface/types/hardware_interface_return_values.hpp>

// rclcpp
#include <rclcpp/macros.hpp>
#include <rclcpp_lifecycle/node_interfaces/lifecycle_node_interface.hpp>
#include <rclcpp_lifecycle/state.hpp>

#include "denso_robot_hw/visibility_control.h"

// denso
#include "denso_robot_core/denso_robot_core.hpp"

namespace denso2
{

class denso_hw : public hardware_interface::SystemInterface
{
public:
    RCLCPP_SHARED_PTR_DEFINITIONS(denso_hw)

    DENSO_HW_PUBLIC
    hardware_interface::CallbackReturn on_init(
    const hardware_interface::HardwareInfo & info) override;
    
    DENSO_HW_PUBLIC
    hardware_interface::CallbackReturn on_configure(
    const rclcpp_lifecycle::State & previous_state) override;

    DENSO_HW_PUBLIC
    std::vector<hardware_interface::StateInterface> export_state_interfaces() override;

    DENSO_HW_PUBLIC
    std::vector<hardware_interface::CommandInterface> export_command_interfaces() override;

    DENSO_HW_PUBLIC
    hardware_interface::CallbackReturn on_activate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    DENSO_HW_PUBLIC
    hardware_interface::CallbackReturn on_deactivate(
        const rclcpp_lifecycle::State & previous_state) override;
    
    DENSO_HW_PUBLIC
    hardware_interface::return_type read(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;
    
    DENSO_HW_PUBLIC
    hardware_interface::return_type write(
        const rclcpp::Time & time, const rclcpp::Duration & period) override;

private:
    /* Functions */
    HRESULT ChangeModeWithClearError(int mode);
    HRESULT CheckRobotType();
    bool denso_init();
    bool hasError();
    void printErrorDescription(HRESULT error_code, const std::string& error_message);

    /* Variables */
    // Denso 
    DensoRobotCore_Ptr denso_core_;
    DensoController_Ptr denso_ctrl_;
    DensoRobot_Ptr denso_robot_;
    DensoVariable_Ptr denso_var_;

    std::vector<int> joint_type_;

    int sendfmt_;
    int recvfmt_;

    std::vector<int> type_;
    std::string name_;
    
    boost::mutex mtxMode_;

    // hardware_interface
    std::vector<double> hw_commands_positions_;
    std::vector<double> hw_positions_;
    std::vector<double> hw_velocities_;
    std::vector<double> hw_efforts_;
    std::vector<double> hw_joints_; // temp read for denso robot

    enum integration_level_t : std::uint8_t
    {
        UNDEFINED = 0,
        POSITION,
        VELOCITY,
        EFFORT
    };
    
    enum dig_gpio : std::uint8_t
    {
        GPIO_0 = 0,
        GPIO_1,
        GPIO_2,
        NUM_GPIO_ALL
    };

    std::bitset<NUM_GPIO_ALL> actual_dig_state_bits_;
    std::bitset<NUM_GPIO_ALL> actual_dig_cmd_bits_;

    std::vector<double> dig_state_gpio_; // for controller
    std::vector<double> dig_cmd_gpio_;

    // active control mode for each actuator
    std::vector<integration_level_t> control_level_;

    // denso state
    std::string ip_;
    int port_;

    size_t num_joints_;
};
    
} // namespace denso2




#endif