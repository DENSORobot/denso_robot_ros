#include "denso_robot_hw/denso_robot_hw.hpp"

#include <chrono>
#include <cmath>
#include <limits>
#include <memory>
#include <string>
#include <vector>

#include <hardware_interface/types/hardware_interface_type_values.hpp>
#include <rclcpp/rclcpp.hpp>

#include "denso_robot_core/denso_robot_core.hpp"
#include "denso_robot_core/denso_controller.hpp"
#include "denso_robot_core/denso_robot.hpp"
#include "denso_robot_core/denso_variable.hpp"

#define RAD2DEG(x) ((x)*180.0 / M_PI)
#define DEG2RAD(x) ((x) / 180.0 * M_PI)

#define M2MM(x) ((x)*1000.0)
#define MM2M(x) ((x) / 1000.0)

namespace denso2
{

/*
 * @brief: Hardware Interface Initial
*/
hardware_interface::CallbackReturn denso_hw::on_init(
    const hardware_interface::HardwareInfo & info)
{
    if (
        hardware_interface::SystemInterface::on_init(info) !=
        hardware_interface::CallbackReturn::SUCCESS)
    {
        return hardware_interface::CallbackReturn::ERROR;
    }
    num_joints_ = info_.joints.size();
    hw_positions_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
    hw_velocities_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
    hw_efforts_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
    hw_commands_positions_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
    hw_joints_.resize(num_joints_, std::numeric_limits<double>::quiet_NaN());
    control_level_.resize(num_joints_, integration_level_t::POSITION);
    dig_state_gpio_.resize(NUM_GPIO_ALL);
    dig_cmd_gpio_.resize(NUM_GPIO_ALL);
    joint_type_.resize(num_joints_);

    sendfmt_ = DensoRobot::SENDFMT_MINIIO | DensoRobot::SENDFMT_HANDIO;
    recvfmt_ = DensoRobot::RECVFMT_POSE_PJ | DensoRobot::RECVFMT_MINIIO | DensoRobot::RECVFMT_HANDIO;

    for (const hardware_interface::ComponentInfo & joint : info_.joints)
    {
        if (joint.command_interfaces.size() != 1)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(this->get_name()),
                "joint '%s' has %zu command interfaces found. 1 expected.", joint.name.c_str(),
                joint.command_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(!(joint.command_interfaces[0].name == hardware_interface::HW_IF_POSITION))
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(this->get_name()),
                "Joint '%s' has %s command interface. Expected %s.", joint.name.c_str(),
                joint.command_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION);
            return hardware_interface::CallbackReturn::ERROR;
        }
        
        if (joint.state_interfaces.size() != 3)
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(this->get_name()),
                "joint '%s' has %zu state interfaces found. 3 expected.", joint.name.c_str(),
                joint.state_interfaces.size());
            return hardware_interface::CallbackReturn::ERROR;
        }

        if(!(joint.state_interfaces[0].name == hardware_interface::HW_IF_POSITION ||
             joint.state_interfaces[0].name == hardware_interface::HW_IF_VELOCITY ||
             joint.state_interfaces[0].name == hardware_interface::HW_IF_EFFORT))
        {
            RCLCPP_FATAL(
                rclcpp::get_logger(this->get_name()),
                "Joint '%s' has %s state interface. Expected %s , %s or %s.", joint.name.c_str(),
                joint.state_interfaces[0].name.c_str(), hardware_interface::HW_IF_POSITION,
                hardware_interface::HW_IF_VELOCITY,
                hardware_interface::HW_IF_EFFORT);
            return hardware_interface::CallbackReturn::ERROR;
        }
    }
    RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()), "Configured over");
    return hardware_interface::CallbackReturn::SUCCESS;    
}

bool denso_hw::denso_init()
{
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso Initializing...");
    ip_ = info_.hardware_parameters["robot_ip"];
    port_ = stoi(info_.hardware_parameters["robot_port"]);
    sendfmt_ = stoi(info_.hardware_parameters["send_format"]);
    recvfmt_ = stoi(info_.hardware_parameters["recv_format"]);
    int ctrl_type = stoi(info_.hardware_parameters["controller_type"]);
    std::string config_file = info_.hardware_parameters["config_file"];
    name_ = "";

    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso joints size is %zx, transmission size is %zx", num_joints_, info_.transmissions[0].joints.size());
    for (size_t i = 0; i < num_joints_; i++)
    {
        std::string joint_type = info_.transmissions[0].joints[i].role;
        joint_type_[i] = joint_type == "revolute" ? 1 : joint_type == "prismatic"? 0 : -1;
    }
    
    int armGroup = 0; // default to 0 -- cdy 2022.11.09 TODO

    denso_core_.reset(new DensoRobotCore());
    denso_ctrl_.reset();
    denso_robot_.reset();
    denso_var_.reset();
    HRESULT hr = denso_core_->Initialize(ctrl_type, ip_, port_, config_file);
    if (FAILED(hr))
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),"Failed to connect real controller. (%X)", hr);
        return false;
    }

    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso hw Initializing...  1");
    // Get the controller ptr
    denso_ctrl_ = denso_core_->get_Controller();

    // Get the Robot ptr
    DensoRobot_Ptr pRob;
    hr = denso_ctrl_->get_Robot(DensoBase::SRV_ACT, &pRob);
    if (FAILED(hr))
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),"Failed to connect real robot. (%X)", hr);
        return false;
    }
    denso_robot_ = pRob;
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso hw Initializing...  2");

    hr = CheckRobotType();
    if (FAILED(hr))
    {
        RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),"Invaild Robot Type.");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso hw Initializing...  3");

    denso_robot_->ChangeArmGroup(armGroup);

    hr = denso_ctrl_->AddVariable("@ERROR_CODE");
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to add @ERROR_CODE object");
        return false;
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso hw Initializing...  4");

    hr = denso_ctrl_->get_Variable("@ERROR_CODE", &denso_var_);
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to get @ERROR_CODE object");
        return false;
    }

    hr = denso_ctrl_->ExecClearError();
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to clear error");
        return false;
    }

    hr = denso_ctrl_->ExecResetStoState();
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to reset STO");
        return false;
    }

    hr = denso_robot_->ExecCurJnt(hw_joints_);
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to get current joint");
        return false;
    }

    for (size_t i = 0; i < num_joints_; i++)
    {
        switch (joint_type_[i])
        {
        case 0:  // prismatic
        {
            hw_positions_[i] = MM2M(hw_joints_[i]);
            RCLCPP_INFO(rclcpp::get_logger(this->get_name()),"The joint%zx is in the prismatic type", i);
            break;
        }
        case 1:  // revolute
        {
            hw_positions_[i] = DEG2RAD(hw_joints_[i]);
            RCLCPP_INFO(rclcpp::get_logger(this->get_name()),"The joint%zx is in the rotation type", i);
            break;
        }
        case -1:  // fixed
        default:
            hw_positions_[i] = 0.0;
            break;
        }
        hw_commands_positions_[i] = hw_positions_[i];
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Denso initialized successfully!");

    return true;
}

/*
 * @brief: Robot communication initial
*/
hardware_interface::CallbackReturn denso_hw::on_configure(
    const rclcpp_lifecycle::State & previous_state) 
{
    RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()), "Configuring ...please wait...");
    if(!denso_init()){
        RCLCPP_FATAL(
            rclcpp::get_logger(this->get_name()), "Configuration Failure! Please check your connection.");
        return hardware_interface::CallbackReturn::ERROR;
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Successfully configured!");

    return hardware_interface::CallbackReturn::SUCCESS;
}


std::vector<hardware_interface::StateInterface>
denso_hw::export_state_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "STATE interface Initializing...");
    std::vector<hardware_interface::StateInterface> state_interfaces;
    for (size_t i = 0; i < info_.joints.size(); i++)
    {
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_positions_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_VELOCITY, &hw_velocities_[i]));
        state_interfaces.emplace_back(hardware_interface::StateInterface(
            info_.joints[i].name, hardware_interface::HW_IF_EFFORT, &hw_efforts_[i]));
    }
    for (size_t i = 0; i < info_.gpios.size() ; i++)
    {
        if (info_.gpios[i].state_interfaces.size()!=0)
        {
            state_interfaces.emplace_back(hardware_interface::StateInterface(
                info_.gpios[i].name, hardware_interface::HW_IF_POSITION, &dig_state_gpio_[i]
            ));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "STATE interface initialized!");
    return state_interfaces;
}

std::vector<hardware_interface::CommandInterface>
denso_hw::export_command_interfaces()
{
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "COMMAND interface Initializing...");
    std::vector<hardware_interface::CommandInterface> command_interfaces;
    for(size_t i = 0; i< info_.joints.size(); i++)
    {
        command_interfaces.emplace_back(hardware_interface::CommandInterface(
            info_.joints[i].name, hardware_interface::HW_IF_POSITION, &hw_commands_positions_[i]));
    }
    for (size_t i = 0; i < info_.gpios.size() ; i++)
    {
        if (info_.gpios[i].command_interfaces.size()!=0)
        {
            command_interfaces.emplace_back(hardware_interface::CommandInterface(
                info_.gpios[i].name, hardware_interface::HW_IF_POSITION, &dig_cmd_gpio_[i]
            ));
        }
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "COMMAND interface initialized!");
    return command_interfaces;
}

/*
 * @brief: Robot Power on!
*/
hardware_interface::CallbackReturn denso_hw::on_activate(
  const rclcpp_lifecycle::State & /*previous_state*/)
{
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Starting... please wait...");

    // state init
    HRESULT hr;
    hr = denso_robot_->AddVariable("@SERVO_ON");
    if (SUCCEEDED(hr))
    {
        DensoVariable_Ptr pVar;
        hr = denso_robot_->get_Variable("@SERVO_ON", &pVar);

        if (SUCCEEDED(hr))
        {
            VARIANT_Ptr vntVal(new VARIANT());
            vntVal->vt = VT_BOOL;
            vntVal->boolVal = VARIANT_TRUE;
            hr = pVar->ExecPutValue(vntVal);
        }
    }
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to motor on");
        RCLCPP_FATAL(
            rclcpp::get_logger(this->get_name()), "Can not active motor!");
        return hardware_interface::CallbackReturn::ERROR;
    }

    int recvfmt_temp = recvfmt_; // check the format
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Chaning the robot mode... 1");
    switch (recvfmt_ & DensoRobot::RECVFMT_POSE)
    {
        case DensoRobot::RECVFMT_POSE_J:
        case DensoRobot::RECVFMT_POSE_PJ:
        case DensoRobot::RECVFMT_POSE_TJ:
            break;
        case DensoRobot::RECVFMT_POSE_P:
            recvfmt_temp = DensoRobot::RECVFMT_POSE_PJ;
            break;
        case DensoRobot::RECVFMT_POSE_T:
            recvfmt_temp = DensoRobot::RECVFMT_POSE_TJ;
            break;
        default:
            recvfmt_temp = DensoRobot::RECVFMT_POSE_J;
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Chaning the robot mode... 2");
    if (recvfmt_temp != recvfmt_)
    {
        RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Chaning the robot mode... 3");
        recvfmt_ = ((recvfmt_ & ~DensoRobot::RECVFMT_POSE) | recvfmt_temp);
        RCLCPP_WARN(rclcpp::get_logger(this->get_name()), "Changed recv_format to %d to contain joint",recvfmt_);
    }
    RCLCPP_INFO(rclcpp::get_logger(this->get_name()), "Chaning the robot mode... 4");
    denso_robot_->set_SendFormat(sendfmt_);
    denso_robot_->set_RecvFormat(recvfmt_);

    hr = ChangeModeWithClearError(DensoRobot::SLVMODE_SYNC_WAIT | DensoRobot::SLVMODE_POSE_J);
    if (FAILED(hr))
    {
        printErrorDescription(hr, "Failed to change to slave mode");
        RCLCPP_FATAL(
            rclcpp::get_logger(this->get_name()), "Failed to change to slave mode");
        return hardware_interface::CallbackReturn::ERROR;
    }
    
    RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()), "System successfully started! %u",
        control_level_[0]);
    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
 * @brief: Robot Power off!
*/
hardware_interface::CallbackReturn denso_hw::on_deactivate(
        const rclcpp_lifecycle::State & previous_state)
{
    RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()), "Stopping... please wait...");
    
    // stopping system
    RCLCPP_INFO(
        rclcpp::get_logger(this->get_name()), "System successfully stopped!");
    return hardware_interface::CallbackReturn::SUCCESS;
}

/*
 * @brief: Read!
*/
hardware_interface::return_type denso_hw::read(
        const rclcpp::Time & time, const rclcpp::Duration & period)
{
    boost::mutex::scoped_lock lockMode(mtxMode_);
    if (denso_core_->get_Mode() == DensoRobot::SLVMODE_NONE)
    {
        HRESULT hr = denso_robot_->ExecCurJnt(hw_joints_);
        if (FAILED(hr))
        {
            RCLCPP_ERROR(rclcpp::get_logger(this->get_name()), "Failed to get current joint. (%X)", hr);
        }
    }
    for (size_t i = 0; i < num_joints_; i++)
    {
        switch (joint_type_[i])
        {
        case 0:  // prismatic
        {
            hw_positions_[i] = MM2M(hw_joints_[i]);
            break;
        }
        case 1:  // revolute
        {
            hw_positions_[i] = DEG2RAD(hw_joints_[i]);
            break;
        }
        case -1:  // fixed
        default:
            hw_positions_[i] = 0.0;
            break;
        }
        hw_commands_positions_[i] = hw_positions_[i];
    }
    return hardware_interface::return_type::OK;
}

hardware_interface::return_type denso_hw::write(
        const rclcpp::Time & time, const rclcpp::Duration & period)
{
    boost::mutex::scoped_lock lockMode(mtxMode_);

    if (denso_core_->get_Mode() != DensoRobot::SLVMODE_NONE)
    {
        std::vector<double> pose;
        pose.resize(num_joints_);
        int bits = 0x0000;
        for (size_t i = 0; i < num_joints_; i++)
        {
            switch (joint_type_[i])
            {
            case 0: // prismatic
            {
                pose[i] = M2MM(hw_commands_positions_[i]);
                break;
            }
            case 1: // revolute
            {
                pose[i] = RAD2DEG(hw_commands_positions_[i]);
                break;
            }
            case -1: //fixed
            {
                pose[i] = 0.0;
                break;
            }
            }
        bits |= (1 << i);
        }
        pose.push_back(0x400000 | bits);
        HRESULT hr = denso_robot_->ExecSlaveMove(pose, hw_joints_); // Why it need joints?  TODO
        if (SUCCEEDED(hr))
        {
            if (recvfmt_ & DensoRobot::RECVFMT_HANDIO)
            {
                // some topic cmd operation .....  TODO
            }
        }
        else if (FAILED(hr) && (hr != DensoRobot::E_BUF_FULL))
        {
            printErrorDescription(hr, "Failed to write");
            if (!hasError())
            {
                return hardware_interface::return_type::ERROR;
            }
        }
    }
    
    return hardware_interface::return_type::OK;
}

HRESULT denso_hw::CheckRobotType()
{
  DensoVariable_Ptr pVar;
  VARIANT_Ptr vntVal(new VARIANT());
  std::string strTypeName = "@TYPE_NAME";

  HRESULT hr = denso_robot_->AddVariable(strTypeName);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to add @TYPE_NAME");
    return hr;
  }
  denso_robot_->get_Variable(strTypeName, &pVar);
  hr = pVar->ExecGetValue(vntVal);
  if (FAILED(hr))
  {
    printErrorDescription(hr, "Failed to get @TYPE_NAME");
    return hr;
  }
  strTypeName = DensoBase::ConvertBSTRToString(vntVal->bstrVal);
  if (strncmp(name_.c_str(), strTypeName.c_str(),
              (name_.length() < strTypeName.length()) ? name_.length() : strTypeName.length()))
  {
    return E_FAIL;
  }

  return 0;
}

bool denso_hw::hasError()
{
  HRESULT hr;
  VARIANT_Ptr vntVal(new VARIANT());

  hr = denso_var_->ExecGetValue(vntVal);
  if (SUCCEEDED(hr) && (vntVal->lVal == 0))
  {
    return false;
  }
  return true;
}

HRESULT denso_hw::ChangeModeWithClearError(int mode)
{
    HRESULT hr = denso_core_->ChangeMode(mode, mode == DensoRobot::SLVMODE_NONE);
    if (FAILED(hr))
    {
        // Clear Error
        HRESULT hres = denso_ctrl_->ExecClearError();
        if (FAILED(hr))
        {
            printErrorDescription(hres, "Failed to clear error");
        }
    }
    return hr;
}

void denso_hw::printErrorDescription(HRESULT error_code, const std::string& error_message)
{
  HRESULT hr;
  std::string error_description;

  if (denso_core_->get_Mode() == DensoRobot::SLVMODE_NONE)
  {
    hr = denso_ctrl_->ExecGetErrorDescription(error_code, error_description);
    if (SUCCEEDED(hr))
    {
      RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),"%s: %s (%X)", error_message.c_str(), error_description.c_str(), error_code);
      return;
    }
  }
  RCLCPP_ERROR(rclcpp::get_logger(this->get_name()),"%s (%X)", error_message.c_str(), error_code);
}
} // namespace denso2

#include <pluginlib/class_list_macros.hpp>
PLUGINLIB_EXPORT_CLASS(
    denso2::denso_hw,
    hardware_interface::SystemInterface)