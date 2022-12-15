/*
* Copyright (c) 2022, Autonics Co.,Ltd.
* All rights reserved.
*
* Redistribution and use in source and binary forms, with or without
* modification, are permitted provided that the following conditions
* are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*
*   * Neither the name of the Autonics Co.,Ltd nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
* THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
* "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
* LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
* FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
* COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
* INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
* BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
* LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
* CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
* LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
* ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
* POSSIBILITY OF SUCH DAMAGE.
*/


#include <sstream>
#include <thread>

#include <chrono>
#include <string>
#include <functional>

#include "autolidar.hpp"

using namespace std::chrono_literals;

extern const char* cmd_type_list[NUM_OF_CMD_TYPE];
extern const char* cmd_list[NUM_OF_CMD];

Autolidar::Autolidar(): Node("autonics_lsc_lidar")
#ifdef USE_DIAGNOSTICS
, diagnostic_updater_laser_(this)
#endif
{
}

Autolidar::~Autolidar()
{
}


int Autolidar::init()
{
  uint16_t reconnect_cnt = 0;

  declearParam<std::string>("addr", "192.168.0.1");
  declearParam<uint16_t>("port", 8000);
  declearParam<std::string>("frame_id", "laser");
  declearParam<double>("range_min", 0.05);
  declearParam<double>("range_max", 25);
  declearParam<bool>("intensities", true);

  declearParam<std::string>("pub_topic", "scan");
  #ifdef USE_DIAGNOSTICS
  declearParam<double>("diagnostics_tolerance", 0.1);
  declearParam<int>("diagnostics_windows_time", 1);
  #endif

  ip_addr = this->get_parameter("addr").get_value<std::string>();
  port_number = this->get_parameter("port").get_value<uint16_t>();
  frame_id = this->get_parameter("frame_id").get_value<std::string>();
  intensities = this->get_parameter("intensities").get_value<bool>();
  range_min = this->get_parameter("range_min").get_value<double>();
  range_max = this->get_parameter("range_max").get_value<double>();

  pub_topic = this->get_parameter("pub_topic").get_value<std::string>();
  #ifdef USE_DIAGNOSTICS
  diagnostics_tolerance = this->get_parameter("diagnostics_tolerance").get_value<double>();
  diagnostics_windows_time = (int) this->get_parameter("diagnostics_windows_time").get_value<int>();
  #endif

  param_callback_ = this->add_on_set_parameters_callback(
  std::bind(&Autolidar::param_callback, this, std::placeholders::_1));

  if(comm_ == nullptr)
  {
    std::shared_ptr<Communication> comm_type = std::make_shared<tcpComm>();
    setCommType(comm_type);
  }

  if(parser_ == nullptr)
  {
    std::shared_ptr<Parser> parser = std::make_shared<AsciiParser>();
    setParser(parser);
  }


  if(comm_->CommConnect(ip_addr, port_number) < 0)
  {
    RCLCPP_ERROR(this->get_logger(), "failed to connect device");
    while(rclcpp::ok())
    {
      comm_->CommDisconnect();

      if(comm_->CommConnect(ip_addr, port_number) < 0)
      {
        reconnect_cnt++;
        if(errno == 111)
        {
          std::this_thread::sleep_for(std::chrono::milliseconds(1000));
        }

        if(reconnect_cnt >= RECONNECT_TRY_COUNT)
        {
          return -1;
        }
        RCLCPP_INFO(this->get_logger(), "trying to reconnect");
      }
      else
      {
        reconnect_cnt = 0;
        break;
      }
    }
  }

  lsc_ = std::make_shared<Lsc_t>();
  scan_msg_ = std::make_shared<sensor_msgs::msg::LaserScan>();
  laser_topic_publisher_ = this->create_publisher<sensor_msgs::msg::LaserScan>(pub_topic, rclcpp::QoS(rclcpp::SystemDefaultsQoS()));


  /// receive firstdummy data
  if(get_response("FirstConnectDummySend") < 0)
  {
    RCLCPP_INFO(this->get_logger(), "failed to get firstdummydata");
  }
  send_command("SensorScanInfo");

#ifdef USE_DIAGNOSTICS
  diagnostic_updater_laser_.add("Connection", this, &Autolidar::check_connection);
  diagnostic_updater_laser_.setHardwareID(lsc_->scan_info.model_name);
  min_freq_ = 0.00025;  /// Hz
  max_freq_ = 15.0;     /// Hz

  scan_pub_freq_.reset(new diagnostic_updater::HeaderlessTopicDiagnostic(
    "laser_scan", \
    diagnostic_updater_laser_, \
    diagnostic_updater::FrequencyStatusParam(&min_freq_, &max_freq_, diagnostics_tolerance, diagnostics_windows_time)
    ));

#endif

  send_command("SensorStart");
  pub_timer_ = this->create_wall_timer(10ms, std::bind(&Autolidar::run, this));

  return 0;
}

void Autolidar::run()
{
  if(comm_->IsConnected() == false)
  {
    comm_->CommDisconnect();

    if(comm_->CommConnect(ip_addr, port_number) < 0)
    {
      if(errno == 111)
      {
        std::this_thread::sleep_for(std::chrono::milliseconds(1000));
      }
      RCLCPP_INFO(this->get_logger(), "trying to reconnect");
    }
    else
    {
      send_command("SensorStart");
    }
  }

  if(!comm_->recvQueue.empty())
  {
    parser_->parsingMsg(comm_->recvQueue.front(), lsc_);
    comm_->recvQueue.pop();

    if(!lsc_->scan_mea.ranges.empty())
    {
      publish_laserscan(scan_msg_, lsc_);
      #ifdef USE_DIAGNOSTICS
        if(!diagnostic_param_setting)
        {
          scan_pub_freq_->tick();
        }

      #endif
    }
  }
}


unsigned char Autolidar::itohascii(unsigned char input)
{
  unsigned char ret = 0;

  if(input < 10)
  {
    ret = input + '0';
  }
  else if(input < 16)
  {
    ret = input-10 + 'A';
  }
  else
  {
    ret = 255;
    RCLCPP_INFO(this->get_logger(), "out of hex range");
  }

  return ret;
}


int Autolidar::makeCommand(std::vector<unsigned char>& v_cmd_frame, std::string cmd)
{
    uint8_t cmd_num = 0;
    size_t cmd_size = 0;

    v_cmd_frame.reserve(40);
    v_cmd_frame.resize(5);
    v_cmd_frame[0] = 2;
    v_cmd_frame.push_back(',');

    for(cmd_num = 0; cmd_num < NUM_OF_CMD; cmd_num++)
    {
        if(strcmp(cmd_list[cmd_num], cmd.c_str()) == 0)
        {
            break;
        }
    }

    switch(cmd_num)
    {
        case SET_ACCESS_LEVEL :
        case SENSOR_START :
        case SENSOR_STOP :
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('M');
            v_cmd_frame.push_back('C');
            break;

        case SENSOR_SCAN_INFO :
            v_cmd_frame.push_back('s');
            v_cmd_frame.push_back('R');
            v_cmd_frame.push_back('C');
            break;

        case NONE :
        default :
            RCLCPP_INFO(this->get_logger(), "invalid command, type correct command %d", cmd_num);
            return -1;
            break;
    }
    v_cmd_frame.push_back(',');
    v_cmd_frame.insert(v_cmd_frame.end(), cmd.begin(), cmd.end());
    v_cmd_frame.push_back(3);
    cmd_size = v_cmd_frame.size();

    v_cmd_frame[4] = itohascii(cmd_size%16);
    for(uint8_t i = 3; i > 0; i--)
    {
        cmd_size /= 16;
        v_cmd_frame[i] = itohascii((unsigned char)cmd_size);
    }

    #ifdef PRINT_PARSED_DATA
    std::cout << "cmd_struct : ";
    for(uint16_t i = 0; i < v_cmd_frame.size(); i++)
    {
        std::cout << v_cmd_frame[i];
    }
    std::cout << "" << std::endl;
    #endif

    return (int)v_cmd_frame.size();
}


int Autolidar::get_response(std::string cmd)
{
  uint16_t cnt = 0;
  while(cnt < RESPONS_WAIT_CNT)
  {
    if(!comm_->recvQueue.empty())
    {
      parser_->parsingMsg(comm_->recvQueue.front(), lsc_);
      comm_->recvQueue.pop();

      if(strcmp(cmd.c_str(), parser_->getResponse().c_str()) == 0)
      {
        break;
      }
      else
      {

      }
    }
    cnt++;
    std::this_thread::sleep_for(std::chrono::milliseconds(100));

    if(cnt >= RESPONS_WAIT_CNT)
    {
      RCLCPP_INFO(this->get_logger(), "getting response failed, count : %d", cnt);
      return -1;
    }
  }
  return 0;
}

void Autolidar::setCommType(std::shared_ptr<Communication> comm_type)
{
  comm_ = comm_type;
}

void Autolidar::setParser(std::shared_ptr<Parser> parser)
{
  parser_ = parser;
}


int8_t Autolidar::send_command(std::string cmd)
{
  std::vector<unsigned char> cmd_struct;

  int cmd_size = 0;
  uint8_t try_send_cnt = 0;
  int8_t ret = -1;

  cmd_size = makeCommand(cmd_struct, cmd);
  do{
    comm_->CommWrite(&cmd_struct[0], cmd_size);
    try_send_cnt++;

    if(get_response(cmd) == 0)
    {
      ret = 0;
      break;
    }
    else
    {
      ;
    }

  }while(try_send_cnt < 3);

  return ret;
}


#ifdef USE_DIAGNOSTICS
void Autolidar::check_connection(diagnostic_updater::DiagnosticStatusWrapper & stat)
{
  if(comm_->IsConnected())
  {
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::OK, "connection is ok");
  }
  else
  {
#ifdef _MSC_VER
    stat.summary(2, "lost connection");
#else
    stat.summary(diagnostic_msgs::msg::DiagnosticStatus::ERROR, "lost connection");
#endif
  }
}
#endif


rcl_interfaces::msg::SetParametersResult
 Autolidar::param_callback(const std::vector<rclcpp::Parameter> &parameters)
{
  rcl_interfaces::msg::SetParametersResult result;
  result.successful = true;
  result.reason = "success";

  for (const auto &param : parameters)
  {
    std::string param_name = param.get_name();
    rclcpp::ParameterType param_type = param.get_type();

    if(param_name == "frame_id")
    {
      if(param_type != rclcpp::ParameterType::PARAMETER_STRING)
      {
        result.successful = false;
        result.reason = "parameter type is not string, ex) laser";
      }
      else
      {
        frame_id = param.as_string();
      }
    }
    else if(param_name == "range_min")
    {
      if(param_type != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "parameter type is not double, ex) 0.05";
      }
      else if (param.as_double() > range_max)
      {
        result.successful = false;
        result.reason = "range_min must be lower value than range_max";
      }
      else if (param.as_double() < 0.05)
      {
        result.successful = false;
        result.reason = "range_min must be upper than 0.05";
      }
      else
      {
        range_min = param.as_double();
      }
    }
    else if(param_name == "range_max")
    {
      if(param_type != rclcpp::ParameterType::PARAMETER_DOUBLE)
      {
        result.successful = false;
        result.reason = "parameter type is not double, ex) 25.0";
      }
      else if (param.as_double() < range_min)
      {
        result.successful = false;
        result.reason = "range_max must be upper value than range_min";
      }
      else if (param.as_double() > 25.0)
      {
        result.successful = false;
        result.reason = "range_max must be lower than 25.0";
      }
      else
      {
        range_max = param.as_double();
      }
    }
    else if(param_name == "intensities")
    {
      if(param_type != rclcpp::ParameterType::PARAMETER_BOOL)
      {
        result.successful = false;
        result.reason = "parameter type is not bool, ex) true";
      }
      else
      {
        intensities = param.as_bool();
      }
    }
    else if(param_name == "diagnostics_tolerance" || param_name == "diagnostics_windows_time")
    {
        result.successful = false;
        result.reason = "this parameter is static, go to launch file to change this";
    }
    else
    {

    }
  }

  return result;
}


void Autolidar::publish_laserscan(std::shared_ptr<sensor_msgs::msg::LaserScan> msg, std::shared_ptr<Lsc_t> lsc)
{
  msg->header.frame_id = frame_id;
  msg->range_max = (float) range_max;
  msg->range_min = (float) range_min;
  msg->header.stamp = this->now();
  msg->angle_min = (float) (lsc->scan_mea.angle_begin / 10000.0 * DEG2RAD);
  msg->angle_max = (float) ((lsc->scan_mea.angle_begin / 10000.0 + (lsc->scan_mea.angle_resol / 10000.0 * lsc->scan_mea.amnt_of_data)) * DEG2RAD);
  msg->angle_increment = (float) (lsc->scan_mea.angle_resol / 10000.0 * DEG2RAD);
  msg->time_increment = (float) (60.0 / lsc->scan_mea.meas_freq) / lsc->scan_mea.scan_counter;
  msg->scan_time = (float) (60.0 / lsc->scan_mea.meas_freq);

  msg->ranges.reserve(lsc->scan_mea.ranges.size());

  for(uint16_t i=0; i < lsc->scan_mea.ranges.size(); i++)
  {
    msg->ranges.emplace_back(lsc->scan_mea.ranges[i]);
  }

  if(intensities)
  {
    if(!lsc->scan_mea.rssi.empty())
    {
      msg->intensities.reserve(lsc->scan_mea.rssi.size());
      for(uint16_t i=0; i < lsc->scan_mea.rssi.size(); i++)
      {
        msg->intensities.emplace_back(lsc->scan_mea.rssi[i]);
      }
    }
  }

  laser_topic_publisher_->publish(*msg);

  msg->ranges.clear();
  msg->intensities.clear();
}

template<typename T>
bool Autolidar::declearParam(std::string name, T value)
{
  if(this->has_parameter(name) != true)
  {
    this->declare_parameter<T>(name, value);
    return true;
  }
  else
  {
    return false;
  }

}
