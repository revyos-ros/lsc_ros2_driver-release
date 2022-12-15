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


#include <queue>
#include <cstring>
#include <iostream>

#include <memory>

#include "parser.hpp"

#include <chrono>


const char* cmd_type_list[NUM_OF_CMD_TYPE] =
{"sMC",
"sMA",
"sRC",
"sRA",
};

const char* cmd_list[NUM_OF_CMD] =
{"None",
"SetAccessLevel",
"SensorScanInfo",
"SensorStart",
"SensorStop",
"ScanData",
"FirstConnectDummySend"
};

std::string AsciiParser::getResponse()
{
    return command_;
}


void AsciiParser::parsingMsg(std::vector<unsigned char> raw_msg, std::shared_ptr<Lsc_t> parsed_data)
{
    uint16_t index= 0;
 #ifdef PRINT_PARSED_DATA
    int cmd_type = 0;
#endif
    int cmd = 0;
    std::vector<char *> field;
    char* ptr;


    if(raw_msg[0] == 0x02)
    {
    #ifdef PRINT_PARSED_DATA
        std::cout << "raw_msg : ";
        for(uint16_t i = 0; i < raw_msg.size(); i++)
        {
            std::cout << raw_msg[i];
        }
        std::cout << "" << std::endl;
    #endif


        ptr = strtok((char*) &raw_msg[1], ",");

        if(raw_msg[strtoul(ptr,NULL, 16) - 1] == 0x03)
        {
            raw_msg[strtoul(ptr, NULL, 16) - 1] = '\0';
        }

        while(ptr != NULL)
        {
            field.push_back(ptr);
            ptr = strtok(NULL, ",");
        }
        index++;

    #ifdef PRINT_PARSED_DATA
        for(int i = 0; i < NUM_OF_CMD_TYPE; i++)
        {
            int temp_ret = strcmp(field[index], cmd_type_list[i]);

            if(temp_ret == 0)
            {
                cmd_type = i;
                break;
            }
        }
    #endif
        index++;

        command_ = field[index];

        for(int i = 0; i < NUM_OF_CMD; i++)
        {
            if(strcmp(field[index], cmd_list[i]) == 0)
            {
                cmd = i;
                break;
            }
        }
        index++;


        switch(cmd)
        {
            case SENSOR_SCAN_INFO :
                parsed_data->scan_info.fw_ver = (uint16_t)strtoul(field[8], NULL, 16);
                parsed_data->scan_info.model_name = field[13];

                #ifdef PRINT_PARSED_DATA
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "fw_ver : " << parsed_data->scan_info.fw_ver << std::endl;
                std::cout << "Model_name : " << parsed_data->scan_info.model_name << std::endl;
                #endif

                break;

            case SCAN_DATA :


                for(int i = 0; i < NUMBER_OF_PARAM; i++)
                {
                    ptr = strtok(NULL, ",");
                    field.push_back(ptr);
                }

                parsed_data->scan_mea.scan_counter = (uint16_t)strtoul(field[7], NULL, 16);
                parsed_data->scan_mea.scan_freq = (uint16_t)strtoul(field[10], NULL, 16);
                parsed_data->scan_mea.meas_freq = (uint16_t)strtoul(field[11], NULL, 16);
                parsed_data->scan_mea.angle_begin = (int32_t)strtoul(field[12], NULL, 16);
                parsed_data->scan_mea.angle_resol = (uint16_t)strtoul(field[13], NULL, 16);
                parsed_data->scan_mea.amnt_of_data = (uint16_t)strtoul(field[14], NULL, 16);
                index = 16;

                #ifdef PRINT_PARSED_DATA
                std::cout << "Cmd type : " << cmd_type_list[cmd_type] << std::endl;
                std::cout << "Command : " << cmd_list[cmd] << std::endl;
                std::cout << "Scan counter : " << parsed_data->scan_mea.scan_counter << std::endl;
                std::cout << "Scan frequency : " << std::dec << parsed_data->scan_mea.scan_freq << std::endl;
                std::cout << "Measuring frequency : " << parsed_data->scan_mea.meas_freq << std::endl;
                std::cout << "Angle begin : " << parsed_data->scan_mea.angle_begin << std::endl;
                std::cout << "Angle resolution : " << parsed_data->scan_mea.angle_resol << std::endl;
                std::cout << "Amount of data : " << parsed_data->scan_mea.amnt_of_data << std::endl;
                #endif


                parsed_data->scan_mea.ranges.clear();
                if(strcmp(field[index++], "DIST1") == 0)
                {
                    parsed_data->scan_mea.ranges.reserve(parsed_data->scan_mea.amnt_of_data);
                    for(uint16_t i = 0; i < parsed_data->scan_mea.amnt_of_data; i++)
                    {
                        parsed_data->scan_mea.ranges.emplace_back((float) (strtoul(field[index++], NULL, 16) / 1000.0));
                    }

                    #ifdef PRINT_PARSED_DATA
                    std::cout << "ranges : " << std::endl;
                    for(uint16_t i = 0; i < parsed_data->scan_mea.ranges.size(); i++)
                    {
                        std::cout << parsed_data->scan_mea.ranges[i] << " ";
                    }
                    std::cout << "" << std::endl;
                    #endif
                }

                parsed_data->scan_mea.rssi.clear();
                if(field[index] != field.back())
                {
                    if(strcmp(field[index++], "RSSI1") == 0 )
                    {
                        parsed_data->scan_mea.rssi.reserve(parsed_data->scan_mea.amnt_of_data);
                        for(uint16_t i = 0; i < parsed_data->scan_mea.amnt_of_data; i++)
                        {
                            parsed_data->scan_mea.rssi.emplace_back((float) strtoul(field[index++], NULL, 16));
                        }

                        #ifdef PRINT_PARSED_DATA
                        std::cout << "rssi : " << std::endl;
                        for(uint16_t i = 0; i < parsed_data->scan_mea.rssi.size(); i++)
                        {
                            std::cout << parsed_data->scan_mea.rssi[i] << " ";
                        }
                        std::cout << "" << std::endl;
                        #endif
                    }
                }
                break;

            case SENSOR_START :
            case SENSOR_STOP :
            case SET_ACCESS_LEVEL :
            case FIRST_CONNECT_DUMMY_SEND :
                for(std::size_t i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                std::cout << "" << std::endl;
                break;

            default :
                for(std::size_t i = 2; i < field.size(); i++)
                {
                    std::cout << field[i] << " ";
                }
                std::cout << "" << std::endl;
                break;
        }

    raw_msg.clear();
    }
}
