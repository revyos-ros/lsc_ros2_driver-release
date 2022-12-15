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

#include <iostream>

#ifndef _MSC_VER
#include <unistd.h>
#include <sys/poll.h>
#endif

#include <string.h>
#include <errno.h>

#include <thread>

#include "socket.hpp"


tcpComm::tcpComm()
{
    m_server_sock_ = 0;
    m_connected_ = false;
    rcv_timeout_ = false;
    rcv_error_ = false;
    thread_running_ = false;
}

tcpComm::~tcpComm()
{
    CommDisconnect();
}


void tcpComm::putBufToMsg(unsigned char* buf, uint16_t size)
{
    std::vector<unsigned char> vec;
    vec.reserve(size);

    for(int i = 0; i < size; i++)
    {
        vec.push_back(*(buf+i));
    }
    recvQueue.push(vec);
}


#ifdef _MSC_VER
SOCKET tcpComm::getServerSocket(void)
#else
int tcpComm::getServerSocket(void)
#endif
{
    return m_server_sock_;
}


bool tcpComm::IsConnected(void)
{
    return m_connected_;
}

bool tcpComm::getthreadRunning(void)
{
    return thread_running_;
}


bool tcpComm::getRcvTimeout(void)
{
    return rcv_timeout_;
}

bool tcpComm::getRcvError(void)
{
    return rcv_error_;
}

void tcpComm::setConnected(bool flag)
{
    m_connected_ = flag;
}

void tcpComm::setThreadRunning(bool flag)
{
    thread_running_ = flag;
}

void tcpComm::setRcvTimeout(bool flag)
{
    rcv_timeout_ = flag;
}

void tcpComm::setRcvError(bool flag)
{
    rcv_error_ = flag;
}


void* readCallback(void* arg)
{
    int32_t read_cnt = 0;
    uint16_t packet_size = 0, total_byte = 0, offset = 0;
    int ret = 0;
    unsigned char r_buffer[16384], buffer[16384], cp_temp[16];
    tcpComm* sock = (tcpComm*) arg;

#ifdef _MSC_VER
    struct pollfd fd[1];
    fd[0].fd = sock->getServerSocket();
    fd[0].events = POLLIN;
#else
    struct pollfd fd;
    fd.fd = sock->getServerSocket();
    fd.events = POLLIN;
#endif

    while(sock->getthreadRunning())
    {
        read_cnt = 0;
        offset = 0;
#ifdef _MSC_VER
        ret = WSAPoll(fd, 1, 1000);
#else
        ret = poll(&fd, 1, 1000);
#endif
        if(ret < 0)
        {
            sock->setRcvError(true);
            std::cout << "polling error" << std::endl;
        }
        else /// ret >= 0
        {
            sock->setRcvError(false);

            read_cnt = sock->CommRead(r_buffer, sizeof(r_buffer));

            if(ret == 0)
            {
                sock->setRcvTimeout(true);
            #ifdef PRINT_DEBUG
                std::cout << "polling timeout" << std::endl;
            #endif
            }

            if(read_cnt > 0)
            {
                sock->setRcvTimeout(false);

                /// Search packet size
                if(packet_size == 0)
                {
                    for(uint16_t i = 0; i < read_cnt; i++)
                    {
                        if(r_buffer[i] == STX)
                        {
                            for(int j = 0; j < 4; j++)
                            {
                                cp_temp[j] = r_buffer[i+j+1];
                            }
                            packet_size = (uint16_t) strtoul((const char*) cp_temp, NULL, 16);
                            break;
                        }
                    }
                }

                /// Search ETX
                for(uint16_t i = 0; i < read_cnt - 1; i++)
                {
                    if(r_buffer[read_cnt - 1 - i] == ETX)
                    {
                        read_cnt -= i;
                        break;
                    }
                }

                if(packet_size > total_byte)
                {
                    memcpy(&buffer[total_byte], &r_buffer[offset], read_cnt - offset);
                    total_byte += read_cnt - offset;
                }
                else if(packet_size < total_byte)
                {
                    if(buffer[0] == 0x02 && buffer[packet_size] == 0x03)
                    {
                        total_byte = packet_size;
                    }
                    else
                    {
                        memset(buffer, 0x00, sizeof(buffer));
                        packet_size = 0;
                        total_byte = 0;
                        offset = 0;
                    }
                }
                else
                {

                }

                if(packet_size !=0 && total_byte != 0 && packet_size == total_byte)
                {
                    if(buffer[total_byte - 1] == 0x03)
                    {
                        sock->putBufToMsg(buffer, total_byte);
                        memset(buffer, 0x00, sizeof(buffer));
                        packet_size = 0;
                        total_byte = 0;
                    }
                }
                else
                {

                }

                memset(r_buffer, 0x00, sizeof(r_buffer));
            }
            else if(read_cnt < 0)    /// disconnect
            {
                std::cout << "server is disconnected" << std::endl;
                sock->setRcvError(false);
                sock->setConnected(false);
                sock->setThreadRunning(false);
            }
            else /// receive error
            {
                if(sock->CommWrite(r_buffer, sizeof(r_buffer)))
                {
                    std::cout << "checked connection, server is disconnected" << std::endl;
                    sock->setConnected(false);
                    sock->setThreadRunning(false);
                }
            }
        }
    }

    return 0;
}


void tcpComm::CommDisconnect()
{
    uint16_t bk_cnt = 0;
    thread_running_ = false;

    if(thread_read_.joinable())
    {
        thread_read_.join();
    }

#ifdef _MSC_VER
    closesocket(m_server_sock_);
#else
    while(close(m_server_sock_) !=0 || bk_cnt >= 500)
    {
        bk_cnt++;
        std::this_thread::sleep_for(std::chrono::milliseconds(10));
    }

#endif
    if(m_connected_ == true)
    {
        m_connected_ = false;
        std::cout << "disconnect server" << std::endl;
    }
}


int tcpComm::CommConnect(std::string addr, uint16_t port)
{
    int m_server_addr_size = 0;
    timeval tv;
    addr_st_ = addr;
    port_num_ = port;

    m_server_sock_ = socket(PF_INET, SOCK_STREAM, 0);

    if(m_server_sock_ < 0)
    {
        std::cout << "faild to create client socket" << std::endl;
        return -1;
    }

    tv.tv_sec = RS_TIMEOUT_S;
    tv.tv_usec = RS_TIMEOUT_NS;

    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_RCVTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed rcvtimeo setsockopt" << std::endl;
    }
    if(setsockopt(m_server_sock_, SOL_SOCKET, SO_SNDTIMEO, (char*)&tv, sizeof(tv)) < 0)
    {
        std::cout << "failed sndtimeo setsockopt" << std::endl;
    }

    memset(&m_server_addr_, 0x00, sizeof(m_server_addr_));

    m_server_addr_.sin_family = AF_INET;
#ifdef _MSC_VER
    m_server_addr_.sin_addr.s_addr = inet_addr(addr.c_str());
#else
    m_server_addr_.sin_addr.s_addr = inet_addr(addr.begin().base());
#endif
    m_server_addr_.sin_port = htons(port);

    m_server_addr_size = sizeof(m_server_addr_);

    if(connect(m_server_sock_, (const sockaddr*)&m_server_addr_, m_server_addr_size) < 0)
    {
        std::cout << "failed client connect, "<< errno << " " << strerror(errno) << std::endl;
        return -1;
    }
    else
    {
        m_connected_ = true;
        std::cout << "connected" << std::endl;
    }

    try
    {
        thread_read_ = std::thread(readCallback, this);
        thread_running_ = true;
        std::cout << "thread created" << std::endl;
    }
    catch(const std::exception& e)
    {
        std::cerr << e.what() << '\n';
        std::cout << "thread create failed" << std::endl;
        return -1;
    }

    return 0;
}


int tcpComm::CommRead(unsigned char* buffer, int buf_size)
{
    int read_size = 0;

    if(m_connected_ == true)
    {
        read_size = recv(m_server_sock_, (char *)buffer, buf_size, 0);
        if(read_size < 0)
        {
            std::cout << "client read failed, " << errno << " " << strerror(errno) << std::endl;
            if(errno == 104)    /// connect reset
            {
                thread_running_ = false;
                m_connected_ = false;
                read_size = 0;
            }
            else if(errno == 11)    /// no message received
            {
                read_size = 0;
            }
        }
        else if(read_size == 0)
        {
            std::cout << "server socket has closed" << std::endl;
        }
    }
    else
    {
        std::cout << "read failed, server is disconnected" << std::endl;
        read_size = -1;
    }

    return read_size;
}


int tcpComm::CommWrite(unsigned char* buffer, int buf_size)
{
    int write_size = 0;

    if(m_connected_ == true)
    {
        write_size = send(m_server_sock_, (const char *)buffer, buf_size, 0);
        if(write_size < 0)
        {
            std::cout << "client write failed, " << errno << " " << strerror(errno) << std::endl;
            if(errno == 104)
            {
                thread_running_ = false;
                m_connected_ = false;
                write_size = 0;
            }
        }
        else if(write_size == 0)
        {
            std::cout << "no byte sent" << std::endl;
        }
    }
    else
    {
        std::cout << "write failed, server is disconnected" << std::endl;
        write_size = -1;
    }

    return write_size;
}

