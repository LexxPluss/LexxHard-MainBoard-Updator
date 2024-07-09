/*
 * Copyright (c) 2022, LexxPluss Inc.
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *
 * 1. Redistributions of source code must retain the above copyright notice,
 *    this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
 * ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
 * WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR
 * ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
 * ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 * (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
 * SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include <stdint.h>
#include <fcntl.h>
#include <string.h>
#include <ctime>
#include "ros/ros.h"
#include "std_msgs/UInt8MultiArray.h"
#include "std_msgs/UInt16MultiArray.h"

namespace {

ros::Publisher *pub{nullptr};
std_msgs::UInt8MultiArray packet;
char* pathname{nullptr};
bool reboot{false};
bool echoed{false};
bool error{false};
bool transmission_done{false};
uint16_t count{0};

enum class RESP {
    OK                = 0,
    COMPLETE          = 1,
    COMPLETE_RESET    = 2,
    ERR_PARTITION     = 3,
    ERR_FLASH_AREA    = 4,
    ERR_FLASH_ERASE   = 5,
    ERR_FLASH_PROGRAM = 6,
    ERR_CHECKSUM      = 7,
    ERR_TIMEOUT       = 8,
    ERR_POINTER       = 9,
};

enum class CMD {
    START           = 0,
    DATA            = 1,
    RESET           = 2,
    RESET_NO_REBOOT = 3,
};

void callback(const std_msgs::UInt16MultiArray::ConstPtr &response)
{
    switch (static_cast<RESP>(response->data[0])) {
    case RESP::OK:
        ROS_INFO("[Echoed from Mainboard] Packet number: %u", response->data[1]);
        if (count == response->data[1])
            echoed = true;
        break;
    case RESP::COMPLETE:
        ROS_INFO("[From Mainboard] data transmission has been completed");
        echoed = true;
        break;
    case RESP::COMPLETE_RESET:
        ROS_INFO("[From Mainboard] data transmission has been completed, rebooting in 2 seconds");
        echoed = true;
        break;
    case RESP::ERR_PARTITION:
        ROS_INFO("[From Mainboard] ERROR: partition not found");
        echoed = true;
        error = true;
        break;
    case RESP::ERR_FLASH_AREA:
        ROS_INFO("[From Mainboard] ERROR: failed to retrieve flash_area from flash map for image-1");
        echoed = true;
        error = true;
        break;
    case RESP::ERR_FLASH_ERASE:
        ROS_INFO("[From Mainboard] ERROR: erase unsuccessful");
        echoed = true;
        error = true;
        break;
    case RESP::ERR_FLASH_PROGRAM:
        ROS_INFO("[From Mainboard] ERROR: program unsuccessful");
        echoed = true;
        error = true;
        break;
    case RESP::ERR_CHECKSUM:
        ROS_INFO("[From Mainboard] Message: checksum failed");
        ROS_INFO("Message: re-sending packet %u", response->data[1]);
        pub->publish(packet);
        break;
    case RESP::ERR_TIMEOUT:
        ROS_INFO("[From Mainboard] ERROR: write unsuccessful, time out");
        echoed = true;
        error = true;
        break;
    case RESP::ERR_POINTER:
        ROS_INFO("[From Mainboard] ERROR: flash area pointer == nullptr when it shoud not");
        echoed = true;
        error = true;
        break;
    }
}

void resend_dropped_packet(const std::time_t packet_dispatch_time)
{
    std::time_t current_time{std::time(nullptr)};
    if (current_time - packet_dispatch_time > 8) {
        pub->publish(packet);
        ROS_INFO("Message: Mainboard has not echoed back");
        ROS_INFO("Message: re-sending...");
        ROS_INFO("[Data Transferred to MainBoard] ...");
        sleep(2);
    }
    if (current_time - packet_dispatch_time > 20) {
        ROS_INFO("ERROR: failed to send dropped packet");
        error = true;
    }
}

void checksum_calculator(uint8_t *data)
{
    uint8_t checksum{0x0};
    for (int i{3}; i < 259; ++i)
        checksum += data[i];
    data[259] = checksum;
}

bool sanity_check(int argc, char **argv)
{
    if (argc < 2 || argc > 3)
        return false;
    pathname = argv[1];
    ROS_INFO("Message: pathname -> %s", pathname);
    if (argc == 3) {
        if (strcmp(argv[2], "reboot") == 0) {
            reboot = true;
            ROS_INFO("Message: reboot option -> enabled");
            return true;
        } else {
            return false;
        }
    } else {
        ROS_INFO("Message: reboot option -> disabled");
        return true;
    }
}

}

int main(int argc, char **argv)
{
    ros::init(argc, argv, "mainboard_updator");
    ros::NodeHandle nh;
    ros::Publisher pub_temp{nh.advertise<std_msgs::UInt8MultiArray>("/global/lexxhard/dfu_data", 10)};
    pub = &pub_temp;
    ros::Subscriber sub{nh.subscribe("/global/lexxhard/dfu_response", 10, callback)};
    sleep(1);
    ros::Rate loop_rate{100};
    packet.data.resize(260);
    if (!sanity_check(argc, argv)) {
        ROS_INFO("ERROR: wrong number of arguments or an invalid optional arugument");
        return 0;
    }
    int fd{open(pathname, O_RDONLY)};
    if (fd < 0) {
        ROS_INFO("ERROR: unable to open new image");
        return 0;
    }
    while (ros::ok() && !transmission_done && !error) {
        echoed = false;
        packet.data[0] = count;
        packet.data[1] = count >> 8;
        packet.data[2] = static_cast<uint8_t>(count == 0 ? CMD::START : CMD::DATA);
        ssize_t size_read{read(fd, &(packet.data[3]), 256)};
        if (size_read != 256) {
            ROS_INFO("ERROR: failed to read new image");
            close(fd);
            return 0;
        }
        checksum_calculator(&packet.data[0]);
        pub->publish(packet);
        std::time_t packet_dispatch_time{std::time(nullptr)};
        ROS_INFO("[Data Transferred to MainBoard] Packet number: %d, Size: %ldB", count, size_read);
        while (ros::ok() && !echoed && !error) {
            ros::spinOnce();
            loop_rate.sleep();
            resend_dropped_packet(packet_dispatch_time);
        }
        if (count == 1023) {
            transmission_done = true;//also when size_read == 0
            close(fd);
            packet.data[2] = static_cast<uint8_t>(reboot ? CMD::RESET : CMD::RESET_NO_REBOOT);
            pub->publish(packet);
            echoed = false;
            while (ros::ok() && !echoed && !error) {
                ros::spinOnce();
                loop_rate.sleep();
                resend_dropped_packet(packet_dispatch_time);
            }
        }
        ++count;
    }
    return 0;
}

// vim: set expandtab shiftwidth=4:
