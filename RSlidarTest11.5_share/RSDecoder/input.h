//
// Created by zhwu on 4/11/19.
//

#ifndef RSLIDAR_CLIENT_2_INPUT_H
#define RSLIDAR_CLIENT_2_INPUT_H

#pragma once

#include <cstdint>
#include <memory>
#include <string>
#include <pcap.h>


namespace robosense
{
  namespace rslidar_input
  {
    const int RSLIDAR_PKT_LEN = 1248;

    enum InputState
    {
      INPUT_OK = 0,
      INPUT_TIMEOUT = 1,
      INPUT_ERROR = 2,
      INPUT_DIFOP = 4,
      INPUT_MSOP = 8,
      INPUT_EXIT = 16
    };

    class Input
    {
    public:
      Input(std::string device_ip,uint16_t msop_port,uint16_t difop_port,
                     std::string pcap_file_dir);
      ~Input();

      InputState getPacket(uint8_t *pkt);

    private:
      int setUpSocket(uint16_t port);

      uint16_t msop_port_;
      uint16_t difop_port_;
      int msop_fd_;
      int difop_fd_;

      pcap_t *pcap_;
      bpf_program pcap_msop_filter_;
      bpf_program pcap_difop_filter_;

      std::string device_ip_;
      std::string pcap_file_dir_;

    };
  }
}

#endif //RSLIDAR_CLIENT_2_INPUT_H
