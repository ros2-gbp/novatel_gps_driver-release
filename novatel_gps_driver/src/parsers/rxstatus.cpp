// *****************************************************************************
//
// Copyright (c) 2019, Southwest Research Institute® (SwRI®)
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are met:
//     * Redistributions of source code must retain the above copyright
//       notice, this list of conditions and the following disclaimer.
//     * Redistributions in binary form must reproduce the above copyright
//       notice, this list of conditions and the following disclaimer in the
//       documentation and/or other materials provided with the distribution.
//     * Neither the name of Southwest Research Institute® (SwRI®) nor the
//       names of its contributors may be used to endorse or promote products
//       derived from this software without specific prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
// AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
// IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
// ARE DISCLAIMED. IN NO EVENT SHALL SOUTHWEST RESEARCH INSTITUTE BE LIABLE FOR ANY
// DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
// (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
// LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
// ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
// SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
// *****************************************************************************

#include "novatel_gps_driver/parsers/parsing_utils.h"
#include <sstream>

#include <novatel_gps_driver/parsers/rxstatus.h>

#include <novatel_gps_driver/parsers/header.h>

namespace novatel_gps_driver
{
  const std::string RxStatusParser::MESSAGE_NAME = "RXSTATUS";

  uint32_t RxStatusParser::GetMessageId() const
  {
    return MESSAGE_ID;
  }

  const std::string RxStatusParser::GetMessageName() const
  {
    return MESSAGE_NAME;
  }

  RxStatusParser::MessageType RxStatusParser::ParseBinary(const BinaryMessage& bin_msg) noexcept(false)
  {
    if (bin_msg.data_.size() != BINARY_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected RXSTATUS message length: " << bin_msg.data_.size();
      throw ParseException(error.str());
    }
    auto ros_msg = std::make_shared<novatel_gps_msgs::msg::NovatelRxStatus>();
    HeaderParser header_parser;
    ros_msg->novatel_msg_header = header_parser.ParseBinary(bin_msg);
    ros_msg->novatel_msg_header.message_name = MESSAGE_NAME;

    ros_msg->error = ParseUInt32(&bin_msg.data_[0]);
    ros_msg->rxstat = ParseUInt32(&bin_msg.data_[8]);
    ros_msg->aux1stat = ParseUInt32(&bin_msg.data_[24]);
    ros_msg->aux2stat = ParseUInt32(&bin_msg.data_[40]);
    ros_msg->aux3stat = ParseUInt32(&bin_msg.data_[56]);
    ros_msg->aux4stat = ParseUInt32(&bin_msg.data_[72]);
    return ros_msg;
  }

  RxStatusParser::MessageType RxStatusParser::ParseAscii(const NovatelSentence& sentence) noexcept(false)
  {
    auto msg = std::make_shared<novatel_gps_msgs::msg::NovatelRxStatus>();
    HeaderParser h_parser;
    msg->novatel_msg_header = h_parser.ParseAscii(sentence);

    if (sentence.body.size() != ASCII_LENGTH)
    {
      std::stringstream error;
      error << "Unexpected number of RXSTATUS message fields: " << sentence.body.size();
      throw ParseException(error.str());
    }

    bool valid = true;

    valid = valid && ParseUInt32(sentence.body[0], msg->error);
    valid = valid && ParseUInt32(sentence.body[1], msg->rxstat);
    valid = valid && ParseUInt32(sentence.body[6], msg->aux1stat);
    valid = valid && ParseUInt32(sentence.body[10], msg->aux2stat);
    valid = valid && ParseUInt32(sentence.body[14], msg->aux3stat);
    valid = valid && ParseUInt32(sentence.body[18], msg->aux4stat);

    if (!valid)
    {
      throw ParseException("Invalid field in RXSTATUS message");
    }

    return msg;
  }
}
