////////////////////////////////////////////////////////////////////////////////
//
// Copyright (c) 2020 NovAtel Inc.
//
// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:
//
// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.
//
// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.
//
////////////////////////////////////////////////////////////////////////////////

#ifndef __MESSAGE_HANDLER_HPP__
#define __MESSAGE_HANDLER_HPP__

#include <pluginlib/class_loader.hpp>
#include <pluginlib/class_list_macros.hpp>



#include "oem7_raw_message_if.hpp"
using bynav_oem7::Oem7RawMessageIf;

#include "bynav_ros_driver/oem7_message_decoder_if.hpp"
#include "bynav_ros_driver/oem7_message_handler_if.hpp"


namespace bynav_ros_driver
{
  /**
   * Encapsulates a collection of message handling plugins, where a message
   * a messages is handled by 0 or more plugins, matching the message on ID.
   */
  class MessageHandler
  {
    pluginlib::ClassLoader<bynav_ros_driver::Oem7MessageHandlerIf> msg_handler_loader_; ///< Plugin loader

    typedef std::shared_ptr<bynav_ros_driver::Oem7MessageHandlerIf> MessageHandlerIf;
    typedef std::list<MessageHandlerIf>                                MsgHandlerIfList;
    typedef std::pair<MessageHandlerIf, unsigned int>                  MessageHandlerRecord;
    typedef std::list<MessageHandlerRecord>                            MsgHandlerRecordList;
    typedef std::map<int, std::unique_ptr<MsgHandlerRecordList>>       MessageHandlerMap;
    
    rclcpp::Node& node_;

    MsgHandlerIfList    msg_handler_list_; ///< All message handlers
    MessageHandlerMap   msg_handler_map_; ///< Dispatch map for raw messages.

    unsigned int msg_filter_; ///< Mask of all mesages to passed to handlers

  public:
    MessageHandler(rclcpp::Node& nh);

    void handleMessage(Oem7RawMessageIf::ConstPtr raw_msg);

    void setMessageFilter(unsigned int filter);
  };
}

#endif
