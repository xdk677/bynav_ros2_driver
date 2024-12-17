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

#include <rclcpp/rclcpp.hpp>

#include <mutex>
#include <condition_variable>
#include <map>
#include <algorithm>
#include <thread>

#include <boost/asio.hpp>

#include "novatel_oem7_msgs/srv/oem7_abascii_cmd.hpp"
#include "novatel_oem7_msgs/msg/oem7_raw_msg.h"

#include <pluginlib/class_loader.hpp>

#include <bynav_ros_driver/oem7_receiver_if.hpp>
#include <bynav_ros_driver/oem7_message_decoder_if.hpp>

#include <bynav_ros_driver/oem7_message_util.hpp>
#include <oem7_ros_publisher.hpp>

#include <message_handler.hpp>
#include <driver_parameter.hpp>

#include "novatel_oem7_msgs/msg/oem7_header.hpp"

#include "bynav_ros_driver/tcp_client.hpp"

namespace {

void genBaseEncode(char *base64_str, const unsigned char *byte, int32_t n) {
  const char table[] =
      "ABCDEFGHIJKLMNOPQRSTUVWXYZabcdefghijklmnopqrstuvwxyz0123456789+/";
  int i, j, k, b;

  for (i = j = 0; i / 8 < n;) {
    for (k = b = 0; k < 6; ++k, ++i) {
      b <<= 1;
      if (i / 8 < n) {
        b |= (byte[i / 8] >> (7 - i % 8)) & 0x1;
      }
    }
    base64_str[j++] = table[b];
  }

  while (j & 0x3) {
    base64_str[j++] = '=';
  }
  base64_str[j] = '\0';
}

std::string genNtripAuthorizeStr(const std::string& username,
                                 const std::string& passwd,
                                 const std::string& mount_point) {
  char enc[512] = {0};
  char base64_str[512] = {0};
  if (username.size() > 128 || passwd.size() > 256) {
    return "";
  }

  sprintf(enc, "%s:%s", username.c_str(), passwd.c_str());
  genBaseEncode(base64_str, (const unsigned char *)enc, strlen(enc));

  char buff[1024];
  sprintf(buff,
          "GET /%s HTTP/1.0\r\n"
          "User-Agent: NTRIP connect\r\n"
          "Authorization: Basic %s\r\n\r\n",
          mount_point.c_str(), base64_str);

  return std::string(buff);
}

constexpr auto kUpdateGap = 20;  // s

}

namespace bynav_ros_driver
{
  typedef std::vector<std::string> init_cmds_t; ///< List of initialization commands.

  /**
   * @return true if the string has the specified prefix
   */
  bool isPrefix(const std::string& prefix, const std::string& str)
  {
     auto const diff_pos = std::mismatch(prefix.begin(), prefix.end(), str.begin());
     return diff_pos.first == prefix.end();
  }


  /**
   * Node publishing raw oem7 messages and issuing oem7 commands.
   * Loads plugins responsible for obtaining byte input from the Oem7 receiver, and decoding it into raw oem7 messages.
   * Implements a service allowing Oem7 Abbreviated ASCII commands to be sent to the receiver.
   */
  class OemMessageNode :
      public Oem7MessageDecoderUserIf,
      public rclcpp::Node
  {
    enum class NtripAuthStatus { kAuthOk = 0, kAuthFailed };

    std::mutex node_mtx_; ///< Protects node internal state

    double publish_delay_sec_; ///< Delay after publishing each message; used to throttle output with static data sources.

    Oem7RosPublisher<novatel_oem7_msgs::msg::Oem7RawMsg> oem7rawmsg_pub_; ///< Publishes raw Oem7 messages.
    bool publish_unknown_oem7raw_; ///< Publish all unknown messages to 'Oem7Raw'


    rclcpp::CallbackGroup::SharedPtr msg_service_cb_grp_; ///< Message service callbacks
    rclcpp::CallbackGroup::SharedPtr cmd_service_cb_grp_; ///< Command service callbacks

    rclcpp::Service<novatel_oem7_msgs::srv::Oem7AbasciiCmd>::SharedPtr oem7_abascii_cmd_srv_;

    // Command service


    std::condition_variable rsp_ready_cond_; ///< Response ready, signaled from response handler to Oem7 Cmd Handler.
    std::mutex              rsp_ready_mtx_; ///< Response condition guard
    std::string rsp_; ///< The latest response from Oem7 receiver.


    rclcpp::TimerBase::SharedPtr timer_; ///< One time service callback.
    rclcpp::TimerBase::SharedPtr recvr_init_timer_; ///< One time init callback.

    pluginlib::ClassLoader<bynav_ros_driver::Oem7ReceiverIf> recvr_loader_;
    pluginlib::ClassLoader<bynav_ros_driver::Oem7MessageDecoderIf> oem7_msg_decoder_loader;

    std::set<long> raw_msg_pub_; ///< Set of raw messages to publish.

    std::unique_ptr<MessageHandler> msg_handler_; ///< Dispatches individual messages for handling.

    // Log statistics
    long total_log_count_; ///< Total number of logs received

    typedef std::map<int, long> log_count_map_t;
    log_count_map_t log_counts_; ///< Indidividual log counts.
    
    long unknown_msg_num_;   ///< number of messages received that could not be identified.
    long discarded_msg_num_; ///< Number of messages received and discarded by the driver.


    std::shared_ptr<bynav_ros_driver::Oem7MessageDecoderIf> msg_decoder; ///< Message Decoder plugin
    std::shared_ptr<bynav_ros_driver::Oem7ReceiverIf> recvr_; ///< Oem7 Receiver Interface plugin
  
    bool rcvr_init_strict_; ///< When TRUE, publish messages other than oem7raw only after successful receiver initialization.
                            ///< This prevents arbitrary (partial, stale) receiver configurations from generating messages.

    int rcvr_init_num_errors_; ///< Error count during receiver initialization

    std::atomic_bool is_running_;
    std::thread rtcm_data_thread_;
    std::string gpgga_data_;
    std::shared_ptr<bynav_ros_driver::TcpClient> ntrip_cli_;
    NtripAuthStatus auth_status_;
    std::chrono::steady_clock::time_point current_time_;
    std::chrono::steady_clock::time_point last_time_;

    template<typename T>
    T getNtripParam(const std::string& key, const T &default_val) {
      declare_parameter<T>("NTRIP." + key, default_val);
      T res;
      get_parameter("NTRIP." + key, res);
      return res;
    }

    bool enableNtrip() {
      auto enable = getNtripParam<int>("enable", 0);
      return enable == 1;
    }

    void initNtripClient() {
      auto ip = getNtripParam<std::string>("ip", "");
      if (ip.empty()) {
        ip = "127.0.0.1";
      }

      auto port = getNtripParam<int>("port", 0);
      if (port == 0) {
        port = 8888;
      }

      RCLCPP_INFO_STREAM(get_logger(), "Ntrip connection info (" + ip + ":" +
                                           std::to_string(port) + ")");

      ntrip_cli_.reset(new TcpClient(this, ip, port));
      ntrip_cli_->SetNonBlockMode();
    }

    void authorizeNtrip() {
      if (auth_status_ == NtripAuthStatus::kAuthOk) {
        return;
      }

      static auto username = getNtripParam<std::string>("username", "");
      static auto password = getNtripParam<std::string>("password", "");
      static auto mount_point = getNtripParam<std::string>("mountpoint", "");

      RCLCPP_INFO_STREAM(get_logger(), "auth info: " << username << ", "
                                                     << password << ", "
                                                     << mount_point);
      std::string auth_str =
          genNtripAuthorizeStr(username, password, mount_point);
      ntrip_cli_->SendData(auth_str);

      sleep(1);

      int8_t auth_retry_cnt = 3;
      while (auth_retry_cnt--) {
        char buf[1024];
        int32_t ret = ntrip_cli_->RecvData(buf, sizeof(buf));
        if (ret == -1) {
          goto fail;
        }

        std::string resp(buf);
        if (resp.find("ICY 200 OK") != std::string::npos) {
          RCLCPP_INFO_STREAM(get_logger(), "Auth ok");
          auth_status_ = NtripAuthStatus::kAuthOk;
          return;
        }

        usleep(10000);
      }

    fail:
      auth_status_ = NtripAuthStatus::kAuthFailed;
      RCLCPP_ERROR_STREAM(get_logger(), "Auth failed");
    }

    void HandleRTCMData() {
      if (!ntrip_cli_) {
        return;
      }

      is_running_.store(true, std::memory_order_relaxed);
      RCLCPP_INFO_STREAM(get_logger(), "handle rtcm start");
      while (is_running_.load(std::memory_order_relaxed)) {
        usleep(1000);

        if (!ntrip_cli_->IsConnected() ||
            ntrip_cli_->HasLostConnection()) {
          auth_status_ = NtripAuthStatus::kAuthFailed;
          int32_t ret = ntrip_cli_->ConnectToServer();
          if (ret == -1) {
            RCLCPP_ERROR_STREAM(get_logger(), "connect to ntrip server failed");
            sleep(2);
            continue;
          }

          authorizeNtrip();
        }

        if (auth_status_ == NtripAuthStatus::kAuthFailed) {
          ntrip_cli_->DisconnectFromServer();
          continue;
        }

        const auto& rtcm_data = ntrip_cli_->ReadAll();
        if (!rtcm_data.empty() && recvr_) {
          recvr_->write(boost::asio::buffer(rtcm_data));
        }
      }

      RCLCPP_INFO_STREAM(get_logger(), "handle rtcm end");
    }

    void extractGPGGAData(Oem7RawMessageIf::ConstPtr raw_msg) {
      std::string gpgga_data(
          raw_msg->getMessageData(0),
          raw_msg->getMessageData(raw_msg->getMessageDataLength()));
      if (gpgga_data.find("GPGGA") == std::string::npos) {
        return;
      }

      current_time_ = std::chrono::steady_clock::now();
      std::chrono::duration<double> time_gap =
          std::chrono::duration_cast<std::chrono::duration<double>>(
              current_time_ - last_time_);

      if (gpgga_data_.empty() || time_gap.count() > kUpdateGap) {
        RCLCPP_DEBUG_STREAM(get_logger(), "GPGGA: " << gpgga_data);
        gpgga_data_ = gpgga_data;
        last_time_ = current_time_;

        if (ntrip_cli_ && ntrip_cli_->IsConnected()) {
          ntrip_cli_->SendData(gpgga_data);
        }
      }
    }

    /**
     * Loads plugins, sets up threading/callbacks, advertises messages and services.
     */
    void construct()
    {
      //auto e  = rcutils_logging_set_logger_level(
      //        get_logger().get_name(), RCUTILS_LOG_SEVERITY_DEBUG);

      std::lock_guard<std::mutex> guard(node_mtx_);

      current_time_ = std::chrono::steady_clock::steady_clock::now();
      last_time_ = std::chrono::steady_clock::steady_clock::now();

      initializeOem7MessageUtil(*this);


      declare_parameter<bool>("oem_strict_receiver_init", true);
      get_parameter("oem_strict_receiver_init", rcvr_init_strict_);
      
      declare_parameter<bool>("oem_publish_unknown_oem7raw", false);
      get_parameter("oem_publish_unknown_oem7raw", publish_unknown_oem7raw_);

      declare_parameter<double>("oem_publish_delay", 0.0);
      get_parameter("oem_publish_delay", publish_delay_sec_);
      if(publish_delay_sec_ > 0)
      {
        RCLCPP_WARN_STREAM(get_logger(), "Publish Delay: " << publish_delay_sec_ << " seconds. Is this is a test?");
      }

      // Load plugins

      // Load Oem7Receiver
      declare_parameter("oem7_if", "");
      rclcpp::Parameter oem7_if_param = get_parameter("oem7_if");
      recvr_ = recvr_loader_.createSharedInstance(oem7_if_param.as_string());
      recvr_->initialize(*this);

      // Load Oem7 Message Decoder
      declare_parameter("oem7_msg_decoder", "");
      rclcpp::Parameter msg_decoder_param = get_parameter("oem7_msg_decoder");
      msg_decoder = oem7_msg_decoder_loader.createSharedInstance(msg_decoder_param.as_string());
      msg_decoder->initialize(*this, recvr_.get(), this);

      msg_handler_ = std::make_unique<MessageHandler>(*this);

      // Oem7 raw messages to publish.
      std::vector<long> init_msg = {0, 0};
      declare_parameter("oem7_raw_msgs", init_msg);
      rclcpp::Parameter oem7_raw_msgs_param = get_parameter("oem7_raw_msgs");

      std::vector<long> raw_msg = oem7_raw_msgs_param.as_integer_array();
      for(const auto& msg : raw_msg)
      {
        if(raw_msg_pub_.find(msg) == raw_msg_pub_.end())
        {
          raw_msg_pub_.insert(msg);

          RCLCPP_INFO_STREAM(get_logger(), "Oem7 Raw message '" << msg << "' will be published." );
        }
        else
        {
          RCLCPP_WARN_STREAM(get_logger(), "Oem7 Raw message '" << msg << "' duplicate specified." );
        }
      }

      msg_service_cb_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);
      cmd_service_cb_grp_ = this->create_callback_group(rclcpp::CallbackGroupType::MutuallyExclusive);


      timer_ = create_wall_timer(
                std::chrono::milliseconds(1),
                std::bind(
                    &OemMessageNode::serviceLoopCb,
                    this),
                    msg_service_cb_grp_);


      recvr_init_timer_ = create_wall_timer(
                std::chrono::milliseconds(50),
                std::bind(
                    &OemMessageNode::initCb,
                    this),
                    cmd_service_cb_grp_);

      if (enableNtrip()) {
        initNtripClient();
        rtcm_data_thread_ = std::thread(&OemMessageNode::HandleRTCMData, this);
      } else {
        RCLCPP_INFO_STREAM(get_logger(), "ntrip not enabled");
      }
      // CMD service is created in initCb(), after our own receiver initialization is complete.
      // This ensure that external commands do not interfere.
    }


    /**
     * Called to request O7AbasciiCmd service
     */
    void
    serviceOem7AbasciiCb(
          const std::shared_ptr<novatel_oem7_msgs::srv::Oem7AbasciiCmd::Request>  req,
                std::shared_ptr<novatel_oem7_msgs::srv::Oem7AbasciiCmd::Response> rsp)
    {
      issueOem7AbasciiCmd(req->cmd, rsp->rsp);
    }

    void issueOem7AbasciiCmd(
        const std::string& cmd,
        std::string& rsp)
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "AACmd: cmd '" << cmd << "'");

      // Retry sending the commands. For configuration commands, there is no harm in duplicates.
      for(int attempt = 0;
              attempt < 10;
              attempt++)
      {
        {
          std::lock_guard<std::mutex> lk(rsp_ready_mtx_);
    	    rsp_.clear();
    	  }

        recvr_->write(boost::asio::buffer(cmd));
        static const std::string NEWLINE("\n");
        recvr_->write(boost::asio::buffer(NEWLINE));

        std::unique_lock<std::mutex> lk(rsp_ready_mtx_);
        if(rsp_ready_cond_.wait_until(lk,
                                      std::chrono::steady_clock::now() + std::chrono::seconds(3)
                                      ) == std::cv_status::no_timeout)
        {
          rsp = rsp_;
          break;
        }

        RCLCPP_ERROR_STREAM(get_logger(), "AACmd '" << cmd << "': Attempt " << attempt << ": timed out waiting for response.");
      }

      if(rsp == "OK")
      {
        RCLCPP_INFO_STREAM(get_logger(), "AACmd '" << cmd << "' : " << "'" << rsp << "'");
      }
      else
      {
        RCLCPP_ERROR_STREAM(get_logger(), "AACmd '" << cmd << "' : " << "'" << rsp << "'");

        rcvr_init_num_errors_++;
      }
    }

    /**
     * Outputs Log statistics to ROS console.
     */
    void outputLogStatistics()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Log Statistics:");
      RCLCPP_INFO_STREAM(get_logger(), "Logs: " << total_log_count_ << "; unknown: "   << unknown_msg_num_
                                                       << "; discarded: " << discarded_msg_num_);

      for(log_count_map_t::iterator itr = log_counts_.begin();
                                    itr != log_counts_.end();
                                    itr++)
      {
        int  id    = itr->first;
        long count = itr->second;

        RCLCPP_INFO_STREAM(get_logger(), "Log[" << id << "]:" <<  count);
      }
    }

    /*
     * Update Log statistics for a particular message
     */
    void updateLogStatistics(const Oem7RawMessageIf::ConstPtr& raw_msg)
    {
      total_log_count_++;

      if(log_counts_.find(raw_msg->getMessageId()) == log_counts_.end())
      {
        log_counts_[raw_msg->getMessageId()] = 0;
      }
      log_counts_[raw_msg->getMessageId()]++;


      assert(total_log_count_ > 0);
      if((total_log_count_ % 10000) == 0)
      {
        outputLogStatistics();
      }
    }

    void publishOem7RawMsg(Oem7RawMessageIf::ConstPtr raw_msg)
    {
        novatel_oem7_msgs::msg::Oem7RawMsg::SharedPtr oem7_raw_msg = std::make_shared<novatel_oem7_msgs::msg::Oem7RawMsg>();
        oem7_raw_msg->message_data.insert(
                                        oem7_raw_msg->message_data.end(),
                                        raw_msg->getMessageData(0),
                                        raw_msg->getMessageData(raw_msg->getMessageDataLength()));

        assert(oem7_raw_msg->message_data.size() == raw_msg->getMessageDataLength());

        oem7rawmsg_pub_.publish(oem7_raw_msg);
    } 


   /**
     * Called by ROS decoder with new raw messages
     */
    void onNewMessage(Oem7RawMessageIf::ConstPtr raw_msg)
    {
      RCLCPP_DEBUG_STREAM(get_logger(),
                           ">Raw[ID: " << raw_msg->getMessageId()         <<
                                " T: " << raw_msg->getMessageType()       <<
                                " F: " << raw_msg->getMessageFormat()     <<
                                " L: " << raw_msg->getMessageDataLength() <<
                                "]");

      // Discard all unknown messages; this is normally when dealing with responses to ASCII commands.
      if(raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_UNKNOWN)
      {
        ++unknown_msg_num_;

        if(publish_unknown_oem7raw_)
        {
            publishOem7RawMsg(raw_msg);
        }
        else
        {
            ++discarded_msg_num_;
        }
      }
      else
      {
        if(raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ASCII  ||
           raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ABASCII)
        {
          extractGPGGAData(raw_msg);
          std::string msg(raw_msg->getMessageData(0), raw_msg->getMessageData(raw_msg->getMessageDataLength()));
          RCLCPP_DEBUG_STREAM(get_logger(),
                            ">---------------------------" << std::endl
                            << msg                         << std::endl
                            << "<---------------------------");
        }

        if(raw_msg->getMessageType() == Oem7RawMessageIf::OEM7MSGTYPE_RSP) // Response
        {
          std::string rsp(raw_msg->getMessageData(0), raw_msg->getMessageData(raw_msg->getMessageDataLength()));
          if(rsp.find_first_not_of(" /t/r/n") != std::string::npos /* && // ignore all-whitespace responses
             std::all_of(rsp.begin(), rsp.end(), [](char c){ return std::isprint(c);} )*/) // Ignore any corrupt responses
          {
            {
              std::lock_guard<std::mutex> lk(rsp_ready_mtx_);
              rsp_ = rsp;
            }
            rsp_ready_cond_.notify_one();
          }
          else
          {
            RCLCPP_ERROR_STREAM(get_logger(), "Discarded corrupt ASCII response: '" << rsp << "'");
          }
        }
        else // Log
        {
          if( raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_SHORTBINARY ||
              raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_BINARY       ||
             (raw_msg->getMessageFormat() == Oem7RawMessageIf::OEM7MSGFMT_ASCII && isNMEAMessage(raw_msg)))
          {
            updateLogStatistics(raw_msg);
            msg_handler_->handleMessage(raw_msg);
      
            // Publish raw messages regardless; they are all for debugging.

            // Publish Oem7RawMsg if specified
            if(raw_msg_pub_.find(raw_msg->getMessageId()) != raw_msg_pub_.end())
            {
                publishOem7RawMsg(raw_msg);
            }

            if(publish_delay_sec_ > 0)
            {
              sleep(publish_delay_sec_);
            }
          }
        }
      }
    }


    /**
     * Service loop; drives Oem7 message decoder. onNewMessage is called on this thread.
     */

    void serviceLoopCb()
    {
      RCLCPP_DEBUG_STREAM(get_logger(), "Service" );

      timer_->cancel(); // One-time entry into service loop.

      msg_decoder->service();

      outputLogStatistics();

      RCLCPP_WARN_STREAM(get_logger(), "No more input from Decoder; OemMessageNode finished." );
    }

    void initCb()
    {
      RCLCPP_INFO_STREAM(get_logger(), "Standard Receiver Initialization:" );

      recvr_init_timer_->cancel();

      // When strict mode enabled, disallow any messages related to position to avoid generation of position based on wrong config.
      msg_handler_->setMessageFilter(rcvr_init_strict_ ? MSGFLAG_STATUS_OR_CONFIG : MSGFLAG_ALL);

      DriverParameter<init_cmds_t> init_cmds_p(    "receiver_init_commands", init_cmds_t(), *this);
      DriverParameter<init_cmds_t> ext_init_cmds_p("receiver_ext_init_commands", init_cmds_t(), *this);

      for(const auto& cmd : init_cmds_p.value())
      {
        issueConfigCmd(cmd);
      }

      RCLCPP_INFO_STREAM(get_logger(), "Extended Receiver Initialization:" );
      for(const auto& cmd : ext_init_cmds_p.value())
      {
        issueConfigCmd(cmd);
      }

      RCLCPP_INFO_STREAM(get_logger(), "Receiver Initialization completed, errors= " << rcvr_init_num_errors_ );

      if(rcvr_init_num_errors_ == 0)
      {
        msg_handler_->setMessageFilter(MSGFLAG_ALL); // Allow all messages
      }
      else if(rcvr_init_strict_)
      {
        RCLCPP_INFO_STREAM(get_logger(),
                           "Receiver Initialization completed with errors; 'Strict' mode is on; no Position / Odoemetry / INS output." );
      }

      // Allow command entry via service for diagnostics, regardless of init status.

      // Now that all internal init commands have been issued, allow external commands:
      static rmw_qos_profile_t qos = rmw_qos_profile_default;
      qos.depth = 20;
      oem7_abascii_cmd_srv_ = create_service<novatel_oem7_msgs::srv::Oem7AbasciiCmd>(
                               "Oem7Cmd",
                               std::bind(
                                      &OemMessageNode::serviceOem7AbasciiCb,
                                     this,
                                     std::placeholders::_1,
                                     std::placeholders::_2),
                                     qos,
                               cmd_service_cb_grp_);
    }

     /**
     * Executes Driver-specific command, like PAUSE.
     *
     * @return true when the provided command is a recongized internal command.
     */
      bool executeInternalCommand(const std::string& cmd)
      {
        static const std::string CMD_PAUSE("!PAUSE");
        if(isPrefix(CMD_PAUSE, cmd))
        {
           std::stringstream ss(cmd);
           std::string token;
           ss >> token; // Prefix
           ss >> token; // Period
           int pause_period_sec = 0;
           if(std::stringstream(token) >> pause_period_sec)
           {
              RCLCPP_INFO_STREAM(get_logger(), "Sleeping for " << pause_period_sec << " seconds....");
              std::chrono::seconds sleep_dur( pause_period_sec );
              std::this_thread::sleep_for( sleep_dur );
              RCLCPP_INFO_STREAM(get_logger(), "... done sleeping.");
           }
           else
           {
              RCLCPP_ERROR_STREAM(get_logger(), "Invalid Driver command syntax: '" << cmd << "'");
           }

           return true;
        }
        else // Not a recognized internal command
        {
           return false;
        }
      }

      /**
       * Issues Oem7 configuration command
       */
      void issueConfigCmd(const std::string& cmd /**< The command to issue */)
      {
        if(executeInternalCommand(cmd))
        {
          return;
        }

        std::string rsp;
        issueOem7AbasciiCmd(cmd, rsp);
        // FIXME: check for response and signal diagnostics
      }

 public:

    OemMessageNode(const rclcpp::NodeOptions& options):
      rclcpp::Node("OemMessage", options),
      oem7rawmsg_pub_("Oem7RawMsg", *this),
      recvr_loader_(          "bynav_ros_driver", "bynav_ros_driver::Oem7ReceiverIf"),
      oem7_msg_decoder_loader("bynav_ros_driver", "bynav_ros_driver::Oem7MessageDecoderIf"),
      total_log_count_(0),
      unknown_msg_num_(0),
      discarded_msg_num_(0),
      publish_delay_sec_(0),
      publish_unknown_oem7raw_(false),
      rcvr_init_strict_(true),
      rcvr_init_num_errors_(0),
      is_running_{false},
      auth_status_(NtripAuthStatus::kAuthFailed)
    {
      construct();
    }
  };
}


#include <rclcpp_components/register_node_macro.hpp>
RCLCPP_COMPONENTS_REGISTER_NODE(bynav_ros_driver::OemMessageNode)


int main(int argc, char* argv[])
{
  rclcpp::init(argc, argv);

  rclcpp::NodeOptions options;
  auto oem7 = std::make_shared<bynav_ros_driver::OemMessageNode>(options);

  static const size_t THREAD_NUM = 3; // Default + Receive and blocking command service.
  rclcpp::ExecutorOptions defaultOptions;
  rclcpp::executors::MultiThreadedExecutor executor(defaultOptions, THREAD_NUM);
  executor.add_node(oem7);
  executor.spin();
  rclcpp::shutdown();
  return 0;
}
