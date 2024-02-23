#define LIBCAER_FRAMECPP_OPENCV_INSTALLED 0
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "ev_msgs/msg/event.hpp"
#include "ev_msgs/msg/event_array.hpp"
#include "rmw/qos_profiles.h"
#include <libcaercpp/libcaer.hpp>
#include <libcaercpp/filters/dvs_noise.hpp>
#include <libcaercpp/events/special.hpp>
#include <libcaercpp/devices/davis.hpp>
#include "libcaer/filters/dvs_noise.h"
#include <sensor_msgs/msg/image.hpp>
#include <opencv2/opencv.hpp> 
#include <opencv2/core.hpp> 
#include <opencv2/highgui/highgui.hpp> 
#include <opencv2/imgproc.hpp> 
#include <cv_bridge/cv_bridge.h> 
#include <atomic>
#include <csignal>
#include <chrono>
#include <functional>
#include <memory>

// USB Shutdown handler
/**
 * @brief USB Shutdown handler
 * @param ptr 
 */
static void usbShutdownHandler(void *ptr);
    
class DavisPublisher : public rclcpp::Node
{
    public:
        /**
         * @brief Construct a new Davis Publisher object
         * 
         */
        DavisPublisher();

        /**
         * @brief Destroy the Davis Publisher object
         */
        ~DavisPublisher();

        /**
         * @brief Callback function for the Dynamic Reconfigure
         * @param parameters 
         * @return rcl_interfaces::msg::SetParametersResult 
         */
        rcl_interfaces::msg::SetParametersResult parameter_change_callback(const std::vector<rclcpp::Parameter> parameters);

    private:
        /**
         * @brief Callback function for the timer that publishes the events
         */
        void packet_callback();

        /**
         * @brief Compute Exposure for the frame(/davis/image_raw) 
         * 
         * @param cvFrame 
         * @param current_exposure 
         * @return int new_exposure
         */
        int computeNewExposure(const cv::Mat &cvFrame, const int current_exposure);

        std::unique_ptr<libcaer::events::EventPacketContainer> packetContainer_; 
        rclcpp::Publisher<ev_msgs::msg::EventArray>::SharedPtr pub_event_;
        rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr  pub_image_;
        libcaer::devices::davis davisHandle_ = libcaer::devices::davis(1);
        rclcpp::TimerBase::SharedPtr ev_timer_;
        rclcpp::TimerBase::SharedPtr frame_timer_;
        struct caer_davis_info davis_info_;
        struct caer_bias_coarsefine coarseFineBias_;
        size_t       count_;
        uint32_t     prBias;
        uint32_t     prsfBias;
        std::string  event_topic_;
        std::string  image_topic_;
        std::string  frame_id_;
        rclcpp::Time reset_timestamp_;
        OnSetParametersCallbackHandle::SharedPtr reset_param_handler_;


};