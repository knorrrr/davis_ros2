#include "davis_driver/davis_driver.hpp"

using namespace std;
using namespace std::chrono_literals;
static void usbShutdownHandler(void *ptr) {
	(void) (ptr);
  RCLCPP_ERROR(rclcpp::get_logger("rclcpp"), "USB Disconnected");
}

DavisPublisher::DavisPublisher()
: Node("davis_driver_node"), count_(0)
{
  // declare_parameter
  this->declare_parameter("frame_id",    "davis");
  this->declare_parameter<int> ("max_container_interval", 500);// micro sec
  this->declare_parameter<int> ("max_container_packet_size", 4098); 
  this->declare_parameter<int> ("usb_early_packet_delay", 1);
  this->declare_parameter<bool>("datachange_blocking", true);
  this->declare_parameter<bool>("enable_image_raw", true);
  this->declare_parameter<bool>("enable_events", true);
  this->declare_parameter<bool>("enable_imu6", false);
  this->declare_parameter<bool>("enable_noise_filter", false);
  this->declare_parameter<bool>("enable_autoexposure", true);
  this->declare_parameter<int> ("autoexposure_desired_intensity", 40000);
  //PRBP
  this->declare_parameter<int> ("PRBP.coarseValue", 2);
  this->declare_parameter<int> ("PRBP.fineValue", 58);
  this->declare_parameter<bool>("PRBP.enabled", true);
  this->declare_parameter<bool>("PRBP.sexN", false);
  this->declare_parameter<bool>("PRBP.typeNormal", true);
  this->declare_parameter<bool>("PRBP.currentLevelNormal", true);
  //PRSFBP
  this->declare_parameter<int> ("PRSFBP.coarseValue", 1);
  this->declare_parameter<int> ("PRSFBP.fineValue", 33);
  this->declare_parameter<bool>("PRSFBP.enabled", true);
  this->declare_parameter<bool>("PRSFBP.sexN", false);
  this->declare_parameter<bool>("PRSFBP.typeNormal", true);
  this->declare_parameter<bool>("PRSFBP.currentLevelNormal", true);

  frame_id_    = this->get_parameter("frame_id").get_value<std::string>();

  auto parameter_change_cb = std::bind(&DavisPublisher::parameter_change_callback, this, std::placeholders::_1);
  reset_param_handler_     = this->add_on_set_parameters_callback(parameter_change_cb);

  pub_event_ = create_publisher<ev_msgs::msg::EventArray>("~/output/event" ,rclcpp::SensorDataQoS());
  pub_image_ = create_publisher<sensor_msgs::msg::Image> ("~/output/image", rclcpp::SensorDataQoS());

  #ifndef NDEBUG
  libcaer::log::logLevelSet(libcaer::log::logLevel::DEBUG);
  #endif

  ev_timer_ = this->create_wall_timer(1ns, std::bind(&DavisPublisher::packet_callback, this));

  davis_info_ = davisHandle_.infoGet();
  RCLCPP_INFO(this->get_logger(), "%s : ID: %d, Serial Number: %s, USB Bus: %d, USB Device Address: %d", 
      davis_info_.deviceString, davis_info_.deviceID, davis_info_.deviceSerialNumber, davis_info_.deviceUSBBusNumber, davis_info_.deviceUSBDeviceAddress);
  RCLCPP_INFO(this->get_logger(), "Firmware Version: %d, Logic Version: %d, Chip ID: %d, Master: %d, Mux Statistics: %d", 
      davis_info_.firmwareVersion, davis_info_.logicVersion, davis_info_.chipID, davis_info_.deviceIsMaster, davis_info_.muxHasStatistics);
  RCLCPP_INFO(this->get_logger(), "DVS X: %d, DVS Y: %d, DVS Pixel Filter: %d, DVS Background Activity Filter: %d, DVS ROI Filter: %d", 
      davis_info_.dvsSizeX, davis_info_.dvsSizeY, davis_info_.dvsHasPixelFilter, davis_info_.dvsHasBackgroundActivityFilter, davis_info_.dvsHasROIFilter);
  RCLCPP_INFO(this->get_logger(), "DVS Skip Filter: %d, DVS Polarity Filter: %d, DVS Statistics: %d", 
      davis_info_.dvsHasSkipFilter, davis_info_.dvsHasPolarityFilter, davis_info_.dvsHasStatistics);
  RCLCPP_INFO(this->get_logger(), "APS X: %d, APS Y: %d, APS Color Filter: %d, APS Global Shutter: %d", 
      davis_info_.apsSizeX, davis_info_.apsSizeY, davis_info_.apsColorFilter,  davis_info_.apsHasGlobalShutter);
  RCLCPP_INFO(this->get_logger(), "IMU Type: %d, Ext Input Generator: %d", davis_info_.imuType, davis_info_.extInputHasGenerator);

  davisHandle_.sendDefaultConfig();
  // Reset timestamps
  davisHandle_.configSet(DAVIS_CONFIG_MUX, DAVIS_CONFIG_MUX_TIMESTAMP_RESET, true);
  reset_timestamp_ = rclcpp::Clock(RCL_ROS_TIME).now();

  RCLCPP_INFO(this->get_logger(), "Reset Timestamps");

  coarseFineBias_.coarseValue        = this->get_parameter("PRBP.coarseValue").get_value<int>();
  coarseFineBias_.fineValue          = this->get_parameter("PRBP.fineValue").get_value<int>();
  coarseFineBias_.enabled            = this->get_parameter("PRBP.enabled").get_value<bool>();
  coarseFineBias_.sexN               = this->get_parameter("PRBP.sexN").get_value<bool>();
  coarseFineBias_.typeNormal         = this->get_parameter("PRBP.typeNormal").get_value<bool>();
  coarseFineBias_.currentLevelNormal = this->get_parameter("PRBP.currentLevelNormal").get_value<bool>();
  davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));

  coarseFineBias_.coarseValue        = this->get_parameter("PRSFBP.coarseValue").get_value<int>();
  coarseFineBias_.fineValue          = this->get_parameter("PRSFBP.fineValue").get_value<int>();
  coarseFineBias_.enabled            = this->get_parameter("PRSFBP.enabled").get_value<bool>();
  coarseFineBias_.sexN               = this->get_parameter("PRSFBP.sexN").get_value<bool>();
  coarseFineBias_.typeNormal         = this->get_parameter("PRSFBP.typeNormal").get_value<bool>(); 
  coarseFineBias_.currentLevelNormal = this->get_parameter("PRSFBP.currentLevelNormal").get_value<bool>();
  davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));

  //Data Start
  davisHandle_.dataStart(nullptr, nullptr, nullptr, &usbShutdownHandler, nullptr);
  //These parameters are set AFTER data start
  davisHandle_.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, this->get_parameter("enable_image_raw").get_value<bool>());
  davisHandle_.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, this->get_parameter("enable_events").get_value<bool>());
  davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, this->get_parameter("enable_imu6").get_value<bool>());
  davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE,     this->get_parameter("enable_imu6").get_value<bool>());
  davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE,   this->get_parameter("enable_imu6").get_value<bool>());
  davisHandle_.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, this->get_parameter("datachange_blocking").get_value<bool>());
  davisHandle_.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, this->get_parameter("max_container_interval").get_value<int>());
  davisHandle_.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE, this->get_parameter("max_container_packet_size").get_value<int>());
}


DavisPublisher::~DavisPublisher(){ 
  davisHandle_.dataStop();
  printf("Shutdown successful.\n");
}

rcl_interfaces::msg::SetParametersResult DavisPublisher::parameter_change_callback(const std::vector<rclcpp::Parameter> parameters){
  auto result = rcl_interfaces::msg::SetParametersResult();
  result.successful = true;
  for (auto parameter : parameters){
    if (parameter.get_name() == "frame_id"){
      frame_id_ = parameter.as_string();
    }
    else if (parameter.get_name() == "max_container_interval"){
      davisHandle_.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_INTERVAL, parameter.as_int());
    }
    else if(parameter.get_name() == "max_container_packet_size"){
      davisHandle_.configSet(CAER_HOST_CONFIG_PACKETS, CAER_HOST_CONFIG_PACKETS_MAX_CONTAINER_PACKET_SIZE, parameter.as_int());
    }
    else if (parameter.get_name() == "usb_early_packet_delay"){
      davisHandle_.configSet(DAVIS_CONFIG_USB, DAVIS_CONFIG_USB_EARLY_PACKET_DELAY, parameter.as_int());
    }
    else if (parameter.get_name() == "datachange_blocking"){
      davisHandle_.configSet(CAER_HOST_CONFIG_DATAEXCHANGE, CAER_HOST_CONFIG_DATAEXCHANGE_BLOCKING, parameter.as_bool());
    }
    else if (parameter.get_name() == "enable_image_raw"){
      davisHandle_.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_RUN, parameter.as_bool());
    }
    else if (parameter.get_name() == "enable_events"){
      davisHandle_.configSet(DAVIS_CONFIG_DVS, DAVIS_CONFIG_DVS_RUN, parameter.as_bool());
    }
    else if (parameter.get_name() == "enable_imu6"){
      davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_ACCELEROMETER, parameter.as_bool());
      davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_GYROSCOPE,     parameter.as_bool());
      davisHandle_.configSet(DAVIS_CONFIG_IMU, DAVIS_CONFIG_IMU_RUN_TEMPERATURE,   parameter.as_bool());
    }
    else if (parameter.get_name() == "enable_noise_filter"){
      libcaer::filters::DVSNoise dvsNoiseFilter = libcaer::filters::DVSNoise(davis_info_.dvsSizeX, davis_info_.dvsSizeX);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TWO_LEVELS, true);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_CHECK_POLARITY, true);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MIN, 2);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_SUPPORT_MAX, 8);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_TIME, 2000);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_BACKGROUND_ACTIVITY_ENABLE, true);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_REFRACTORY_PERIOD_TIME, 200);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_REFRACTORY_PERIOD_ENABLE, true);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_HOTPIXEL_ENABLE, true);
     dvsNoiseFilter.configSet(CAER_FILTER_DVS_HOTPIXEL_LEARN, true);
    }
    // PRBP
    else if (parameter.get_name() == "PRBP.coarseValue"){
      coarseFineBias_.coarseValue = parameter.as_int();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRBP.fineValue"){
      coarseFineBias_.fineValue = parameter.as_int();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRBP.enabled"){
      coarseFineBias_.enabled = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRBP.sexN"){
      coarseFineBias_.sexN = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRBP.typeNormal"){
      coarseFineBias_.typeNormal = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRBP.currentLevelNormal"){
      coarseFineBias_.currentLevelNormal = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    // PRSFBP
    else if (parameter.get_name() == "PRSFBP.coarseValue"){
      coarseFineBias_.coarseValue = parameter.as_int();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRSFBP.fineValue"){
      coarseFineBias_.fineValue = parameter.as_int();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRSFBP.enabled"){
      coarseFineBias_.enabled = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRSFBP.sexN"){
      coarseFineBias_.sexN = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRSFBP.typeNormal"){
      coarseFineBias_.typeNormal = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else if (parameter.get_name() == "PRSFBP.currentLevelNormal"){
      coarseFineBias_.currentLevelNormal = parameter.as_bool();
      davisHandle_.configSet(DAVIS_CONFIG_BIAS, DAVIS240_CONFIG_BIAS_PRSFBP, caerBiasCoarseFineGenerate(coarseFineBias_));
    }
    else{
      result.successful = false;
    }
  }
  return result;
}



void DavisPublisher::packet_callback(){
  packetContainer_ = davisHandle_.dataGet();
  if (packetContainer_ == nullptr) {
    // if block mode is enabled, we will never get nullptr
    RCLCPP_DEBUG(this->get_logger(), "No data received, skipping callback.");
    return;
  }
  RCLCPP_DEBUG(this->get_logger(), "Got event container with %d packets.", packetContainer_->size());
  for (auto &packet : *packetContainer_) {
    if (packet == nullptr) {
      RCLCPP_DEBUG(this->get_logger(), "Packet is empty, continuing.");
      continue; 
    }
    //0 :Special, 1;Polarity, 2:Frame, 3 IMU6 | Capacity:: Store the maximum number of events that can be stored in the packet.
    if (packet->getEventType() == POLARITY_EVENT) {
      RCLCPP_DEBUG(this->get_logger(), "Polarity:: %d -> %d events, %d capacity.", packet->getEventType(), packet->getEventNumber(),packet->getEventCapacity());
      std::shared_ptr<const libcaer::events::PolarityEventPacket> polarity = std::static_pointer_cast<libcaer::events::PolarityEventPacket>(packet);
      ev_msgs::msg::EventArray ev_array;
      ev_array.header.stamp = this->get_clock()->now();
      ev_array.header.frame_id = frame_id_;
      ev_array.height = davis_info_.dvsSizeY;
      ev_array.width  = davis_info_.dvsSizeX;
      for (int j = 0; j < packet->getEventNumber(); j++)
      {
        const libcaer::events::PolarityEvent &Event = (*polarity)[j];
        ev_msgs::msg::Event event_msg;
        event_msg.x        = Event.getX();
        event_msg.y        = Event.getY();
        event_msg.polarity = Event.getPolarity();
        event_msg.time     = reset_timestamp_ + rclcpp::Duration::from_nanoseconds(Event.getTimestamp64(*polarity) * 1000);
        ev_array.events.push_back(event_msg);
      }
      pub_event_->publish(ev_array);
    }
    else if (packet->getEventType() == FRAME_EVENT) {
      RCLCPP_DEBUG(this->get_logger(), "Frame:: %d -> %d events, %d capacity.", packet->getEventType(), packet->getEventNumber(),packet->getEventCapacity());
      std::shared_ptr<const libcaer::events::FrameEventPacket> frame = std::static_pointer_cast<libcaer::events::FrameEventPacket>(packet);
      for (const auto &f : *frame) {
        if ((!f.isValid()) || (f.getROIIdentifier() != 0)) {
          continue;
        }
        sensor_msgs::msg::Image img;
        cv_bridge::CvImage cv_img;
        const cv::Size frameSize(f.getLengthX(), f.getLengthY());
        const cv::Mat cvFrame(frameSize, CV_16UC1, reinterpret_cast<void *>(const_cast<uint16_t *>(f.pixels)));
        cv_img.encoding        = "mono16";
        cv_img.image           = cvFrame;
        cv_img.header.stamp    = reset_timestamp_ + rclcpp::Duration::from_nanoseconds(f.getTimestamp64(*frame) * 1000);
        cv_img.header.frame_id = frame_id_;
        cv_img.toImageMsg(img);
        pub_image_ -> publish(img); 
        //Auto Exposure
        if(this->get_parameter("enable_autoexposure").get_value<bool>()){
          uint32_t current_exposure;
          davisHandle_.configGet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, &current_exposure);
          RCLCPP_DEBUG(this->get_logger(), "Current Exposure: %d", current_exposure);
          const int new_exposure = DavisPublisher::computeNewExposure(cvFrame, current_exposure);
          davisHandle_.configSet(DAVIS_CONFIG_APS, DAVIS_CONFIG_APS_EXPOSURE, new_exposure);
          RCLCPP_DEBUG(this->get_logger(), "New Exposure: %d", new_exposure);
        }
      }
    }
    #ifndef NDEBUG
    else if (packet ->getEventType() == IMU6_EVENT){
      RCLCPP_DEBUG(this->get_logger(), "IMU6:: %d -> %d events, %d capacity.", packet->getEventType(), packet->getEventNumber(),packet->getEventCapacity());
    }
    else if (packet ->getEventType() == SPECIAL_EVENT){
      for (int i = 0; i < packet->getEventNumber(); i++){
        libcaer::events::SpecialEvent s = (*special)[i];
        RCLCPP_DEBUG(this->get_logger(), "Special Event: %d", s.getType());
      }
    }
    else {
      RCLCPP_DEBUG(this->get_logger(), "Unknown event type: %d.", packet->getEventType());
    }
    #endif
  }
}
int DavisPublisher::computeNewExposure(const cv::Mat &cvFrame, const int current_exposure){
  const int desired_intensity = this->get_parameter("autoexposure_desired_intensity").get_value<int>();
  const int min_exposure      = 10;
  const int max_exposure      = 25000;
  const float step_size       = 0.01;

  if (cvFrame.empty()){
    return current_exposure;
  }
  const int mean_intensity = cv::mean(cvFrame).val[0];
  RCLCPP_DEBUG(this->get_logger(), "Mean Intensity: %d", mean_intensity);
  int delta = step_size * abs(mean_intensity - desired_intensity);

  return(mean_intensity > desired_intensity) ?
    std::max(current_exposure - delta , min_exposure) : //OverExposed
    std::min(current_exposure + delta , max_exposure);  //UnderExposed
};

int main(int argc, char * argv[])
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<DavisPublisher>());
  rclcpp::shutdown();
  return 0;
}
