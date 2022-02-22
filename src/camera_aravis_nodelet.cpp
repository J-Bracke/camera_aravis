/****************************************************************************
 *
 * camera_aravis
 *
 * Copyright © 2019 Fraunhofer FKIE, Straw Lab, van Breugel Lab, and contributors
 * Authors: Dominik A. Klein,
 * 			Floris van Breugel,
 * 			Andrew Straw,
 * 			Steve Safarik
 *
 * Licensed under the LGPL, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     https://www.gnu.org/licenses/lgpl-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 ****************************************************************************/

#include "../include/camera_aravis/camera_aravis_nodelet.h"

#include <pluginlib/class_list_macros.h>
PLUGINLIB_EXPORT_CLASS(camera_aravis::CameraAravisNodelet, nodelet::Nodelet)

namespace camera_aravis
{

void renameImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::renameImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  out->encoding = out_format;
}

void shift(uint16_t* data, const size_t length, const size_t digits) {
  for (size_t i=0; i<length; ++i) {
    data[i] <<= digits;
  }
}

void shiftImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::shiftImg(): no input image given.");
    return;
  }

  // make a shallow copy (in-place operation on input)
  out = in;

  // shift
  shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  out->encoding = out_format;
}

void interleaveImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const size_t n_digits, const std::string out_format)
{
  if (!in) {
    ROS_WARN("camera_aravis::interleaveImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::interleaveImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = in->step;
  out->data.resize(in->data.size());

  const size_t n_bytes = in->data.size() / (3 * in->width * in->height);
  uint8_t* c0 = in->data.data();
  uint8_t* c1 = in->data.data() + (in->data.size() / 3);
  uint8_t* c2 = in->data.data() + (2 * in->data.size() / 3);
  uint8_t* o = out->data.data();

  for (uint32_t h=0; h<in->height; ++h) {
    for (uint32_t w=0; w<in->width; ++w) {
      for (size_t i=0; i<n_bytes; ++i) {
        o[i] = c0[i];
        o[i+n_bytes] = c1[i];
        o[i+2*n_bytes] = c2[i];
      }
      c0 += n_bytes;
      c1 += n_bytes;
      c2 += n_bytes;
      o += 3*n_bytes;
    }
  }

  if (n_digits>0) {
    shift(reinterpret_cast<uint16_t*>(out->data.data()), out->data.size()/2, n_digits);
  }
  out->encoding = out_format;
}

void unpack10p32Img(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format LSB
  //  byte 3 | byte 2 | byte 1 | byte 0
  // 00CCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack a full RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    to+=3;
    from+=4;
  }

  out->encoding = out_format;
}

void unpack10PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 3*10+2 = 32 Bit = 4 Byte format
  //  byte 3 | byte 2 | byte 1 | byte 0
  // AAAAAAAA BBBBBBBB CCCCCCCC 00CCBBAA
  // into 3*16 = 48 Bit = 6 Byte format
  //  bytes 5+4       | bytes 3+2       | bytes 1+0
  // CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 0 contains the lsb of C, B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/4; ++i) {

    to[0] = from[0]<<6;
    to[1] = from[3];
    to[2] = (from[0] & 0b00001100)<<4;
    to[3] = from[2];
    to[4] = (from[0] & 0b00110000)<<2;
    to[5] = from[1];

    to+=6;
    from+=4;
  }
  out->encoding = out_format;
}


void unpack10pMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (8*in->step)/5;
  out->data.resize((8*in->data.size())/5);

  // change pixel bit alignment from every 4*10 = 40 Bit = 5 Byte format LSB
  // byte 4  | byte 3 | byte 2 | byte 1 | byte 0
  // DDDDDDDD DDCCCCCC CCCCBBBB BBBBBBAA AAAAAAAA
  // into 4*16 = 64 Bit = 8 Byte format
  // bytes 7+6        | bytes 5+4       | bytes 3+2       | bytes 1+0
  // DDDDDDDD DD000000 CCCCCCCC CC000000 BBBBBBBB BB000000 AAAAAAAA AA000000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/5; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 6;

    std::memcpy(&to[1], &from[1], 2);
    to[1] <<= 4;
    to[1] &= 0b1111111111000000;

    std::memcpy(&to[2], &from[2], 2);
    to[2] <<= 2;
    to[2] &= 0b1111111111000000;

    std::memcpy(&to[3], &from[3], 2);
    to[3] &= 0b1111111111000000;

    to+=4;
    from+=5;
  }
  out->encoding = out_format;
}

void unpack10PackedMonoImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack10pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack10pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*10+4 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB 00BB00AA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BB000000 AAAAAAAA AA000000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 4 mono pixels per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<6;
    to[1] = from[0];

    to[2] = from[1] & 0b11000000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format LSB
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  uint8_t* from = in->data.data();
  uint16_t* to = reinterpret_cast<uint16_t*>(out->data.data());
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    std::memcpy(to, from, 2);
    to[0] <<= 4;

    std::memcpy(&to[1], &from[1], 2);
    to[1] &= 0b1111111111110000;

    to+=2;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack12PackedImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack12pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack12pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (4*in->step)/3;
  out->data.resize((4*in->data.size())/3);

  // change pixel bit alignment from every 2*12 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // BBBBBBBB BBBBAAAA AAAAAAAA
  // into 2*16 = 32 Bit = 4 Byte format
  //  bytes 3+2       | bytes 1+0
  // BBBBBBBB BBBB0000 AAAAAAAA AAAA0000

  // note that in this old style GigE format, byte 1 contains the lsb of B as well as A

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack 2 values per iteration
  for (size_t i=0; i<in->data.size()/3; ++i) {

    to[0] = from[1]<<4;
    to[1] = from[0];

    to[2] = from[1] & 0b11110000;
    to[3] = from[2];

    to+=4;
    from+=3;
  }
  out->encoding = out_format;
}

void unpack565pImg(sensor_msgs::ImagePtr& in, sensor_msgs::ImagePtr& out, const std::string out_format) {
  if (!in) {
    ROS_WARN("camera_aravis::unpack565pImg(): no input image given.");
    return;
  }

  if (!out) {
    out.reset(new sensor_msgs::Image);
    ROS_INFO("camera_aravis::unpack565pImg(): no output image given. Reserved a new one.");
  }

  out->header = in->header;
  out->height = in->height;
  out->width = in->width;
  out->is_bigendian = in->is_bigendian;
  out->step = (3*in->step)/2;
  out->data.resize((3*in->data.size())/2);

  // change pixel bit alignment from every 5+6+5 = 16 Bit = 2 Byte format LSB
  //  byte 1 | byte 0
  // CCCCCBBB BBBAAAAA
  // into 3*8 = 24 Bit = 3 Byte format
  //  byte 2 | byte 1 | byte 0
  // CCCCC000 BBBBBB00 AAAAA000

  uint8_t* from = in->data.data();
  uint8_t* to = out->data.data();
  // unpack a whole RGB pixel per iteration
  for (size_t i=0; i<in->data.size()/2; ++i) {
    to[0] = from[0] << 3;

    to[1] = from[0] >> 3;
    to[1] |= (from[1]<<5);
    to[1] &= 0b11111100;

    to[2] = from[1] & 0b11111000;

    to+=3;
    from+=2;
  }
  out->encoding = out_format;
}

CameraAravisNodelet::CameraAravisNodelet()
{
}

CameraAravisNodelet::~CameraAravisNodelet()
{
  for(int i=0; i < p_streams_.size(); i++) {
    if(p_streams_[i]) {
      arv_stream_set_emit_signals(p_streams_[i], FALSE);
    }
  }

  software_trigger_active_ = false;
  if (software_trigger_thread_.joinable())
  {
    software_trigger_thread_.join();
  }

  tf_thread_active_ = false;
  if (tf_dyn_thread_.joinable())
  {
    tf_dyn_thread_.join();
  }


  for(int i=0; i < p_streams_.size(); i++) {
    guint64 n_completed_buffers = 0;
    guint64 n_failures = 0;
    guint64 n_underruns = 0;
    arv_stream_get_statistics(p_streams_[i], &n_completed_buffers, &n_failures, &n_underruns);
    ROS_INFO("Completed buffers = %Lu", (unsigned long long ) n_completed_buffers);
    ROS_INFO("Failures          = %Lu", (unsigned long long ) n_failures);
    ROS_INFO("Underruns         = %Lu", (unsigned long long ) n_underruns);
    if (arv_camera_is_gv_device(p_camera_))
    {
    guint64 n_resent;
    guint64 n_missing;
    arv_gv_stream_get_statistics(reinterpret_cast<ArvGvStream*>(p_streams_[i]), &n_resent, &n_missing);
    ROS_INFO("Resent buffers    = %Lu", (unsigned long long ) n_resent);
    ROS_INFO("Missing           = %Lu", (unsigned long long ) n_missing);
    }
  }
  

  if (p_device_)
  {
    arv_device_execute_command(p_device_, "AcquisitionStop");
  }

  for(int i = 0; i < p_streams_.size(); i++) {
      g_object_unref(p_streams_[i]);
  }
  g_object_unref(p_camera_);
}

void CameraAravisNodelet::onInit()
{
  ros::NodeHandle pnh = getPrivateNodeHandle();

  // Print out some useful info.
  ROS_INFO("Attached cameras:");
  arv_update_device_list();
  uint n_interfaces = arv_get_n_interfaces();
  ROS_INFO("# Interfaces: %d", n_interfaces);

  uint n_devices = arv_get_n_devices();
  ROS_INFO("# Devices: %d", n_devices);
  for (uint i = 0; i < n_devices; i++)
    ROS_INFO("Device%d: %s", i, arv_get_device_id(i));

  if (n_devices == 0)
  {
    ROS_ERROR("No cameras detected.");
    return;
  }

  // Get the camera guid as a parameter or use the first device.
  std::string guid;
  if (pnh.hasParam("guid"))
  {
    pnh.getParam("guid", guid);
  }

  // Open the camera, and set it up.
  while (!p_camera_)
  {
    if (guid.empty())
    {
      ROS_INFO("Opening: (any)");
      p_camera_ = arv_camera_new(NULL);
    }
    else
    {
      ROS_INFO_STREAM("Opening: " << guid);
      p_camera_ = arv_camera_new(guid.c_str());
    }
    ros::Duration(1.0).sleep();
    ros::spinOnce();
  }

  p_device_ = arv_camera_get_device(p_camera_);
  ROS_INFO("Opened: %s-%s", arv_camera_get_vendor_name(p_camera_),
           arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber"));

  // See which features exist in this camera device
  discoverFeatures();

  // Check the number of streams for this camera
  num_streams_ = 0;
  num_streams_ = arv_device_get_integer_feature_value(p_device_, "DeviceStreamChannelCount");
  // if this return 0, try the deprecated GevStreamChannelCount in case this is an older camera
  if (num_streams_ == 0) {
    num_streams_ = arv_device_get_integer_feature_value(p_device_, "GevStreamChannelCount");
  }
  ROS_INFO("Number of supported stream channels %i.", (int) num_streams_);

  std::string stream_channel_args;
  if (pnh.getParam("channel_names", stream_channel_args)) {
    parseStringArgs(stream_channel_args, stream_names_);
  } else {
    stream_names_ = { "" };
  }

  // Start the dynamic_reconfigure servers.
  for(int i = 0; i < 1; i++) {
    ros::NodeHandle pnh_(pnh, stream_names_[0]);
    reconfigure_server_vis_.reset(new dynamic_reconfigure::Server<ConfigVis>(reconfigure_mutex_vis_, pnh_));
    reconfigure_server_vis_->getConfigDefault(config_vis_);
    reconfigure_server_vis_->getConfigMin(config_vis_min_);
    reconfigure_server_vis_->getConfigMax(config_vis_max_);
  }

  for(int i = 0; i < 1; i++) {
    ros::NodeHandle pnh_(pnh, stream_names_[1]);
    reconfigure_server_nir_.reset(new dynamic_reconfigure::Server<ConfigNir>(reconfigure_mutex_nir_, pnh_));
    reconfigure_server_nir_->getConfigDefault(config_nir_);
    reconfigure_server_nir_->getConfigMin(config_nir_min_);
    reconfigure_server_nir_->getConfigMax(config_nir_max_);
  }

  for(int i = 0; i < 1; i++) {
    ros::NodeHandle pnh_(pnh, stream_names_[2]);
    reconfigure_server_nir2_.reset(new dynamic_reconfigure::Server<ConfigNir>(reconfigure_mutex_nir2_, pnh_));
    reconfigure_server_nir2_->getConfigDefault(config_nir2_);
    reconfigure_server_nir2_->getConfigMin(config_nir2_min_);
    reconfigure_server_nir2_->getConfigMax(config_nir2_max_);
  }

  std::string pixel_format_args;
  std::vector<std::string> pixel_formats;
  pnh.param("PixelFormat", pixel_format_args, pixel_format_args);
  parseStringArgs(pixel_format_args, pixel_formats);

  std::string calib_url_args;
  std::vector<std::string> calib_urls;
  pnh.param("camera_info_url", calib_url_args, calib_url_args);
  parseStringArgs(calib_url_args, calib_urls);

  // check if every stream channel has been given a channel name
  if (stream_names_.size() < num_streams_) {
    num_streams_ = stream_names_.size();
  }

  // initialize the sensor structs
  for(int i = 0; i < num_streams_; i++) {
    sensors_.push_back(new Sensor());
    p_streams_.push_back(NULL);
    p_buffer_pools_.push_back(CameraBufferPool::Ptr());
    p_camera_info_managers_.push_back(NULL);
    camera_infos_.push_back(sensor_msgs::CameraInfoPtr());
    cam_pubs_.push_back(image_transport::CameraPublisher());
  }

  // Get parameter bounds
  arv_camera_get_frame_rate_bounds(p_camera_, &config_vis_min_.AcquisitionFrameRate, &config_vis_max_.AcquisitionFrameRate);
  arv_camera_get_frame_rate_bounds(p_camera_, &config_nir_min_.AcquisitionFrameRate, &config_nir_max_.AcquisitionFrameRate);
  for(int i = 0; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_, i);
    arv_camera_get_sensor_size(p_camera_, &sensors_[i]->width, &sensors_[i]->height);
  }
  arv_camera_get_width_bounds(p_camera_, &roi_.width_min, &roi_.width_max);
  arv_camera_get_height_bounds(p_camera_, &roi_.height_min, &roi_.height_max);
  
  // set default config parameter setting as initial camera setting
  // for first (visible) channel

  arv_camera_gv_select_stream_channel(p_camera_, 0);
  std::string source_control_ = "Source0";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());

  arv_device_set_float_feature_value(p_device_, "AcquisitionFrameRate", config_vis_.AcquisitionFrameRate);
  arv_device_set_float_feature_value(p_device_, "ExposureTime", config_vis_.ExposureTime);
  arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config_vis_.ExposureAutoControlMin);
  arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config_vis_.ExposureAutoControlMax);
  arv_device_set_string_feature_value(p_device_, "ExposureAuto", config_vis_.ExposureAuto.c_str());
  
  arv_device_set_string_feature_value(p_device_, "GainAuto", config_vis_.GainAuto.c_str());
  arv_device_set_float_feature_value(p_device_, "Gain", config_vis_.GainAnalogAll);
  arv_device_set_float_feature_value(p_device_, "GainAutoControlMin", config_vis_.GainAutoControlMin);
  arv_device_set_float_feature_value(p_device_, "GainAutoControlMax", config_vis_.GainAutoControlMax);

  arv_device_set_string_feature_value(p_device_, "BalanceWhiteAuto", config_vis_.BalanceWhiteAuto.c_str());
  std::string gain_selector = "DigitalRed";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  arv_device_set_float_feature_value(p_device_, "Gain", config_vis_.GainDigitalRed);
  gain_selector = "DigitalBlue";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  arv_device_set_float_feature_value(p_device_, "Gain", config_vis_.GainDigitalBlue);
  arv_device_set_integer_feature_value(p_device_, "AWBControlSpeed", config_vis_.AWBControlSpeed);
  arv_device_set_integer_feature_value(p_device_, "ALCReference", config_vis_.ALCReference);
  arv_device_set_integer_feature_value(p_device_, "ALCControlSpeed", config_vis_.ALCControlSpeed);
  
  arv_device_set_float_feature_value(p_device_, "Gamma", config_vis_.Gamma);

  // init default to full sensor resolution
  arv_camera_set_region(p_camera_, 0, 0, roi_.width_max, roi_.height_max);

  // activate Acquisition Sync Mode
  std::string sync_mode = "SyncMode";
  arv_device_set_string_feature_value(p_device_, "AcquisitionSyncMode", sync_mode.c_str());

  // possibly set or override from given parameter
  gain_selector = "AnalogAll";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  writeCameraFeaturesFromRosparam();

  // for second & third (near-infrared) channels
  for(int i = 1; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_, i);
    if (i == 1)
    {
      source_control_ = "Source1";
    }
    else if (i == 2)
    {
      source_control_ = "Source2";
    }
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());
    
    arv_device_set_float_feature_value(p_device_, "AcquisitionFrameRate", config_nir_.AcquisitionFrameRate);
    arv_device_set_float_feature_value(p_device_, "ExposureTime", config_nir_.ExposureTime);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config_nir_.ExposureAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config_nir_.ExposureAutoControlMax);
    arv_device_set_string_feature_value(p_device_, "ExposureAuto", config_nir_.ExposureAuto.c_str());

    arv_device_set_string_feature_value(p_device_, "GainAuto", config_nir_.GainAuto.c_str());
    arv_device_set_float_feature_value(p_device_, "Gain", config_nir_.GainAnalogAll);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMin", config_nir_.GainAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMax", config_nir_.GainAutoControlMax);

    arv_device_set_integer_feature_value(p_device_, "ALCReference", config_nir_.ALCReference);
    arv_device_set_integer_feature_value(p_device_, "ALCControlSpeed", config_nir_.ALCControlSpeed);

    arv_device_set_float_feature_value(p_device_, "Gamma", config_nir_.Gamma);

    // init default to full sensor resolution
    arv_camera_set_region(p_camera_, 0, 0, roi_.width_max, roi_.height_max);

    // activate Acquisition Sync Mode
    arv_device_set_string_feature_value(p_device_, "AcquisitionSyncMode", sync_mode.c_str());

    // possibly set or override from given parameter
    writeCameraFeaturesFromRosparam();
  }

  // get current camera settings, in case they have be changed via the rosparam option
  // for first (visible) channel
  arv_camera_gv_select_stream_channel(p_camera_, 0);
  source_control_ = "Source0";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());

  config_vis_.ExposureAuto = arv_device_get_string_feature_value(p_device_, "ExposureAuto");
  config_vis_.ExposureTime = arv_device_get_float_feature_value(p_device_, "ExposureTime");

  gain_selector = "AnalogAll";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  config_vis_.GainAuto = arv_device_get_string_feature_value(p_device_, "GainAuto");
  config_vis_.GainAnalogAll = arv_device_get_float_feature_value(p_device_, "Gain");
  config_vis_.GainAutoControlMin = arv_device_get_float_feature_value(p_device_, "GainAutoControlMin");
  config_vis_.GainAutoControlMax = arv_device_get_float_feature_value(p_device_, "GainAutoControlMax");

  config_vis_.BalanceWhiteAuto = arv_device_get_string_feature_value(p_device_, "BalanceWhiteAuto");
  gain_selector = "DigitalRed";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  config_vis_.GainDigitalRed = arv_device_get_float_feature_value(p_device_, "Gain");
  gain_selector = "DigitalBlue";
  arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
  config_vis_.GainDigitalBlue = arv_device_get_float_feature_value(p_device_, "Gain");
  config_vis_.AWBControlSpeed = arv_device_get_integer_feature_value(p_device_, "AWBControlSpeed");
  config_vis_.ALCReference = arv_device_get_integer_feature_value(p_device_, "ALCReference");
  config_vis_.ALCControlSpeed = arv_device_get_integer_feature_value(p_device_, "ALCControlSpeed");

  config_vis_.AcquisitionFrameRate = arv_device_get_float_feature_value(p_device_, "AcquisitionFrameRate");
  config_vis_.Gamma = arv_device_get_float_feature_value(p_device_, "Gamma");

  arv_camera_get_region(p_camera_, &roi_.x, &roi_.y, &roi_.width, &roi_.height);

  // for second & third (near-infrared) channels
  for(int i = 1; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_, i);
    if (i == 1)
    {
      source_control_ = "Source1";
    }
    else if (i == 2)
    {
      source_control_ = "Source2";
    }
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());

    config_nir_.ExposureAuto = arv_device_get_string_feature_value(p_device_, "ExposureAuto");
    config_nir_.ExposureTime = arv_device_get_float_feature_value(p_device_, "ExposureTime");

    config_nir_.GainAuto = arv_device_get_string_feature_value(p_device_, "GainAuto");
    config_nir_.GainAnalogAll = arv_device_get_float_feature_value(p_device_, "Gain");
    config_nir_.GainAutoControlMin = arv_device_get_float_feature_value(p_device_, "GainAutoControlMin");
    config_nir_.GainAutoControlMax = arv_device_get_float_feature_value(p_device_, "GainAutoControlMax");

    config_nir_.ALCReference = arv_device_get_integer_feature_value(p_device_, "ALCReference");
    config_nir_.ALCControlSpeed = arv_device_get_integer_feature_value(p_device_, "ALCControlSpeed");

    config_nir_.AcquisitionFrameRate = arv_device_get_float_feature_value(p_device_, "AcquisitionFrameRate");
    config_nir_.Gamma = arv_device_get_float_feature_value(p_device_, "Gamma");
  }

  // get pixel format name and translate into corresponding ROS name
  for(int i = 0; i < num_streams_; i++) {
    arv_camera_gv_select_stream_channel(p_camera_, i);
    std::string source_control_ = "Source2";
    if (i == 0)
    {
      source_control_ = "Source0";
    }
    else if (i == 1)
    {
      source_control_ = "Source1";
    }
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());
    arv_device_set_string_feature_value(p_device_, "PixelFormat", pixel_formats[i].c_str());
    sensors_[i]->pixel_format = std::string(arv_device_get_string_feature_value(p_device_, "PixelFormat"));
    const auto sensor_iter = CONVERSIONS_DICTIONARY.find(sensors_[i]->pixel_format);
    if (sensor_iter!=CONVERSIONS_DICTIONARY.end()) {
      convert_format.push_back(sensor_iter->second);
    }
    else {
      ROS_WARN_STREAM("There is no known conversion from " << sensors_[i]->pixel_format << " to a usual ROS image encoding. Likely you need to implement one.");
    }

    sensors_[i]->n_bits_pixel = ARV_PIXEL_FORMAT_BIT_PER_PIXEL(
        arv_device_get_integer_feature_value(p_device_, "PixelFormat"));

  }

  // should we publish tf transforms to camera optical frame?
  bool pub_tf_optical;
  pnh.param<bool>("publish_tf", pub_tf_optical, false);
  if (pub_tf_optical)
  {
    tf_optical_.transform.translation.x = 0.0;
    tf_optical_.transform.translation.y = 0.0;
    tf_optical_.transform.translation.z = 0.0;
    tf_optical_.transform.rotation.x = -0.5;
    tf_optical_.transform.rotation.y = 0.5;
    tf_optical_.transform.rotation.z = -0.5;
    tf_optical_.transform.rotation.w = 0.5;

    double tf_publish_rate;
    pnh.param<double>("tf_publish_rate", tf_publish_rate, 0);
    if (tf_publish_rate > 0.)
    {
      // publish dynamic tf at given rate (recommended when running as a Nodelet, since latching has bugs-by-design)
      p_tb_.reset(new tf2_ros::TransformBroadcaster());
      tf_dyn_thread_ = std::thread(&CameraAravisNodelet::publishTfLoop, this, tf_publish_rate);
    }
    else
    {
      // publish static tf only once (latched)
      p_stb_.reset(new tf2_ros::StaticTransformBroadcaster());
      tf_optical_.header.stamp = ros::Time::now();
      p_stb_->sendTransform(tf_optical_);
    }
  }

  // default calibration url is [DeviceSerialNumber/DeviceID].yaml
  ArvGcNode *p_gc_node;
  GError *error = NULL;

  p_gc_node = arv_device_get_feature(p_device_, "DeviceSerialNumber");

  if(calib_urls[0].empty()) {
    if( arv_gc_feature_node_is_implemented( ARV_GC_FEATURE_NODE(p_gc_node), &error) ) {
      GType device_serial_return_type = arv_gc_feature_node_get_value_type( ARV_GC_FEATURE_NODE(p_gc_node));
      // If the feature DeviceSerialNumber is not string, it indicates that the camera is using an older version of the genicam SFNC.
      // Older camera models do not have a DeviceSerialNumber as string, but as integer and often set to 0.
      // In those cases use the outdated DeviceID (deprecated since genicam SFNC v2.0).
      if (device_serial_return_type == G_TYPE_STRING) {
        calib_urls[0] = arv_device_get_string_feature_value(p_device_, "DeviceSerialNumber");
        calib_urls[0] += ".yaml";
      } else if (device_serial_return_type == G_TYPE_INT64) {
        calib_urls[0] = arv_device_get_string_feature_value(p_device_, "DeviceID");
        calib_urls[0] += ".yaml";
      }
    }
  }


  for(int i = 0; i < num_streams_; i++) {
    ros::NodeHandle pnh_(pnh, stream_names_[i]);
    // Start the camerainfo manager.
    p_camera_info_managers_[i].reset(new camera_info_manager::CameraInfoManager(pnh_, "jai_cam/" + stream_names_[i], calib_urls[i]));
    // publish an extended camera info message
    ROS_INFO("Reset %s Camera Info Manager", stream_names_[i].c_str());
    ROS_INFO("%s Calib URL: %s", stream_names_[i].c_str(), calib_urls[i].c_str());
  }

  // synchronize the nir-channel configs
  config_nir2_ = config_nir_;
  config_nir2_min_ = config_nir_min_;
  config_nir2_max_ = config_nir_max_;

  // update the reconfigure configs
  reconfigure_server_vis_->setConfigMin(config_vis_min_);
  reconfigure_server_vis_->setConfigMax(config_vis_max_);
  reconfigure_server_vis_->updateConfig(config_vis_);
  ros::Duration(2.0).sleep();
  reconfigure_server_vis_->setCallback(boost::bind(&CameraAravisNodelet::rosReconfigureCallbackVis, this, _1, _2));

  reconfigure_server_nir_->setConfigMin(config_nir_min_);
  reconfigure_server_nir_->setConfigMax(config_nir_max_);
  reconfigure_server_nir_->updateConfig(config_nir_);
  ros::Duration(2.0).sleep();
  reconfigure_server_nir_->setCallback(boost::bind(&CameraAravisNodelet::rosReconfigureCallbackNir, this, _1, _2));

  reconfigure_server_nir2_->setConfigMin(config_nir2_min_);
  reconfigure_server_nir2_->setConfigMax(config_nir2_max_);
  reconfigure_server_nir2_->updateConfig(config_nir2_);
  ros::Duration(2.0).sleep();
  reconfigure_server_nir2_->setCallback(boost::bind(&CameraAravisNodelet::rosReconfigureCallbackNir2, this, _1, _2));

  if (sync_mode == "SyncMode") {
    adaptExposureSettingsToFPS();
  }

  // Print information.
  ROS_INFO("    Using Camera Configuration:");
  ROS_INFO("    ---------------------------");
  ROS_INFO("    Vendor name          = %s", arv_device_get_string_feature_value(p_device_, "DeviceVendorName"));
  ROS_INFO("    Model name           = %s", arv_device_get_string_feature_value(p_device_, "DeviceModelName"));
  ROS_INFO("    Sensor width         = %d", sensors_[0]->width);
  ROS_INFO("    Sensor height        = %d", sensors_[0]->height);
  ROS_INFO("    Pixel format         = %s", sensors_[0]->pixel_format.c_str());
  ROS_INFO("    BitsPerPixel         = %lu", sensors_[0]->n_bits_pixel);
  ROS_INFO("    AcquisitionFrameRate = %g hz", config_vis_.AcquisitionFrameRate);
  ROS_INFO("    Max. AcquisitionFrameRate = %g hz", config_vis_max_.AcquisitionFrameRate);
  

  for(int i = 0; i < num_streams_; i++) {
    std::string source_control_ = "Source2";
    if (i == 0)
    {
      source_control_ = "Source0";
    }
    else if (i == 1)
    {
      source_control_ = "Source1";
    }
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());
    while (true) {

      arv_camera_gv_select_stream_channel(p_camera_, i);
      p_streams_[i] = arv_camera_create_stream(p_camera_, NULL, NULL);

        if (p_streams_[i])
        {
          // Load up some buffers.
          arv_camera_gv_select_stream_channel(p_camera_, i);
          const gint n_bytes_payload_stream_ = arv_camera_get_payload(p_camera_);

          p_buffer_pools_[i].reset(new CameraBufferPool(p_streams_[i], n_bytes_payload_stream_, 10));

          if (arv_camera_is_gv_device(p_camera_))
          {
            tuneGvStream(reinterpret_cast<ArvGvStream*>(p_streams_[i]));
          }
          break;
        }
        else
        {
          ROS_WARN("Stream %i: Could not create image stream for %s.  Retrying...", i, guid.c_str());
          ros::Duration(1.0).sleep();
          ros::spinOnce();
        }
    }
  }

  // Monitor whether anyone is subscribed to the camera stream
  std::vector<image_transport::SubscriberStatusCallback> image_cbs_;
  std::vector<ros::SubscriberStatusCallback> info_cbs_;

  image_transport::SubscriberStatusCallback image_cb = [this](const image_transport::SingleSubscriberPublisher &ssp)
  { this->rosConnectCallback();};
  ros::SubscriberStatusCallback info_cb = [this](const ros::SingleSubscriberPublisher &ssp)
  { this->rosConnectCallback();};

  for(int i = 0; i < num_streams_; i++) {
    image_transport::ImageTransport *p_transport;
    // Set up image_raw
    std::string frame_id = "";
    p_transport = new image_transport::ImageTransport(pnh);
    if(num_streams_ == 1 && stream_names_[i].empty()) {
      frame_id = "";
    } else {
      frame_id = stream_names_[i] + "/";
    }

    ROS_INFO("Mapping to %s", (frame_id + "image_raw").c_str());
    cam_pubs_[i] = p_transport->advertiseCamera(
      ros::names::remap(frame_id + "image_raw"),
      1, image_cb, image_cb, info_cb, info_cb);
  }

  // Connect signals with callbacks.
  for(int i = 0; i < num_streams_; i++) {
    StreamIdData* data = new StreamIdData();
    data->can = this;
    data->stream_id = i;
    g_signal_connect(p_streams_[i], "new-buffer", (GCallback)CameraAravisNodelet::newBufferReadyCallback, data);
  }
  g_signal_connect(p_device_, "control-lost", (GCallback)CameraAravisNodelet::controlLostCallback, this);

  for(int i = 0; i < num_streams_; i++) {
    std::string source_control_ = "Source2";
    if (i == 0)
    {
      source_control_ = "Source0";
    }
    else if (i == 1)
    {
      source_control_ = "Source1";
    }
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_.c_str());
    arv_stream_set_emit_signals(p_streams_[i], TRUE);
  }

  if (std::any_of(cam_pubs_.begin(), cam_pubs_.end(),
    [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() > 0; })
  ) {
    arv_camera_start_acquisition(p_camera_);
  }

  ROS_INFO("Done initializing camera_aravis.");
}
/*
void CameraAravisNodelet::cameraAutoInfoCallback(const CameraAutoInfoConstPtr &msg_ptr)
{
  if (config_.AutoSlave && p_device_)
  {

    if (auto_params_.exposure_time != msg_ptr->exposure_time && implemented_features_["ExposureTime"])
    {
      arv_device_set_float_feature_value(p_device_, "ExposureTime", msg_ptr->exposure_time);
    }

    if (implemented_features_["Gain"])
    {
      if (auto_params_.gain != msg_ptr->gain)
      {
        if (implemented_features_["GainSelector"])
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "All");
        }
        arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain);
      }

      if (implemented_features_["GainSelector"])
      {
        if (auto_params_.gain_red != msg_ptr->gain_red)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Red");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_red);
        }

        if (auto_params_.gain_green != msg_ptr->gain_green)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Green");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_green);
        }

        if (auto_params_.gain_blue != msg_ptr->gain_blue)
        {
          arv_device_set_string_feature_value(p_device_, "GainSelector", "Blue");
          arv_device_set_float_feature_value(p_device_, "Gain", msg_ptr->gain_blue);
        }
      }
    }

    if (implemented_features_["BlackLevel"])
    {
      if (auto_params_.black_level != msg_ptr->black_level)
      {
        if (implemented_features_["BlackLevelSelector"])
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "All");
        }
        arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->black_level);
      }

      if (implemented_features_["BlackLevelSelector"])
      {
        if (auto_params_.bl_red != msg_ptr->bl_red)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Red");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_red);
        }

        if (auto_params_.bl_green != msg_ptr->bl_green)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Green");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_green);
        }

        if (auto_params_.bl_blue != msg_ptr->bl_blue)
        {
          arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Blue");
          arv_device_set_float_feature_value(p_device_, "BlackLevel", msg_ptr->bl_blue);
        }
      }
    }

    // White balance as TIS is providing
    if (strcmp("The Imaging Source Europe GmbH", arv_camera_get_vendor_name(p_camera_)) == 0)
    {
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceRedRegister", (int)(auto_params_.wb_red * 255.));
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceGreenRegister", (int)(auto_params_.wb_green * 255.));
      arv_device_set_integer_feature_value(p_device_, "WhiteBalanceBlueRegister", (int)(auto_params_.wb_blue * 255.));
    }
    else if (implemented_features_["BalanceRatio"] && implemented_features_["BalanceRatioSelector"])
    {
      if (auto_params_.wb_red != msg_ptr->wb_red)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Red");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_red);
      }

      if (auto_params_.wb_green != msg_ptr->wb_green)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Green");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_green);
      }

      if (auto_params_.wb_blue != msg_ptr->wb_blue)
      {
        arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Blue");
        arv_device_set_float_feature_value(p_device_, "BalanceRatio", msg_ptr->wb_blue);
      }
    }

    auto_params_ = *msg_ptr;
  }
}

void CameraAravisNodelet::syncAutoParameters()
{
  auto_params_.exposure_time = auto_params_.gain = auto_params_.gain_red = auto_params_.gain_green =
      auto_params_.gain_blue = auto_params_.black_level = auto_params_.bl_red = auto_params_.bl_green =
          auto_params_.bl_blue = auto_params_.wb_red = auto_params_.wb_green = auto_params_.wb_blue =
              std::numeric_limits<double>::quiet_NaN();

  if (p_device_)
  {
    if (implemented_features_["ExposureTime"])
    {
      auto_params_.exposure_time = arv_device_get_float_feature_value(p_device_, "ExposureTime");
    }

    if (implemented_features_["Gain"])
    {
      if (implemented_features_["GainSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "GainSelector", "All");
      }
      auto_params_.gain = arv_device_get_float_feature_value(p_device_, "Gain");
      if (implemented_features_["GainSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Red");
        auto_params_.gain_red = arv_device_get_float_feature_value(p_device_, "Gain");
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Green");
        auto_params_.gain_green = arv_device_get_float_feature_value(p_device_, "Gain");
        arv_device_set_string_feature_value(p_device_, "GainSelector", "Blue");
        auto_params_.gain_blue = arv_device_get_float_feature_value(p_device_, "Gain");
      }
    }

    if (implemented_features_["BlackLevel"])
    {
      if (implemented_features_["BlackLevelSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "All");
      }
      auto_params_.black_level = arv_device_get_float_feature_value(p_device_, "BlackLevel");
      if (implemented_features_["BlackLevelSelector"])
      {
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Red");
        auto_params_.bl_red = arv_device_get_float_feature_value(p_device_, "BlackLevel");
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Green");
        auto_params_.bl_green = arv_device_get_float_feature_value(p_device_, "BlackLevel");
        arv_device_set_string_feature_value(p_device_, "BlackLevelSelector", "Blue");
        auto_params_.bl_blue = arv_device_get_float_feature_value(p_device_, "BlackLevel");
      }
    }

    // White balance as TIS is providing
    if (strcmp("The Imaging Source Europe GmbH", arv_camera_get_vendor_name(p_camera_)) == 0)
    {
      auto_params_.wb_red = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceRedRegister") / 255.;
      auto_params_.wb_green = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceGreenRegister") / 255.;
      auto_params_.wb_blue = arv_device_get_integer_feature_value(p_device_, "WhiteBalanceBlueRegister") / 255.;
    }
    // the standard way
    else if (implemented_features_["BalanceRatio"] && implemented_features_["BalanceRatioSelector"])
    {
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Red");
      auto_params_.wb_red = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Green");
      auto_params_.wb_green = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
      arv_device_set_string_feature_value(p_device_, "BalanceRatioSelector", "Blue");
      auto_params_.wb_blue = arv_device_get_float_feature_value(p_device_, "BalanceRatio");
    }
  }
}

void CameraAravisNodelet::setAutoMaster(bool value)
{
  if (value)
  {
    syncAutoParameters();
    auto_pub_ = getNodeHandle().advertise<CameraAutoInfo>(ros::names::remap("camera_auto_info"), 1, true);
  }
  else
  {
    auto_pub_.shutdown();
  }
}

void CameraAravisNodelet::setAutoSlave(bool value)
{
  if (value)
  {
    // deactivate all auto functions
    if (implemented_features_["ExposureAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "ExposureAuto", "Off");
    }
    if (implemented_features_["GainAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "GainAuto", "Off");
    }
    if (implemented_features_["GainAutoBalance"])
    {
      arv_device_set_string_feature_value(p_device_, "GainAutoBalance", "Off");
    }
    if (implemented_features_["BlackLevelAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "BlackLevelAuto", "Off");
    }
    if (implemented_features_["BlackLevelAutoBalance"])
    {
      arv_device_set_string_feature_value(p_device_, "BlackLevelAutoBalance", "Off");
    }
    if (implemented_features_["BalanceWhiteAuto"])
    {
      arv_device_set_string_feature_value(p_device_, "BalanceWhiteAuto", "Off");
    }
    syncAutoParameters();
    auto_sub_ = getNodeHandle().subscribe(ros::names::remap("camera_auto_info"), 1,
                                          &CameraAravisNodelet::cameraAutoInfoCallback, this);
  }
  else
  {
    auto_sub_.shutdown();
  }
}
*/

// Extra stream options for GigEVision streams.
void CameraAravisNodelet::tuneGvStream(ArvGvStream *p_stream)
{
  gboolean b_auto_buffer = TRUE;
  gboolean b_packet_resend = FALSE;
  unsigned int timeout_packet = 400; // milliseconds
  unsigned int timeout_frame_retention = 2000;

  if (p_stream)
  {
    if (!ARV_IS_GV_STREAM(p_stream))
    {
      ROS_WARN("Stream is not a GV_STREAM");
      return;
    }

    if (b_auto_buffer)
      g_object_set(p_stream, "socket-buffer", ARV_GV_STREAM_SOCKET_BUFFER_AUTO, "socket-buffer-size", 0,
      NULL);
    if (!b_packet_resend)
      g_object_set(p_stream, "packet-resend",
                   b_packet_resend ? ARV_GV_STREAM_PACKET_RESEND_ALWAYS : ARV_GV_STREAM_PACKET_RESEND_NEVER,
                   NULL);
    g_object_set(p_stream, "packet-timeout", timeout_packet * 1000, "frame-retention", timeout_frame_retention * 1000,
    NULL);
  }
}

void CameraAravisNodelet::rosReconfigureCallbackVis(ConfigVis &config, uint32_t level)
{
  reconfigure_mutex_vis_.lock();
  std::string tf_prefix = tf::getPrefixParam(getNodeHandle());
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  arv_camera_gv_select_stream_channel(p_camera_, 0);
  std::string source_control_vis = "Source0";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_vis.c_str());

  std::string gain_selector = "DigitalRed";

  // Limit params to legal values.
  config.AcquisitionFrameRate = CLAMP(config.AcquisitionFrameRate, config_vis_min_.AcquisitionFrameRate,
                                      config_vis_max_.AcquisitionFrameRate);
  config.ExposureTime = CLAMP(config.ExposureTime, config_vis_min_.ExposureTime, config_vis_max_.ExposureTime);
  config.ExposureAutoControlMin = CLAMP(config.ExposureAutoControlMin, config_vis_min_.ExposureAutoControlMin, config_vis_max_.ExposureAutoControlMin);
  config.ExposureAutoControlMax = CLAMP(config.ExposureAutoControlMax, config_vis_min_.ExposureAutoControlMax, config_vis_max_.ExposureAutoControlMax);
  config.GainAnalogAll = CLAMP(config.GainAnalogAll, config_vis_min_.GainAnalogAll, config_vis_max_.GainAnalogAll);
  config.GainAutoControlMin = CLAMP(config.GainAutoControlMin, config_vis_min_.GainAutoControlMin, config_vis_max_.GainAutoControlMin);
  config.GainAutoControlMax = CLAMP(config.GainAutoControlMax, config_vis_min_.GainAutoControlMax, config_vis_max_.GainAutoControlMax);
  config.GainDigitalRed = CLAMP(config.GainDigitalRed, config_vis_min_.GainDigitalRed, config_vis_max_.GainDigitalRed);
  config.GainDigitalBlue = CLAMP(config.GainDigitalBlue, config_vis_min_.GainDigitalBlue, config_vis_max_.GainDigitalBlue);
  config.AWBControlSpeed = CLAMP(config.AWBControlSpeed, config_vis_min_.AWBControlSpeed, config_vis_max_.AWBControlSpeed);
  config.ALCReference = CLAMP(config.ALCReference, config_vis_min_.ALCReference, config_vis_max_.ALCReference);
  config.ALCControlSpeed = CLAMP(config.ALCControlSpeed, config_vis_min_.ALCControlSpeed, config_vis_max_.ALCControlSpeed);
  config.Gamma = CLAMP(config.Gamma, config_vis_min_.Gamma, config_vis_max_.Gamma);

  // reset values controlled by auto functions
  if (config.ExposureAuto.compare("Off") != 0)
  {
    config.ExposureTime = config_vis_.ExposureTime;
    ROS_WARN("(vis) ExposureAuto is active. Cannot manually set ExposureTime.");
  }
  if (config.GainAuto.compare("Off") != 0)
  {
    config.GainAnalogAll = config_vis_.GainAnalogAll;
    ROS_WARN("(vis) GainAuto is active. Cannot manually set Gain.");
  }
  if (config.BalanceWhiteAuto.compare("Off") != 0)
  {
    config.GainDigitalBlue = config_vis_.GainDigitalBlue;
    config.GainDigitalRed = config_vis_.GainDigitalRed;
    ROS_WARN("(vis) BalanceWhiteAuto is active. Cannot manually set DigitalRed- oder DigitalBlue-Gain.");
  }

  // Find valid user changes we need to react to.
  const bool changed_acquisition_frame_rate = (config_vis_.AcquisitionFrameRate != config.AcquisitionFrameRate);
  const bool changed_exposure_auto = (config_vis_.ExposureAuto != config.ExposureAuto);
  const bool changed_exposure_time = (config_vis_.ExposureTime != config.ExposureTime);
  const bool changed_exposure_autocontrolmin = (config_vis_.ExposureAutoControlMin != config.ExposureAutoControlMin);
  const bool changed_exposure_autocontrolmax = (config_vis_.ExposureAutoControlMax != config.ExposureAutoControlMax);
  const bool changed_gain_auto = (config_vis_.GainAuto != config.GainAuto);
  const bool changed_gain = (config_vis_.GainAnalogAll != config.GainAnalogAll);
  const bool changed_gain_autocontrolmin = (config_vis_.GainAutoControlMin != config.GainAutoControlMin);
  const bool changed_gain_autocontrolmax = (config_vis_.GainAutoControlMax != config.GainAutoControlMax);
  const bool changed_balancewhiteauto = (config_vis_.BalanceWhiteAuto != config.BalanceWhiteAuto);
  const bool changed_gain_digitalred = (config_vis_.GainDigitalRed != config.GainDigitalRed);
  const bool changed_gain_digitalblue = (config_vis_.GainDigitalBlue != config.GainDigitalBlue);
  const bool changed_awbcontrolspeed = (config_vis_.AWBControlSpeed != config.AWBControlSpeed);
  const bool changed_alcreference = (config_vis_.ALCReference != config.ALCReference);
  const bool changed_alccontrolspeed = (config_vis_.ALCControlSpeed != config.ALCControlSpeed);
  const bool changed_gamma = (config_vis_.Gamma != config.Gamma);

  // Set params into the camera.
  if (changed_acquisition_frame_rate)
  {
    ROS_INFO("(vis, nir, nir2) Set AquisitionFrameRate = %f hz", config.AcquisitionFrameRate);
    arv_device_set_float_feature_value(p_device_, "AcquisitionFrameRate", config.AcquisitionFrameRate);
    
    //update AquisitionFrameRate value for the other channels
    config_nir_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_nir_->updateConfig(config_nir_);
    config_nir2_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_nir2_->updateConfig(config_nir2_);
    config_vis_ = config;

    if (sync_mode == "SyncMode") {
      adaptExposureSettingsToFPS();
    }
    config = config_vis_;

    //reset source selector
    arv_camera_gv_select_stream_channel(p_camera_, 0);
    source_control_vis = "Source0";
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_vis.c_str());
  }
  
  if (changed_exposure_auto)
  {
    ROS_INFO("(vis) Set ExposureAuto = %s", config.ExposureAuto.c_str());
    config.ExposureTime = arv_camera_get_exposure_time(p_camera_);
    arv_device_set_string_feature_value(p_device_, "ExposureAuto", config.ExposureAuto.c_str());
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(vis) Get ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
        config_vis_ = config;
        reconfigure_server_vis_->updateConfig(config_vis_);
      }
  }

  if (changed_exposure_time)
  {
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(vis) Set ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
      }
    else
      {
        ROS_WARN("(vis) ExposureAuto is active. Cannot manually set ExposureTime.");
      }
  }
  
  if (changed_exposure_autocontrolmin)
  {
    ROS_INFO("(vis) Set ExposureAutoControlMin = %f us", config.ExposureAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config.ExposureAutoControlMin);
  }
    
  if (changed_exposure_autocontrolmax)
  {
    ROS_INFO("(vis) Set ExposureAutoControlMax = %f us", config.ExposureAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config.ExposureAutoControlMax);
  }

  if (changed_gain_auto)
  {
    ROS_INFO("(vis) Set GainAuto = %s", config.GainAuto.c_str());
    gain_selector = "AnalogAll";
    arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
    config.GainAnalogAll = arv_camera_get_gain(p_camera_);
    arv_device_set_string_feature_value(p_device_, "GainAuto", config.GainAuto.c_str());
    if (config.GainAuto.compare("Off") == 0)
      {
        ROS_INFO("(vis) Get Gain = %f", config.GainAnalogAll);
        arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
        config_vis_ = config;
        reconfigure_server_vis_->updateConfig(config_vis_);
      }
  }

  if (changed_gain)
  {
    ROS_INFO("(vis) Set Gain = %f", config.GainAnalogAll);
    gain_selector = "AnalogAll";
    arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
    arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
  }

  if (changed_gain_digitalred)
  {
    ROS_INFO("(vis) Set GainDigitalRed = %f", config.GainDigitalRed);
    gain_selector = "DigitalRed";
    arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
    arv_device_set_float_feature_value(p_device_, "Gain", config.GainDigitalRed);
  }

  if (changed_gain_digitalblue)
  {
    ROS_INFO("(vis) Set GainDigitalBlue = %f", config.GainDigitalBlue);
    gain_selector = "DigitalBlue";
    arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
    arv_device_set_float_feature_value(p_device_, "Gain", config.GainDigitalBlue);
  }

  if (changed_gain_autocontrolmin)
  {
    ROS_INFO("(vis) Set GainAutoControlMin = %f", config.GainAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMin", config.GainAutoControlMin);
  }

  if (changed_gain_autocontrolmax)
  {
    ROS_INFO("(vis) Set GainAutoControlMax = %f", config.GainAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMax", config.GainAutoControlMax);
  }

  if (changed_balancewhiteauto)
  {
    if (config.BalanceWhiteAuto.compare("Off") == 0)
      {
        gain_selector = "DigitalRed";
        arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
        config.GainDigitalRed = arv_camera_get_gain(p_camera_);

        gain_selector = "DigitalBlue";
        arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
        config.GainDigitalBlue = arv_camera_get_gain(p_camera_);
        
        ROS_INFO("(vis) Set BalanceWhiteAuto = %s", config.BalanceWhiteAuto.c_str());
        arv_device_set_string_feature_value(p_device_, "BalanceWhiteAuto", config.BalanceWhiteAuto.c_str());
      
        arv_device_set_float_feature_value(p_device_, "Gain", config.GainDigitalBlue);
        gain_selector = "DigitalRed";
        arv_device_set_string_feature_value(p_device_, "GainSelector", gain_selector.c_str());
        arv_device_set_float_feature_value(p_device_, "Gain", config.GainDigitalRed);

        config_vis_ = config;
        reconfigure_server_vis_->updateConfig(config_vis_);
      }
    else
    {
      ROS_INFO("(vis) Set BalanceWhiteAuto = %s", config.BalanceWhiteAuto.c_str());
      arv_device_set_string_feature_value(p_device_, "BalanceWhiteAuto", config.BalanceWhiteAuto.c_str());
    }
  }

  if (changed_awbcontrolspeed)
  {
    ROS_INFO("(vis) Set AWBControlSpeed = %i", config.AWBControlSpeed);
    arv_device_set_integer_feature_value(p_device_, "AWBControlSpeed", config.AWBControlSpeed);
  }

  if (changed_alcreference)
  {
    ROS_INFO("(vis) Set ALCReference = %i", config.ALCReference);
    arv_device_set_integer_feature_value(p_device_, "ALCReference", config.ALCReference);
  }

  if (changed_alccontrolspeed)
  {
    ROS_INFO("(vis) Set ALCControlSpeed = %i", config.ALCControlSpeed);
    arv_device_set_integer_feature_value(p_device_, "ALCControlSpeed", config.ALCControlSpeed);
  }

  if (changed_gamma)
  {
    ROS_INFO("(vis) Set Gamma = %f", config.Gamma);
    arv_device_set_float_feature_value(p_device_, "Gamma", config.Gamma);
  }

  // adopt new config
  config_vis_ = config;
  reconfigure_mutex_vis_.unlock();
}

void CameraAravisNodelet::rosReconfigureCallbackNir(ConfigNir &config, uint32_t level)
{
  reconfigure_mutex_nir_.lock();
  std::string tf_prefix = tf::getPrefixParam(getNodeHandle());
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  arv_camera_gv_select_stream_channel(p_camera_, 1);
  std::string source_control_nir = "Source1";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_nir.c_str());

  // Limit params to legal values.
  config.AcquisitionFrameRate = CLAMP(config.AcquisitionFrameRate, config_nir_min_.AcquisitionFrameRate,
                                      config_nir_max_.AcquisitionFrameRate);
  config.ExposureTime = CLAMP(config.ExposureTime, config_nir_min_.ExposureTime, config_nir_max_.ExposureTime);
  config.ExposureAutoControlMin = CLAMP(config.ExposureAutoControlMin, config_nir_min_.ExposureAutoControlMin, config_nir_max_.ExposureAutoControlMin);
  config.ExposureAutoControlMax = CLAMP(config.ExposureAutoControlMax, config_nir_min_.ExposureAutoControlMax, config_nir_max_.ExposureAutoControlMax);
  config.GainAnalogAll = CLAMP(config.GainAnalogAll, config_nir_min_.GainAnalogAll, config_nir_max_.GainAnalogAll);
  config.GainAutoControlMin = CLAMP(config.GainAutoControlMin, config_nir_min_.GainAutoControlMin, config_nir_max_.GainAutoControlMin);
  config.GainAutoControlMax = CLAMP(config.GainAutoControlMax, config_nir_min_.GainAutoControlMax, config_nir_max_.GainAutoControlMax);
  config.ALCReference = CLAMP(config.ALCReference, config_nir_min_.ALCReference, config_nir_max_.ALCReference);
  config.ALCControlSpeed = CLAMP(config.ALCControlSpeed, config_nir_min_.ALCControlSpeed, config_nir_max_.ALCControlSpeed);
  config.Gamma = CLAMP(config.Gamma, config_nir_min_.Gamma, config_nir_max_.Gamma);

  // reset values controlled by auto functions
  if (config.ExposureAuto.compare("Off") != 0)
  {
    config.ExposureTime = config_nir_.ExposureTime;
    ROS_WARN("(nir) ExposureAuto is active. Cannot manually set ExposureTime.");
  }
  if (config.GainAuto.compare("Off") != 0)
  {
    config.GainAnalogAll = config_nir_.GainAnalogAll;
    ROS_WARN("(nir) GainAuto is active. Cannot manually set Gain.");
  }

  // Find valid user changes we need to react to.
  const bool changed_acquisition_frame_rate = (config_nir_.AcquisitionFrameRate != config.AcquisitionFrameRate);
  const bool changed_exposure_auto = (config_nir_.ExposureAuto != config.ExposureAuto);
  const bool changed_exposure_time = (config_nir_.ExposureTime != config.ExposureTime);
  const bool changed_exposure_autocontrolmin = (config_nir_.ExposureAutoControlMin != config.ExposureAutoControlMin);
  const bool changed_exposure_autocontrolmax = (config_nir_.ExposureAutoControlMax != config.ExposureAutoControlMax);
  const bool changed_gain_auto = (config_nir_.GainAuto != config.GainAuto);
  const bool changed_gain = (config_nir_.GainAnalogAll != config.GainAnalogAll);
  const bool changed_gain_autocontrolmin = (config_nir_.GainAutoControlMin != config.GainAutoControlMin);
  const bool changed_gain_autocontrolmax = (config_nir_.GainAutoControlMax != config.GainAutoControlMax);
  const bool changed_alcreference = (config_nir_.ALCReference != config.ALCReference);
  const bool changed_alccontrolspeed = (config_nir_.ALCControlSpeed != config.ALCControlSpeed);
  const bool changed_gamma = (config_nir_.Gamma != config.Gamma);

  // Set params into the camera.
  if (changed_acquisition_frame_rate)
  {
    ROS_INFO("(vis, nir, nir2) Set AquisitionFrameRate = %f hz", config.AcquisitionFrameRate);
    arv_device_set_float_feature_value(p_device_, "AcquisitionFrameRate", config.AcquisitionFrameRate);
    config_vis_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_vis_->updateConfig(config_vis_);
    config_nir2_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_nir2_->updateConfig(config_nir2_);
    config_nir_ = config;

    if (sync_mode == "SyncMode") {
      adaptExposureSettingsToFPS();
    }
    config = config_nir_;

    //reset source selector
    arv_camera_gv_select_stream_channel(p_camera_, 1);
    source_control_nir = "Source1";
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_nir.c_str());
  }
  
  if (changed_exposure_auto)
  {
    ROS_INFO("(nir) Set ExposureAuto = %s", config.ExposureAuto.c_str());
    config.ExposureTime = arv_camera_get_exposure_time(p_camera_);
    arv_device_set_string_feature_value(p_device_, "ExposureAuto", config.ExposureAuto.c_str());
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir) Get ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
        config_nir_ = config;
        reconfigure_server_nir_->updateConfig(config_nir_);
      }
  }

  if (changed_exposure_time)
  {
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir) Set ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
      }
    else
      {
        ROS_WARN("(nir) ExposureAuto is active. Cannot manually set ExposureTime.");
      }
  }
  
  if (changed_exposure_autocontrolmin)
  {
    ROS_INFO("(nir) Set ExposureAutoControlMin = %f us", config.ExposureAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config.ExposureAutoControlMin);
  }
    
  if (changed_exposure_autocontrolmax)
  {
    ROS_INFO("(nir) Set ExposureAutoControlMax = %f us", config.ExposureAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config.ExposureAutoControlMax);
  }

  if (changed_gain_auto)
  {
    ROS_INFO("(nir) Set GainAuto = %s", config.GainAuto.c_str());
    config.GainAnalogAll = arv_camera_get_gain(p_camera_);
    arv_device_set_string_feature_value(p_device_, "GainAuto", config.GainAuto.c_str());
    if (config.GainAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir) Get Gain = %f", config.GainAnalogAll);
        arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
        config_nir_ = config;
        reconfigure_server_nir_->updateConfig(config_nir_);
      }
  }

  if (changed_gain)
  {
    ROS_INFO("(nir) Set Gain = %f", config.GainAnalogAll);
    arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
  }

  if (changed_gain_autocontrolmin)
  {
    ROS_INFO("(nir) Set GainAutoControlMin = %f", config.GainAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMin", config.GainAutoControlMin);
  }

  if (changed_gain_autocontrolmax)
  {
    ROS_INFO("(nir) Set GainAutoControlMax = %f", config.GainAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMax", config.GainAutoControlMax);
  }

  if (changed_alcreference)
  {
    ROS_INFO("(nir) Set ALCReference = %i", config.ALCReference);
    arv_device_set_integer_feature_value(p_device_, "ALCReference", config.ALCReference);
  }

  if (changed_alccontrolspeed)
  {
    ROS_INFO("(nir) Set ALCControlSpeed = %i", config.ALCControlSpeed);
    arv_device_set_integer_feature_value(p_device_, "ALCControlSpeed", config.ALCControlSpeed);
  }

  if (changed_gamma)
  {
    ROS_INFO("(nir) Set Gamma = %f", config.Gamma);
    arv_device_set_float_feature_value(p_device_, "Gamma", config.Gamma);
  }

  // adopt new config
  config_nir_ = config;
  reconfigure_mutex_nir_.unlock();
}

void CameraAravisNodelet::rosReconfigureCallbackNir2(ConfigNir &config, uint32_t level)
{
  reconfigure_mutex_nir2_.lock();
  std::string tf_prefix = tf::getPrefixParam(getNodeHandle());
  ROS_DEBUG_STREAM("tf_prefix: " << tf_prefix);

  arv_camera_gv_select_stream_channel(p_camera_, 2);
  std::string source_control_nir2 = "Source2";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_nir2.c_str());

  // Limit params to legal values.
  config.AcquisitionFrameRate = CLAMP(config.AcquisitionFrameRate, config_nir2_min_.AcquisitionFrameRate,
                                      config_nir2_max_.AcquisitionFrameRate);
  config.ExposureTime = CLAMP(config.ExposureTime, config_nir2_min_.ExposureTime, config_nir2_max_.ExposureTime);
  config.ExposureAutoControlMin = CLAMP(config.ExposureAutoControlMin, config_nir2_min_.ExposureAutoControlMin, config_nir2_max_.ExposureAutoControlMin);
  config.ExposureAutoControlMax = CLAMP(config.ExposureAutoControlMax, config_nir2_min_.ExposureAutoControlMax, config_nir2_max_.ExposureAutoControlMax);
  config.GainAnalogAll = CLAMP(config.GainAnalogAll, config_nir2_min_.GainAnalogAll, config_nir2_max_.GainAnalogAll);
  config.GainAutoControlMin = CLAMP(config.GainAutoControlMin, config_nir2_min_.GainAutoControlMin, config_nir2_max_.GainAutoControlMin);
  config.GainAutoControlMax = CLAMP(config.GainAutoControlMax, config_nir2_min_.GainAutoControlMax, config_nir2_max_.GainAutoControlMax);
  config.ALCReference = CLAMP(config.ALCReference, config_nir2_min_.ALCReference, config_nir2_max_.ALCReference);
  config.ALCControlSpeed = CLAMP(config.ALCControlSpeed, config_nir2_min_.ALCControlSpeed, config_nir2_max_.ALCControlSpeed);
  config.Gamma = CLAMP(config.Gamma, config_nir2_min_.Gamma, config_nir2_max_.Gamma);

  // reset values controlled by auto functions
  if (config.ExposureAuto.compare("Off") != 0)
  {
    config.ExposureTime = config_nir2_.ExposureTime;
    ROS_WARN("(nir2) ExposureAuto is active. Cannot manually set ExposureTime.");
  }
  if (config.GainAuto.compare("Off") != 0)
  {
    config.GainAnalogAll = config_nir2_.GainAnalogAll;
    ROS_WARN("(nir2) GainAuto is active. Cannot manually set Gain.");
  }

  // Find valid user changes we need to react to.
  const bool changed_acquisition_frame_rate = (config_nir2_.AcquisitionFrameRate != config.AcquisitionFrameRate);
  const bool changed_exposure_auto = (config_nir2_.ExposureAuto != config.ExposureAuto);
  const bool changed_exposure_time = (config_nir2_.ExposureTime != config.ExposureTime);
  const bool changed_exposure_autocontrolmin = (config_nir2_.ExposureAutoControlMin != config.ExposureAutoControlMin);
  const bool changed_exposure_autocontrolmax = (config_nir2_.ExposureAutoControlMax != config.ExposureAutoControlMax);
  const bool changed_gain_auto = (config_nir2_.GainAuto != config.GainAuto);
  const bool changed_gain = (config_nir2_.GainAnalogAll != config.GainAnalogAll);
  const bool changed_gain_autocontrolmin = (config_nir2_.GainAutoControlMin != config.GainAutoControlMin);
  const bool changed_gain_autocontrolmax = (config_nir2_.GainAutoControlMax != config.GainAutoControlMax);
  const bool changed_alcreference = (config_nir2_.ALCReference != config.ALCReference);
  const bool changed_alccontrolspeed = (config_nir2_.ALCControlSpeed != config.ALCControlSpeed);
  const bool changed_gamma = (config_nir2_.Gamma != config.Gamma);

  // Set params into the camera.
  if (changed_acquisition_frame_rate)
  {
    ROS_INFO("(vis, nir, nir2) Set AquisitionFrameRate = %f hz", config.AcquisitionFrameRate);
    arv_device_set_float_feature_value(p_device_, "AcquisitionFrameRate", config.AcquisitionFrameRate);
    config_nir_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_nir_->updateConfig(config_nir_);
    config_vis_.AcquisitionFrameRate = config.AcquisitionFrameRate;
    reconfigure_server_vis_->updateConfig(config_vis_);
    config_nir2_ = config;

    if (sync_mode == "SyncMode") {
      adaptExposureSettingsToFPS();
    }
    config = config_nir2_;

    //reset source selector
    arv_camera_gv_select_stream_channel(p_camera_, 2);
    source_control_nir2 = "Source2";
    arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control_nir2.c_str());
  }
  
  if (changed_exposure_auto)
  {
    ROS_INFO("(nir2) Set ExposureAuto = %s", config.ExposureAuto.c_str());
    config.ExposureTime = arv_camera_get_exposure_time(p_camera_);
    arv_device_set_string_feature_value(p_device_, "ExposureAuto", config.ExposureAuto.c_str());
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir2) Get ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
        config_nir2_ = config;
        reconfigure_server_nir2_->updateConfig(config_nir2_);
      }
  }

  if (changed_exposure_time)
  {
    if (config.ExposureAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir2) Set ExposureTime = %f us", config.ExposureTime);
        arv_device_set_float_feature_value(p_device_, "ExposureTime", config.ExposureTime);
      }
    else
      {
        ROS_WARN("(nir2) ExposureAuto is active. Cannot manually set ExposureTime.");
      }
  }
  
  if (changed_exposure_autocontrolmin)
  {
    ROS_INFO("(nir2) Set ExposureAutoControlMin = %f us", config.ExposureAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config.ExposureAutoControlMin);
  }
    
  if (changed_exposure_autocontrolmax)
  {
    ROS_INFO("(nir2) Set ExposureAutoControlMax = %f us", config.ExposureAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config.ExposureAutoControlMax);
  }

  if (changed_gain_auto)
  {
    ROS_INFO("(nir2) Set GainAuto = %s", config.GainAuto.c_str());
    config.GainAnalogAll = arv_camera_get_gain(p_camera_);
    arv_device_set_string_feature_value(p_device_, "GainAuto", config.GainAuto.c_str());
    if (config.GainAuto.compare("Off") == 0)
      {
        ROS_INFO("(nir2) Get Gain = %f", config.GainAnalogAll);
        arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
        config_nir2_ = config;
        reconfigure_server_nir2_->updateConfig(config_nir2_);
      }
  }

  if (changed_gain)
  {
    ROS_INFO("(nir2) Set Gain = %f", config.GainAnalogAll);
    arv_device_set_float_feature_value(p_device_, "Gain", config.GainAnalogAll);
  }

  if (changed_gain_autocontrolmin)
  {
    ROS_INFO("(nir2) Set GainAutoControlMin = %f", config.GainAutoControlMin);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMin", config.GainAutoControlMin);
  }

  if (changed_gain_autocontrolmax)
  {
    ROS_INFO("(nir2) Set GainAutoControlMax = %f", config.GainAutoControlMax);
    arv_device_set_float_feature_value(p_device_, "GainAutoControlMax", config.GainAutoControlMax);
  }

  if (changed_alcreference)
  {
    ROS_INFO("(nir2) Set ALCReference = %i", config.ALCReference);
    arv_device_set_integer_feature_value(p_device_, "ALCReference", config.ALCReference);
  }

  if (changed_alccontrolspeed)
  {
    ROS_INFO("(nir2) Set ALCControlSpeed = %i", config.ALCControlSpeed);
    arv_device_set_integer_feature_value(p_device_, "ALCControlSpeed", config.ALCControlSpeed);
  }

  if (changed_gamma)
  {
    ROS_INFO("(nir2) Set Gamma = %f", config.Gamma);
    arv_device_set_float_feature_value(p_device_, "Gamma", config.Gamma);
  }

  // adopt new config
  config_nir2_ = config;
  reconfigure_mutex_nir2_.unlock();
}

void CameraAravisNodelet::rosConnectCallback()
{
  if (p_device_)
  {
    if (std::all_of(cam_pubs_.begin(), cam_pubs_.end(),
      [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() == 0; })
    ){
      arv_device_execute_command(p_device_, "AcquisitionStop"); // don't waste CPU if nobody is listening!
    }
    else
    {
      arv_device_execute_command(p_device_, "AcquisitionStart");
    }
  }
}

void CameraAravisNodelet::newBufferReadyCallback(ArvStream *p_stream, gpointer can_instance)
{

  // workaround to get access to the instance from a static method
  StreamIdData *data = (StreamIdData*) can_instance;
  CameraAravisNodelet *p_can = (CameraAravisNodelet*) data->can;
  size_t stream_id = data->stream_id;
  image_transport::CameraPublisher *p_cam_pub = &p_can->cam_pubs_[stream_id];

  std::string stream_frame_id = "";
  // extend frame id
  if( !p_can->stream_names_[stream_id].empty() )
  {
    stream_frame_id += p_can->stream_names_[stream_id];
  }

  newBufferReady(p_stream, p_can, stream_frame_id, stream_id);

}

void CameraAravisNodelet::newBufferReady(ArvStream *p_stream, CameraAravisNodelet *p_can, std::string frame_id, size_t stream_id)
{
  ArvBuffer *p_buffer = arv_stream_try_pop_buffer(p_stream);

  // check if we risk to drop the next image because of not enough buffers left
  gint n_available_buffers;
  arv_stream_get_n_buffers(p_stream, &n_available_buffers, NULL);
  if (n_available_buffers == 0)
  {
    p_can->p_buffer_pools_[stream_id]->allocateBuffers(1);
  }

  if (p_buffer != NULL)
  {
    if (arv_buffer_get_status(p_buffer) == ARV_BUFFER_STATUS_SUCCESS && p_can->p_buffer_pools_[stream_id]
        && p_can->cam_pubs_[stream_id].getNumSubscribers())
    {
      (p_can->n_buffers_)++;
      // get the image message which wraps around this buffer
      sensor_msgs::ImagePtr msg_ptr = (*(p_can->p_buffer_pools_[stream_id]))[p_buffer];
      // fill the meta information of image message
      // get acquisition time
      const guint64 t = arv_buffer_get_system_timestamp(p_buffer);
      msg_ptr->header.stamp.fromNSec(t);
      // get frame sequence number
      msg_ptr->header.seq = arv_buffer_get_frame_id(p_buffer);
      // fill other stream properties
      if (frame_id.empty()) {
        msg_ptr->header.frame_id = p_can->getName(); 
      } else {
        msg_ptr->header.frame_id = p_can->getName()  + '/' + frame_id; 
      }
      msg_ptr->width = p_can->roi_.width;
      msg_ptr->height = p_can->roi_.height;
      msg_ptr->encoding = p_can->sensors_[stream_id]->pixel_format;
      msg_ptr->step = (msg_ptr->width * p_can->sensors_[stream_id]->n_bits_pixel)/8;

      // do the magic of conversion into a ROS format
      if (p_can->convert_format[stream_id]) {
        sensor_msgs::ImagePtr cvt_msg_ptr = p_can->p_buffer_pools_[stream_id]->getRecyclableImg();
        p_can->convert_format[stream_id](msg_ptr, cvt_msg_ptr);
        msg_ptr = cvt_msg_ptr;
      }

      // get current CameraInfo data
      if (!p_can->camera_infos_[stream_id]) {
        p_can->camera_infos_[stream_id].reset(new sensor_msgs::CameraInfo);
      }
      (*p_can->camera_infos_[stream_id]) = p_can->p_camera_info_managers_[stream_id]->getCameraInfo();
      p_can->camera_infos_[stream_id]->header = msg_ptr->header;
      p_can->camera_infos_[stream_id]->width = p_can->roi_.width;
      p_can->camera_infos_[stream_id]->height = p_can->roi_.height;

      p_can->cam_pubs_[stream_id].publish(msg_ptr, p_can->camera_infos_[stream_id]);

    }
    else
    {
      ROS_WARN("(%s) Frame error: %s", frame_id.c_str(), szBufferStatusFromInt[arv_buffer_get_status(p_buffer)]);

      arv_stream_push_buffer(p_stream, p_buffer);
    }
  }

  /*
  // publish current lighting settings if this camera is configured as master
  if (p_can->config_.AutoMaster)
  {
    p_can->syncAutoParameters();
    p_can->auto_pub_.publish(p_can->auto_params_);
  }
  */
}

void CameraAravisNodelet::controlLostCallback(ArvDevice *p_gv_device, gpointer can_instance)
{
  CameraAravisNodelet *p_can = (CameraAravisNodelet*)can_instance;
  ROS_ERROR("Control to aravis device lost.");
  nodelet::NodeletUnload unload_service;
  unload_service.request.name = p_can->getName();
  if (false == ros::service::call(ros::this_node::getName() + "/unload_nodelet", unload_service))
  {
    ros::shutdown();
  }
}

/*
void CameraAravisNodelet::softwareTriggerLoop()
{
  software_trigger_active_ = true;
  ROS_INFO("Software trigger started.");
  std::chrono::system_clock::time_point next_time = std::chrono::system_clock::now();
  while (ros::ok() && software_trigger_active_)
  {
    next_time += std::chrono::milliseconds(size_t(std::round(1000.0 / config_.softwaretriggerrate)));
    if (std::any_of(cam_pubs_.begin(), cam_pubs_.end(),
      [](image_transport::CameraPublisher pub){ return pub.getNumSubscribers() > 0; })
    )
    {
      arv_device_execute_command(p_device_, "TriggerSoftware");
    }
    if (next_time > std::chrono::system_clock::now())
    {
      std::this_thread::sleep_until(next_time);
    }
    else
    {
      ROS_WARN("Camera Aravis: Missed a software trigger event.");
      next_time = std::chrono::system_clock::now();
    }
  }
  ROS_INFO("Software trigger stopped.");
}
*/

void CameraAravisNodelet::publishTfLoop(double rate)
{
  // Publish optical transform for the camera
  ROS_WARN("Publishing dynamic camera transforms (/tf) at %g Hz", rate);

  tf_thread_active_ = true;

  ros::Rate loop_rate(rate);

  while (ros::ok() && tf_thread_active_)
  {
    // Update the header for publication
    tf_optical_.header.stamp = ros::Time::now();
    ++tf_optical_.header.seq;
    p_tb_->sendTransform(tf_optical_);

    loop_rate.sleep();
  }
}

void CameraAravisNodelet::discoverFeatures()
{
  implemented_features_.clear();
  if (!p_device_)
    return;

  // get the root node of genicam description
  ArvGc *gc = arv_device_get_genicam(p_device_);
  if (!gc)
    return;

  std::list<ArvDomNode*> todo;
  todo.push_front((ArvDomNode*)arv_gc_get_node(gc, "Root"));
  GError *error = NULL;

  while (!todo.empty())
  {
    // get next entry
    ArvDomNode *node = todo.front();
    todo.pop_front();
    const std::string name(arv_dom_node_get_node_name(node));

    // Do the indirection
    if (name[0] == 'p')
    {
      if (name.compare("pInvalidator") == 0)
      {
        continue;
      }
      ArvDomNode *inode = (ArvDomNode*)arv_gc_get_node(gc,
                                                       arv_dom_node_get_node_value(arv_dom_node_get_first_child(node)));
      if (inode)
        todo.push_front(inode);
      continue;
    }

    // check for implemented feature
    if (ARV_IS_GC_FEATURE_NODE(node))
    {
      //if (!(ARV_IS_GC_CATEGORY(node) || ARV_IS_GC_ENUM_ENTRY(node) /*|| ARV_IS_GC_PORT(node)*/)) {
      ArvGcFeatureNode *fnode = ARV_GC_FEATURE_NODE(node);
      const std::string fname(arv_gc_feature_node_get_name(fnode));
      const bool usable = arv_gc_feature_node_is_available(fnode, &error)
          && arv_gc_feature_node_is_implemented(fnode, &error);
      //ROS_INFO_STREAM("Feature " << fname << " is " << usable);
      implemented_features_.emplace(fname, usable);
      //}
    }

//		if (ARV_IS_GC_PROPERTY_NODE(node)) {
//			ArvGcPropertyNode* pnode = ARV_GC_PROPERTY_NODE(node);
//			const std::string pname(arv_gc_property_node_get_name(pnode));
//			ROS_INFO_STREAM("Property " << pname << " found");
//		}

    // add children in todo-list
    ArvDomNodeList *children = arv_dom_node_get_child_nodes(node);
    const uint l = arv_dom_node_list_get_length(children);
    for (uint i = 0; i < l; ++i)
    {
      todo.push_front(arv_dom_node_list_get_item(children, i));
    }
  }
}

void CameraAravisNodelet::parseStringArgs(std::string in_arg_string, std::vector<std::string> &out_args) {
  size_t array_start = in_arg_string.find('[');
  size_t array_end = in_arg_string.find(']');
  if(array_start != std::string::npos && array_end != std::string::npos) {
        // parse the string into an array of parameters
        array_start++;  // do not include '[' in string
        std::stringstream ss(in_arg_string.substr(array_start, array_end - array_start));
        while (ss.good()) {
          std::string temp;
          getline( ss, temp, ',');
          boost::trim_left(temp);
          boost::trim_right(temp);
          out_args.push_back(temp);
        }
  } else {
    // add just the one argument onto the vector
    out_args.push_back(in_arg_string);
  }
}

// WriteCameraFeaturesFromRosparam()
// Read ROS parameters from this node's namespace, and see if each parameter has a similarly named & typed feature in the camera.  Then set the
// camera feature to that value.  For example, if the parameter camnode/Gain is set to 123.0, then we'll write 123.0 to the Gain feature
// in the camera.
//
// Note that the datatype of the parameter *must* match the datatype of the camera feature, and this can be determined by
// looking at the camera's XML file.  Camera enum's are string parameters, camera bools are false/true parameters (not 0/1),
// integers are integers, doubles are doubles, etc.
//
void CameraAravisNodelet::writeCameraFeaturesFromRosparam()
{
  XmlRpc::XmlRpcValue xml_rpc_params;
  XmlRpc::XmlRpcValue::iterator iter;
  ArvGcNode *p_gc_node;
  GError *error = NULL;

  getPrivateNodeHandle().getParam(this->getName(), xml_rpc_params);

  if (xml_rpc_params.getType() == XmlRpc::XmlRpcValue::TypeStruct)
  {
    for (iter = xml_rpc_params.begin(); iter != xml_rpc_params.end(); iter++)
    {
      std::string key = iter->first;

      p_gc_node = arv_device_get_feature(p_device_, key.c_str());
      if (p_gc_node && arv_gc_feature_node_is_implemented(ARV_GC_FEATURE_NODE(p_gc_node), &error))
      {
        //				unsigned long	typeValue = arv_gc_feature_node_get_value_type((ArvGcFeatureNode *)pGcNode);
        //				ROS_INFO("%s cameratype=%lu, rosparamtype=%d", key.c_str(), typeValue, static_cast<int>(iter->second.getType()));

        // We'd like to check the value types too, but typeValue is often given as G_TYPE_INVALID, so ignore it.
        switch (iter->second.getType())
        {
          case XmlRpc::XmlRpcValue::TypeBoolean: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeBoolean))// && (typeValue==G_TYPE_INT64))
          {
            int value = (bool)iter->second;
            arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
            //ROS_INFO("Read parameter (bool) %s: %d", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeInt: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeInt))// && (typeValue==G_TYPE_INT64))
          {
            int value = (int)iter->second;
            arv_device_set_integer_feature_value(p_device_, key.c_str(), value);
            //ROS_INFO("Read parameter (int) %s: %d", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeDouble: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeDouble))// && (typeValue==G_TYPE_DOUBLE))
          {
            double value = (double)iter->second;
            arv_device_set_float_feature_value(p_device_, key.c_str(), value);
            //ROS_INFO("Read parameter (float) %s: %f", key.c_str(), value);
          }
            break;

          case XmlRpc::XmlRpcValue::TypeString: //if ((iter->second.getType()==XmlRpc::XmlRpcValue::TypeString))// && (typeValue==G_TYPE_STRING))
          {
            std::string value = (std::string)iter->second;
            arv_device_set_string_feature_value(p_device_, key.c_str(), value.c_str());
            //ROS_INFO("Read parameter (string) %s: %s", key.c_str(), value.c_str());
          }
            break;

          case XmlRpc::XmlRpcValue::TypeInvalid:
          case XmlRpc::XmlRpcValue::TypeDateTime:
          case XmlRpc::XmlRpcValue::TypeBase64:
          case XmlRpc::XmlRpcValue::TypeArray:
          case XmlRpc::XmlRpcValue::TypeStruct:
          default:
            ROS_WARN("Unhandled rosparam type in writeCameraFeaturesFromRosparam()");
        }
      }
    }
  }
}

void CameraAravisNodelet::adaptExposureSettingsToFPS()
{
  std::string source_control = "Source0";
  // get new boundary values for Exposure time related parameters, update the min-/max-config of all 3 channels
  double ExposureAutoControlMin_min;
  double ExposureAutoControlMin_max;
  double ExposureAutoControlMax_min;
  double ExposureAutoControlMax_max;
  double ExposureTime_min;
  double ExposureTime_max;
  //vis
  arv_camera_gv_select_stream_channel(p_camera_, 0);
  source_control = "Source0";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control.c_str());

  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMin_min, &ExposureAutoControlMin_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMax_min, &ExposureAutoControlMax_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureTime", &ExposureTime_min, &ExposureTime_max);

  config_vis_min_.ExposureAutoControlMin = ExposureAutoControlMin_min;
  config_vis_max_.ExposureAutoControlMin = ExposureAutoControlMin_max;
  config_vis_min_.ExposureAutoControlMax = ExposureAutoControlMax_min;
  config_vis_max_.ExposureAutoControlMax = ExposureAutoControlMax_max;
  config_vis_min_.ExposureTime = ExposureTime_min;
  config_vis_max_.ExposureTime = ExposureTime_max;

  if (arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax") > config_vis_max_.ExposureAutoControlMax)
  {
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config_vis_max_.ExposureAutoControlMax);
  }
  
  arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config_vis_min_.ExposureAutoControlMin);
  

  config_vis_.ExposureAutoControlMax = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax");
  config_vis_.ExposureAutoControlMin = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMin");
  //config_vis_.ExposureTime = arv_device_get_float_feature_value(p_device_, "ExposureTime");

  //nir
  arv_camera_gv_select_stream_channel(p_camera_, 1);
  source_control = "Source1";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control.c_str());

  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMin_min, &ExposureAutoControlMin_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMax_min, &ExposureAutoControlMax_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureTime", &ExposureTime_min, &ExposureTime_max);

  config_nir_min_.ExposureAutoControlMin = ExposureAutoControlMin_min;
  config_nir_max_.ExposureAutoControlMin = ExposureAutoControlMin_max;
  config_nir_min_.ExposureAutoControlMax = ExposureAutoControlMax_min;
  config_nir_max_.ExposureAutoControlMax = ExposureAutoControlMax_max;
  config_nir_min_.ExposureTime = ExposureTime_min;
  config_nir_max_.ExposureTime = ExposureTime_max;

  if (arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax") > config_nir_max_.ExposureAutoControlMax)
  {
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config_nir_max_.ExposureAutoControlMax);
  }

  arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config_nir_min_.ExposureAutoControlMin);

  config_nir_.ExposureAutoControlMax = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax");
  config_nir_.ExposureAutoControlMin = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMin");
  //config_nir_.ExposureTime = arv_device_get_float_feature_value(p_device_, "ExposureTime");

  //nir2
  arv_camera_gv_select_stream_channel(p_camera_, 2);
  source_control = "Source2";
  arv_device_set_string_feature_value(p_device_, "SourceSelector", source_control.c_str());

  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMin_min, &ExposureAutoControlMin_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureAutoControlMax", &ExposureAutoControlMax_min, &ExposureAutoControlMax_max);
  arv_device_get_float_feature_bounds(p_device_, "ExposureTime", &ExposureTime_min, &ExposureTime_max);

  config_nir2_min_.ExposureAutoControlMin = ExposureAutoControlMin_min;
  config_nir2_max_.ExposureAutoControlMin = ExposureAutoControlMin_max;
  config_nir2_min_.ExposureAutoControlMax = ExposureAutoControlMax_min;
  config_nir2_max_.ExposureAutoControlMax = ExposureAutoControlMax_max;
  config_nir2_min_.ExposureTime = ExposureTime_min;
  config_nir2_max_.ExposureTime = ExposureTime_max;

  if (arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax") > config_nir2_max_.ExposureAutoControlMax)
  {
    arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMax", config_nir2_max_.ExposureAutoControlMax);
  }

  arv_device_set_float_feature_value(p_device_, "ExposureAutoControlMin", config_nir2_min_.ExposureAutoControlMin);

  config_nir2_.ExposureAutoControlMax = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMax");
  config_nir2_.ExposureAutoControlMin = arv_device_get_float_feature_value(p_device_, "ExposureAutoControlMin");
  //config_nir2_.ExposureTime = arv_device_get_float_feature_value(p_device_, "ExposureTime");
    
  reconfigure_server_vis_->setConfigMin(config_vis_min_);
  reconfigure_server_vis_->setConfigMax(config_vis_max_);
  reconfigure_server_nir_->setConfigMin(config_nir_min_);
  reconfigure_server_nir_->setConfigMax(config_nir_max_);
  reconfigure_server_nir2_->setConfigMin(config_nir2_min_);
  reconfigure_server_nir2_->setConfigMax(config_nir2_max_);

  reconfigure_server_vis_->updateConfig(config_vis_);
  reconfigure_server_nir_->updateConfig(config_nir_);
  reconfigure_server_nir2_->updateConfig(config_nir2_);
}

} // end namespace camera_aravis
