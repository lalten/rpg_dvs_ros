// This file is part of DVS-ROS - the RPG DVS ROS Package
//
// DVS-ROS is free software: you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
//
// DVS-ROS is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with DVS-ROS.  If not, see <http://www.gnu.org/licenses/>.

#include "dvs_calibration/dvs_calibration.h"

namespace dvs_calibration {

DvsCalibration::DvsCalibration()
{
  start_calibration_service_ = nh_.advertiseService("dvs_calibration/start", &DvsCalibration::startCalibrationCallback, this);
  save_calibration_service_ = nh_.advertiseService("dvs_calibration/save", &DvsCalibration::saveCalibrationCallback, this);
  reset_calibration_service_ = nh_.advertiseService("dvs_calibration/reset", &DvsCalibration::resetCalibrationCallback, this);

  num_detections_pub_ = nh_.advertise<std_msgs::Int32>("dvs_calibration/pattern_detections", 1);
  calibration_output_pub_ = nh_.advertise<std_msgs::String>("dvs_calibration/output", 1);

  detected_points_left_or_single_pub_ = nh_.advertise<dvs_msgs::ImageObjectPoints>("dvs_calibration/detected_points_left_or_single", 1);
  detected_points_right_pub_ = nh_.advertise<dvs_msgs::ImageObjectPoints>("dvs_calibration/detected_points_right", 1);


  image_transport::ImageTransport it(nh_);
  detected_points_left_or_single_pattern_pub_ = it.advertise("dvs_calibration/detected_points_left_or_single_pattern", 1);
  detected_points_right_pattern_pub_ = it.advertise("dvs_calibration/detected_points_right_pattern", 1);


  calibration_running_ = false;
  num_detections_ = 0;

  // load parameters
  loadCalibrationParameters();

  for (int i = 0; i < params_.dots_h; i++)
  {
    for (int j = 0; j < params_.dots_w; j++)
    {
      world_pattern_.push_back(cv::Point3f(i * params_.dot_distance , j * params_.dot_distance , 0.0));
    }
  }
}

void DvsCalibration::publishAddedPattern(const int id, ros::Publisher &detected_points_pub,
		image_transport::Publisher &detected_points_patttern_pub, std::vector<cv::Point2f> image_point_v, cv::Mat image_pattern)
{
  //publish detection points
  //can be used for rosbag recordings and inspect the detected points or
  //to store them with rosbag and potentially play them back later, e.g. for calibration again
  dvs_msgs::ImageObjectPoints image_object_points_msg;

  dvs_msgs::Point2f image_point;
  for (cv::Point2f pp : image_point_v) {
	  image_point.x = pp.x;
	  image_point.y = pp.y;
	  image_object_points_msg.image_points.push_back(image_point);
  }
  dvs_msgs::Point3f image_point3;
	for (cv::Point3f pp : world_pattern_) {
	  image_point3.x = pp.x;
	  image_point3.y = pp.y;
	  image_point3.z = pp.z;
	  image_object_points_msg.object_points.push_back(image_point3);
	}
	detected_points_pub.publish(image_object_points_msg);

  //publish detection transition image for the detected points
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image_pattern.clone();

  detected_points_patttern_pub.publish(cv_image.toImageMsg());
}

void DvsCalibration::eventsCallback(const dvs_msgs::EventArray::ConstPtr& msg, int camera_id)
{
  if (calibration_running_)
    return;

  sensor_width_ = msg->width;
  sensor_height_ = msg->height;

  transition_maps_[camera_id].update(msg);
  if (transition_maps_[camera_id].max() > params_.enough_transitions_threshold) {
    transition_maps_[camera_id].find_pattern();
    ROS_DEBUG("Try to find pattern");
    if (transition_maps_[camera_id].has_pattern()) {
      ROS_DEBUG("Found pattern.");
      addPattern(camera_id);

      std_msgs::Int32 msg;
      msg.data = num_detections_;
      num_detections_pub_.publish(msg);

      updateVisualization(camera_id);

      //reset transition map, because we found a pattern
      transition_maps_[camera_id].reset_maps();
    }


  }
  else {
    updateVisualization(camera_id);
  }

  // reset if nothing is found after certain amount of time
  if (ros::Time::now() - transition_maps_[camera_id].get_last_reset_time() > ros::Duration(params_.pattern_search_timeout)) {
    ROS_DEBUG("Reset maps because of time.");
    transition_maps_[camera_id].reset_maps();
  }
}

void DvsCalibration::loadCalibrationParameters()
{
  ros::NodeHandle nh_private("~");
  nh_private.param<int>("blinking_time_us", params_.blinking_time_us, 1000);
  nh_private.param<int>("blinking_time_tolerance_us", params_.blinking_time_tolerance_us, 500);
  nh_private.param<int>("enough_transitions_threshold", params_.enough_transitions_threshold, 200);
  nh_private.param<int>("minimum_transitions_threshold", params_.minimum_transitions_threshold, 10);
  nh_private.param<int>("minimum_led_mass", params_.minimum_led_mass, 50);
  nh_private.param<int>("dots_w", params_.dots_w, 5);
  nh_private.param<int>("dots_h", params_.dots_h, 5);
  nh_private.param<double>("dot_distance", params_.dot_distance, 0.05);
  nh_private.param<double>("pattern_search_timeout", params_.pattern_search_timeout, 2.0);  
}

bool DvsCalibration::resetCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Reset call");
  resetCalibration();
  return true;
}

bool DvsCalibration::startCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Start calibration call");
  startCalibration();
  return true;
}

bool DvsCalibration::saveCalibrationCallback(std_srvs::Empty::Request& request, std_srvs::Empty::Response& response)
{
  ROS_INFO("Save calibration call");
  saveCalibration();
  return true;
}

} // namespace

