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

#include "dvs_calibration/mono_dvs_calibration.h"

namespace dvs_calibration {

MonoDvsCalibration::MonoDvsCalibration()
{
  calibration_running_ = false;

  set_camera_info_client_ = nh_.serviceClient<sensor_msgs::SetCameraInfo>("set_camera_info");

  // add transition map
  transition_maps_.insert(std::pair<int, TransitionMap>(mono_camera_id, TransitionMap(params_)));
  event_sub_ = nh_.subscribe<dvs_msgs::EventArray>("events", 1,
                                                       boost::bind(&MonoDvsCalibration::eventsCallback, this, _1, mono_camera_id));

  image_transport::ImageTransport it(nh_);
  pattern_pub_ = it.advertise("dvs_calibration/pattern", 1);
  visualization_pub_ = it.advertise("dvs_calibration/visualization", 1);

  camera_info_sub_ = nh_.subscribe("camera_info", 1, &MonoDvsCalibration::cameraInfoCallback, this);
  got_camera_info_ = false;
  camera_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>("dvs_calibration/pose", 1);

  image_object_points_sub_ = nh_.subscribe("dvs_calibration/image_object_points", 1,
		  	  	  	  	  	  	  &MonoDvsCalibration::imageObjectPointsCallback, this);
}

void MonoDvsCalibration::calibrate()
{
  // run camera calibration
  cv::Mat cameraMatrix, distCoeffs;
  std::vector<cv::Mat> rvecs, tvecs;
  double reproj_error = cv::calibrateCamera(object_points_, image_points_, cv::Size(sensor_width_, sensor_height_),
                                            cameraMatrix, distCoeffs, rvecs, tvecs, CV_CALIB_FIX_K3);

  // update camera info
  new_camera_info_.height = sensor_height_;
  new_camera_info_.width = sensor_width_;
  new_camera_info_.distortion_model = "plumb_bob";

  new_camera_info_.D.clear();
  for (int i = 0; i < 5; i++)
    new_camera_info_.D.push_back(distCoeffs.at<double>(i));
  for (int i = 0; i < 9; i++)
    new_camera_info_.K[i] = cameraMatrix.at<double>(i);

  // send output
  std::ostringstream output;
  output << "Calibration result" << std::endl;
  output << "Reprojection error: " << std::setprecision(5) << reproj_error << std::endl;
  output << "Camera matrix (K):" << std::endl;
  for (int i = 0; i < 9; i++)
  {
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << new_camera_info_.K[i] << "  ";
    if (i%3 == 2)
      output << std::endl;
  }
  output << "Distortion coefficients (D):" << std::endl;
  for (int i = 0; i < 5; i++)
    output << std::setfill(' ') << std::setw(6) << std::setprecision(5) << new_camera_info_.D[i] << "  ";
  output << std::endl;

  std_msgs::String output_msg;
  output_msg.data = output.str();
  calibration_output_pub_.publish(output_msg);
}

void MonoDvsCalibration::publishVisualizationImage(cv::Mat image)
{
  cv_bridge::CvImage cv_image;
  cv_image.encoding = "bgr8";
  cv_image.image = image.clone();

  visualization_pub_.publish(cv_image.toImageMsg());
}

bool MonoDvsCalibration::setCameraInfo()
{
  sensor_msgs::SetCameraInfo srv;
  srv.request.camera_info = new_camera_info_;
  return set_camera_info_client_.call(srv);
}

void MonoDvsCalibration::resetCalibration()
{
  transition_maps_[mono_camera_id].reset_maps();
  calibration_running_ = false;
  object_points_.clear();
  image_points_.clear();
  num_detections_ = 0;
}

void MonoDvsCalibration::startCalibration()
{
  calibration_running_ = true;
  if (num_detections_ > 0)
  {
    calibrate();
  }
}

void MonoDvsCalibration::saveCalibration()
{
  if (setCameraInfo())
    ROS_INFO("Calibration saved successfully.");
  else
    ROS_ERROR("Error while saving calibration");
}


void MonoDvsCalibration::addPattern(int id)
{
  publishAddedPattern(id, detected_points_left_or_single_pub_,
		  detected_points_left_or_single_pattern_pub_, transition_maps_[id].pattern,
		  transition_maps_[id].get_visualization_image());

  // compute and publish camera pose if camera is calibrated
  if (got_camera_info_)
  {
    cv::Mat rvec, tvec;
    cv::Mat cameraMatrix(3, 3, CV_64F);
    cv::Mat distCoeffs(1, 5, CV_64F);

    // convert to OpenCV
    for (int i = 0; i < 5; i++)
      distCoeffs.at<double>(i) = camera_info_external_.D[i];
    for (int i = 0; i < 9; i++)
      cameraMatrix.at<double>(i) = camera_info_external_.K[i];

    cv::solvePnP(world_pattern_, transition_maps_[id].pattern, cameraMatrix, distCoeffs, rvec, tvec);

    geometry_msgs::PoseStamped pose_msg;
    pose_msg.header.stamp = ros::Time::now();
    pose_msg.header.frame_id = "dvs";
    pose_msg.pose.position.x = tvec.at<double>(0);
    pose_msg.pose.position.y = tvec.at<double>(1);
    pose_msg.pose.position.z = tvec.at<double>(2);

    double angle = cv::norm(rvec);
    cv::normalize(rvec, rvec);
    pose_msg.pose.orientation.x = rvec.at<double>(0) * sin(angle/2.0);
    pose_msg.pose.orientation.y = rvec.at<double>(1) * sin(angle/2.0);
    pose_msg.pose.orientation.z = rvec.at<double>(2) * sin(angle/2.0);
    pose_msg.pose.orientation.w = cos(angle/2.0);

    camera_pose_pub_.publish(pose_msg);
  }
}

void MonoDvsCalibration::imageObjectPointsCallback(dvs_msgs::ImageObjectPoints msg)
{
  ROS_INFO("imageObjectPointsCallback");

  //convert message to point2f vector
  std::vector<cv::Point2f> left;

  cv::Point2f image_point;
  for (dvs_msgs::Point2f pp : msg.image_points) {
	  image_point.x = pp.x;
	  image_point.y = pp.y;
	  left.push_back(image_point);
  }

  // add detection
  image_points_.push_back(left);

  //convert message to point3f vector
  std::vector<cv::Point3f> object_points;

  cv::Point3f object_point;
  for (dvs_msgs::Point3f pp : msg.object_points) {
	  object_point.x = pp.x;
	  object_point.y = pp.y;
	  object_point.z = pp.z;
	  object_points.push_back(object_point);
  }

  // add object points only for left callback
  // are the same for the right callback
  object_points_.push_back(object_points);
  num_detections_++;

  //update publisher
  publishNumDetections();
}

void MonoDvsCalibration::cameraInfoCallback(const sensor_msgs::CameraInfo::ConstPtr& msg)
{
  got_camera_info_ = true;
  camera_info_external_ = *msg;
}

void MonoDvsCalibration::updateVisualization(int id)
{
  publishVisualizationImage(transition_maps_[id].get_visualization_image());
}

} // namespace
