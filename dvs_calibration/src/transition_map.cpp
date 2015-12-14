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

#include "dvs_calibration/transition_map.h"

namespace dvs_calibration {

TransitionMap::TransitionMap(const CalibrationParameters params)
  : params_(params), has_pattern_(false), sensor_width_(0), sensor_height_(0)
{
  reset_maps();
  last_reset_time_ = ros::Time::now();
}

int TransitionMap::max()
{
  int max = 0;
  for (int i = 0; i < sensor_width_; i++)
  {
    for (int j = 0; j < sensor_height_; j++)
    {
      if (transition_sum_map_[i][j] > max)
        max = transition_sum_map_[i][j];
    }
  }
  return max;
}

void TransitionMap::update(const dvs_msgs::EventArray::ConstPtr& msg)
{
  // initialize memory if needed
  if (sensor_width_ != msg->width || sensor_height_ != msg->height)
  {
    init_transition_maps(msg->width, msg->height);
  }

  // iterate over all events
  for (int i = 0; i < msg->events.size(); ++i)
  {
    if (msg->events[i].polarity == true)
    {
      last_off_map_[msg->events[i].x][msg->events[i].y] = msg->events[i].ts;
    }
    else
    {
      int delta_t_us = (msg->events[i].ts - last_off_map_[msg->events[i].x][msg->events[i].y]).toSec()*1e6;
      if (delta_t_us < params_.blinking_time_us + params_.blinking_time_tolerance_us && delta_t_us > params_.blinking_time_us - params_.blinking_time_tolerance_us) {
        const int x = msg->events[i].x;
        const int y = msg->events[i].y;

        // 3x3 Gaussian distribution
        transition_sum_map_[x][y] += 0.25;
        if(x > 0)              transition_sum_map_[x-1][y] += .125;
        if(x < 127)            transition_sum_map_[x+1][y] += .125;
        if(y > 0)              transition_sum_map_[x][y-1] += .125;
        if(y < 127)            transition_sum_map_[x][y+1] += .125;
        if(x > 0 && y > 0)     transition_sum_map_[x-1][y-1] += .0625;
        if(x < 127 && y > 0)   transition_sum_map_[x+1][y-1] += .0625;
        if(x < 127 && y < 127) transition_sum_map_[x+1][y+1] += .0625;
        if(x > 0 && y < 127)   transition_sum_map_[x-1][y+1] += .0625;

        // At borders, add to center pixel instead to keep sum at 1
        if(! x > 0)                transition_sum_map_[x][y] += .125;
        if(! x < 127)              transition_sum_map_[x][y] += .125;
        if(! y > 0)                transition_sum_map_[x][y] += .125;
        if(! y < 127)              transition_sum_map_[x][y] += .125;
        if(! x > 0 && ! y > 0)     transition_sum_map_[x][y] += .0625;
        if(! x < 127 && ! y > 0)   transition_sum_map_[x][y] += .0625;
        if(! x < 127 && ! y < 127) transition_sum_map_[x][y] += .0625;
        if(! x > 0 && ! y < 127)   transition_sum_map_[x][y] += .0625;

      }
    }
  }

}

cv::Mat TransitionMap::get_visualization_image()
{
  cv::Mat image = cv::Mat(sensor_height_, sensor_width_, CV_8UC3);
  image = cv::Scalar(255, 255, 255);
  int max_value = log(max() + 1);
  for (int i = 0; i < sensor_width_; i++)
  {
    for (int j = 0; j < sensor_height_; j++)
    {
      int value = 255.0 - (log((double)transition_sum_map_[i][j] + 1)) / ((double)max_value) * 255.0;
      if(value > 255) value = 255;
      if(value < 0) value = 0;

      if(transition_sum_map_[i][j] >= params_.minimum_transitions_threshold)
        image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 255-value, 0);
      else
        image.at<cv::Vec3b>(j, i) = cv::Vec3b(0, 0, 255-value);

    }
  }

  if (has_pattern())
  {
    cv::drawChessboardCorners(image, cv::Size(params_.dots_w, params_.dots_h), cv::Mat(pattern), true);
  }

  return image;
}

void TransitionMap::init_transition_maps(const int width, const int height)
{
  sensor_height_ = height;
  sensor_width_ = width;

  last_off_map_.clear();
  last_on_map_.clear();
  transition_sum_map_.clear();

  std::vector<ros::Time> col_ts;
  std::vector<double> col_sum;
  for (int y=0; y<width; y++)
  {
    col_ts.push_back(ros::Time());
    col_sum.push_back(0);
  }

  for (int x=0; x<width; x++)
  {
    last_off_map_.push_back(col_ts);
    last_on_map_.push_back(col_ts);
    transition_sum_map_.push_back(col_sum);
  }
}

void TransitionMap::find_pattern()
{
  std::list<PointWithWeight> points;

  for (int i = 0; i < sensor_width_; i++)
  {
    for (int j = 0; j < sensor_height_; j++)
    {
      if (transition_sum_map_[i][j] > params_.minimum_transitions_threshold)
      {
        PointWithWeight p;
        p.point = cv::Point(i, j);
        p.weight = (double) transition_sum_map_[i][j];
        points.push_back(p);
      }
    }
  }

  pattern = BoardDetection::findPattern(points, params_.dots_w, params_.dots_h, params_.minimum_led_mass);

  if (pattern.size() == params_.dots_w * params_.dots_h)
  {
    has_pattern_ = true;
  }
}

void TransitionMap::reset_maps()
{
  last_reset_time_ = ros::Time::now();
  for (int i = 0; i < sensor_width_; i++)
  {
    for (int j = 0; j < sensor_height_; j++)
    {
      last_on_map_[i][j] = ros::Time();
      last_off_map_[i][j] = ros::Time();
      transition_sum_map_[i][j] = 0;
    }
  }
  has_pattern_ = false;
}

} // namespace
