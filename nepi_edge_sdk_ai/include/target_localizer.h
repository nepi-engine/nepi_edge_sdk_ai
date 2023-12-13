/*
 * NEPI Dual-Use License
 * Project: nepi_edge_sdk_ai
 *
 * This license applies to any user of NEPI Engine software
 *
 * Copyright (C) 2023 Numurus, LLC <https://www.numurus.com>
 * see https://github.com/numurus-nepi/nepi_edge_sdk_ai
 *
 * This software is dual-licensed under the terms of either a NEPI software developer license
 * or a NEPI software commercial license.
 *
 * The terms of both the NEPI software developer and commercial licenses
 * can be found at: www.numurus.com/licensing-nepi-engine
 *
 * Redistributions in source code must retain this top-level comment block.
 * Plagiarizing this software to sidestep the license obligations is illegal.
 *
 * Contact Information:
 * ====================
 * - https://www.numurus.com/licensing-nepi-engine
 * - mailto:nepi@numurus.com
 *
 */
#ifndef __TARGET_LOCALIZER_H
#define __TARGET_LOCALIZER_H

#include <unordered_map>

#include <image_transport/image_transport.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include "sdk_node.h"
#include "darknet_ros_msgs/BoundingBoxes.h"
#include "nepi_ros_interfaces/ClassifierSelection.h"
#include "nepi_ros_interfaces/TargetLocalization.h"

// Darknet helper for image decorations
#include "image.h"

namespace Numurus
{

struct TrackedTarget
{
  TrackedTarget(int32_t targ_id, ros::Time first_seen, const nepi_ros_interfaces::TargetLocalization &localized_target);

  int32_t type;
  int32_t id;
  ros::Time last_observed_time;
  double last_observed_range_m;
  double last_observed_azimuth_deg;
  double last_observed_elevation_deg;
};

class TargetLocalizer : public SDKNode
{
public:
  TargetLocalizer();
  virtual ~TargetLocalizer();

  // SDKNode overrides
  void run() override;

private:
  // SDKNode overrides
  void initPublishers() override;
  void retrieveParams() override;
  void initSubscribers() override;

  void localizeTargets(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& img_cam_info,
                       const sensor_msgs::ImageConstPtr& depth_img, const sensor_msgs::CameraInfoConstPtr& depth_cam_info,
                       const darknet_ros_msgs::BoundingBoxesConstPtr& boxes);

  void startClassifierCallback(const nepi_ros_interfaces::ClassifierSelectionConstPtr& classifier);
  void purgeLostTargets();

  const int IMG_SYNC_QUEUE_SIZE = 10;
  const double FIXED_CAMERA_SENSOR_WIDTH_MM = 0.008 * 672;
  const double FIXED_CAMERA_SENSOR_HEIGHT_MM = 0.008 * 376;

  SDKNode::NodeParam<std::string> classifier_full_namespace;
  SDKNode::NodeParam<std::string> depth_cam_full_namespace;
  SDKNode::NodeParam<std::string> data_out_frame_id;
  SDKNode::NodeParam<float> range_avg_subbox_ratio;
  SDKNode::NodeParam<bool> target_tracking_enabled;
  SDKNode::NodeParam<float> tracked_target_lost_period;

  std::string img_topic;
  std::string img_cam_info_topic;
  std::string depth_topic;
  std::string boxes_topic;

  bool subscribed_to_input_topics = false;

  bool needs_new_camera_calcs = true;
  double depth_cam_fov_x_deg = 0.0;
  double depth_cam_fov_y_deg = 0.0;

  uint32_t localization_scene_counter = 0;

  message_filters::Subscriber<sensor_msgs::Image> img_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> img_cam_info_sub;
  message_filters::Subscriber<sensor_msgs::Image> depth_img_sub;
  message_filters::Subscriber<sensor_msgs::CameraInfo> depth_cam_info_sub;
  message_filters::Subscriber<darknet_ros_msgs::BoundingBoxes> bounding_boxes_sub;

  typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                          sensor_msgs::Image, sensor_msgs::CameraInfo,
                                                          darknet_ros_msgs::BoundingBoxes> ApproxSyncPolicy;
  message_filters::Synchronizer<ApproxSyncPolicy>* approx_data_sync = nullptr;

  ros::Publisher targ_localization_scene_pub;
  ros::Publisher localization_img_pub;

  // Darknet helper for localization image decoration
  image **label_alphabet = nullptr;

  nepi_ros_interfaces::TargetLocalization INVALID_TARGET_LOCALIZATION;

  // Tracking stuff
  std::unordered_map<int32_t, std::vector<TrackedTarget>> tracked_targets_by_type;
  int32_t next_tracked_targ_id = 0;
};

} // namespace Numurus
#endif
