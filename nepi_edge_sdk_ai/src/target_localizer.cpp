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
#include <math.h>
#include <unordered_set>

#include "opencv2/highgui.hpp"
#include "cv_bridge/cv_bridge.h"
#include "opencv2/calib3d/calib3d.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include "target_localizer.h"
#include "sdk_utils.h"

#include "nepi_ros_interfaces/TargetLocalizationScene.h"
#include "nepi_ros_interfaces/ImageClassifierStatusQuery.h"

#define NODE_NAME "target_localizer"

#define MAX_SYNC_INTERVAL_DURATION    3.0

static const boost::array<float,9> IDENTITY_3_BY_3 = {{1.0,0.0,0.0,0.0,1.0,0.0,0.0,0.0,1.0}};
static const boost::array<float,9> ZERO_3_BY_3 = {{0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0,0.0}};

static image **load_alphabet_with_file(const std::string &datafile) {
  int i, j;
  const int nsize = 8;
  image **alphabets = (image**)calloc(nsize, sizeof(image));
  std::string files = datafile + "/labels/";

  for(j = 0; j < nsize; ++j){
    alphabets[j] = (image*)calloc(128, sizeof(image));
    for(i = 32; i < 127; ++i){
      std::string buff = files + std::to_string(i) + "_" + std::to_string(j) + ".png";
      alphabets[j][i] = load_image_color(const_cast<char*>(buff.c_str()), 0, 0);
    }
  }
  return alphabets;
}

namespace Numurus
{
TrackedTarget::TrackedTarget(int32_t targ_id, ros::Time first_seen, const nepi_ros_interfaces::TargetLocalization &localized_target) :
  type{localized_target.type},
  id{targ_id},
  last_observed_time{first_seen},
  last_observed_range_m{localized_target.range_m},
  last_observed_azimuth_deg{localized_target.azimuth_deg},
  last_observed_elevation_deg{localized_target.elevation_deg}
{
}

TargetLocalizer::TargetLocalizer():
  classifier_full_namespace{"classifier_full_namespace", "/nepi/s2x/classifier", this},
  depth_cam_full_namespace{"depth_cam_full_namespace", "/nepi/s2x/sensor_3dx/stereo_cam_driver/depth", this},
  data_out_frame_id{"data_out_frame_id", "nepi_center_frame", this},
  range_avg_subbox_ratio{"range_avg_subbox_ratio", 0.5f, this},
  target_tracking_enabled{"target_tracking_enabled", true, this},
  tracked_target_lost_period{"tracked_target_lost_period", 10.0f, this}
{
  // Load the alphabet for localizer image labels
  #ifdef DARKNET_FILE_PATH
  std::string darknet_data_path = DARKNET_FILE_PATH;
  darknet_data_path += "/data";
  label_alphabet = load_alphabet_with_file(darknet_data_path);
  #endif

  // Initialize the invalid target sentinel
  INVALID_TARGET_LOCALIZATION.type = INVALID_TARGET_LOCALIZATION.INT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.name = "Unset";
  INVALID_TARGET_LOCALIZATION.type_confidence = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.tracking_id = INVALID_TARGET_LOCALIZATION.INT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.tracking_confidence = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.range_m = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.azimuth_deg = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.elevation_deg = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.position_covariance = {{INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,
                                                      INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,
                                                      INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET}};
  INVALID_TARGET_LOCALIZATION.box_height_m = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.box_width_m = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.box_rotation_deg = INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET;
  INVALID_TARGET_LOCALIZATION.box_covariance = {{INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,
                                                 INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,
                                                 INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET,INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET}};
}

TargetLocalizer::~TargetLocalizer()
{
}

void TargetLocalizer::initPublishers()
{
  // First call the base method
  SDKNode::initPublishers();

  targ_localization_scene_pub = n_priv.advertise<nepi_ros_interfaces::TargetLocalizationScene>("target_localization_scene", 3);
  localization_img_pub = n_priv.advertise<sensor_msgs::Image>("localization_image", 3);
}

void TargetLocalizer::retrieveParams()
{
  // First, call the base method to get SDKNode params
  SDKNode::retrieveParams();

  // Now do custom params
  classifier_full_namespace.retrieve();
  depth_cam_full_namespace.retrieve();
  data_out_frame_id.retrieve();
  range_avg_subbox_ratio.retrieve();
  target_tracking_enabled.retrieve();
  tracked_target_lost_period.retrieve();

  // Always finish by warning about any unretrieved parameters
  warnUnretrievedParams();
}

void TargetLocalizer::initSubscribers()
{
  // First, call the base method
  SDKNode::initSubscribers();

  // Compute the start_classifier topic name
  const std::string classifier_namespace = classifier_full_namespace;
  const std::string start_classifier_topic = classifier_namespace.substr(0, classifier_namespace.find_last_of("/")) +
                                             "/start_classifier";
  subscribers.push_back(n.subscribe(start_classifier_topic, 10, &TargetLocalizer::startClassifierCallback, this));

  // Set up the synchronizer. We dynamically subscribe and unsubscribe from the input data topics, however
  approx_data_sync = new message_filters::Synchronizer<ApproxSyncPolicy>(ApproxSyncPolicy(IMG_SYNC_QUEUE_SIZE),
                                                                         img_sub, img_cam_info_sub,
                                                                         depth_img_sub, depth_cam_info_sub,
                                                                         bounding_boxes_sub);

  // Give a large max interval duration to reduce the chance that message sets are rejected due to timing issues
  // TODO: Maybe this should be configurable
  approx_data_sync->setMaxIntervalDuration(ros::Duration(MAX_SYNC_INTERVAL_DURATION));
  // TODO: Maybe need to define this through a custom policy as illustrated in this helpful link:
  // https://answers.ros.org/question/284758/roscpp-message_filters-approximatetime-api-questions/
  // TODO: Maybe need to change the age penalty to get the algorithm to run faster via
  // approx_data_sync->setAgePenalty(double age_penalty)

  // Register the approx-sync img callback
  approx_data_sync->registerCallback(boost::bind(&TargetLocalizer::localizeTargets, this, _1, _2, _3, _4, _5));
}

void TargetLocalizer::run()
{
  // Do init() if it hasn't already been done
  if (false == _initialized)
  {
  	init();
  }

  // Check if classifier is already running and if so, tap into its topics
  const std::string classifier_query_service = getPublicNamespace() + "/img_classifier_status_query";
  const bool classifier_exists = ros::service::waitForService(classifier_query_service, 5000); // 5 second timeout
  if (false == classifier_exists)
  {
    ROS_WARN("%s: classifer node does not appear to be launched yet (no service: %s)", NODE_NAME, classifier_query_service.c_str());
  }
  else
  {
    //ros::ServiceClient classifier_status_client = n.serviceClient<nepi_ros_interfaces::ImageClassifierStatusQuery>(classifier_query_service);
    nepi_ros_interfaces::ImageClassifierStatusQuery classifier_status_query;
    if (false == ros::service::call(classifier_query_service, classifier_status_query))
    {
      ROS_WARN("%s: classifier node failed to respond to %s", NODE_NAME, classifier_query_service.c_str());
    }
    else
    {
      if (classifier_status_query.response.classifier_state == classifier_status_query.response.CLASSIFIER_STATE_RUNNING)
      {
        // Set the img_topic
        img_topic = classifier_status_query.response.selected_img_topic;
        img_cam_info_topic = img_topic.substr(0, img_topic.find_last_of("/")) + "/camera_info";

        ROS_INFO("%s: classifier already running at startup with image topic %s", NODE_NAME, img_topic.c_str());

        needs_new_camera_calcs = true;
      }
    }

  }

  current_rate_hz = 100; // Spin fast
  ros::Time last_target_lost_check_time = ros::Time::now();

  // Spin at the current rate
  while (ros::ok())
  {
    ros::Rate r(current_rate_hz);

    const bool publish_targ_localization_scene = (targ_localization_scene_pub.getNumSubscribers() > 0);
    const bool publish_localization_img = (localization_img_pub.getNumSubscribers() > 0);
    // TODO data save i/f and any other publications
    const bool wants_data = publish_targ_localization_scene | publish_localization_img;

    if (true == wants_data)
    {
      // Check if we need to (re)subscribe to input topics
      if (false == subscribed_to_input_topics)
      {
        ROS_INFO("%s: subscribing to data topics because new output data subscriber(s) detected", NODE_NAME);

        // img_topic is defined when the start_classifier callback
        img_sub.subscribe(n, img_topic, 1);
        img_cam_info_sub.subscribe(n, img_cam_info_topic, 1);

        const std::string depth_cam_namespace = depth_cam_full_namespace;
        depth_topic = depth_cam_namespace + "/depth_registered";
        depth_img_sub.subscribe(n, depth_topic, 1);
        depth_cam_info_sub.subscribe(n, depth_cam_namespace + "/camera_info", 1);

        const std::string classifier_namespace = classifier_full_namespace;
        const std::string boxes_topic = classifier_namespace + "/bounding_boxes";
        bounding_boxes_sub.subscribe(n, boxes_topic, 1);

        subscribed_to_input_topics = true;
      }
    }
    else if (true == subscribed_to_input_topics)
    {
      img_sub.unsubscribe();
      img_cam_info_sub.unsubscribe();
      depth_img_sub.unsubscribe();
      depth_cam_info_sub.unsubscribe();
      bounding_boxes_sub.unsubscribe();
      subscribed_to_input_topics = false;
    }

    // Check if it is time to look for lost tracked targets
    const float check_target_lost_period = tracked_target_lost_period / 10.0f;
    const ros::Time now = ros::Time::now();
    if ((now - last_target_lost_check_time).toSec() >= check_target_lost_period)
    {
      purgeLostTargets();
      last_target_lost_check_time = now;
    }

    ros::spinOnce();
    r.sleep();
  }
}

void TargetLocalizer::localizeTargets(const sensor_msgs::ImageConstPtr& img, const sensor_msgs::CameraInfoConstPtr& img_cam_info,
                                      const sensor_msgs::ImageConstPtr& depth_img, const sensor_msgs::CameraInfoConstPtr& depth_cam_info,
                                      const darknet_ros_msgs::BoundingBoxesConstPtr& boxes)
{
  //ROS_WARN("Debug: Into localizeTargets");

  // First, ensure the input images have equal size -- necessary for calculations to follow
  if ((img->height != depth_img->height) || (img->width != depth_img->width))
  {
    ROS_WARN_THROTTLE(3.0, "%s: Can't operate on mismatched image and depth map sizes... %s and %s incompatible", NODE_NAME, img_topic.c_str(), depth_topic.c_str());
    return;
  }

  /* The message filter is already doing a pretty good job of avoiding too much latency between depth and img
  const double img_depth_delta_t = fabs((img->header.stamp - depth_img->header.stamp).toSec());
  if (img_depth_delta_t > 1.0f)
  {
    ROS_WARN_THROTTLE(3.0, "%s: Too much latency between image and depth map... not proceeding", NODE_NAME);
    return;
  }
  */

  nepi_ros_interfaces::TargetLocalizationScene localization_scene_msg;

  // Set header fields for this message
  localization_scene_msg.header.seq = ++localization_scene_counter;
  localization_scene_msg.header.stamp = ros::Time::now();
  localization_scene_msg.header.frame_id = data_out_frame_id;

  // Boxes metadata
  localization_scene_msg.input_boxes_topic = boxes_topic;
  localization_scene_msg.input_boxes_header = boxes->header;

  // Depth Metadata
  localization_scene_msg.input_depth_topic = depth_topic;
  localization_scene_msg.input_depth_header = depth_img->header;

  // Img Metadata
  localization_scene_msg.input_img_topic = img_topic;
  localization_scene_msg.input_img_header = img->header;

  if (true == needs_new_camera_calcs)
  {
    cv::Mat K(3, 3, CV_64F, (void*)(depth_cam_info->K.data()));
    cv::Size image_size(depth_cam_info->width, depth_cam_info->height);

    double focal_length;
    cv::Point2d principal_point;
    double aspect_ratio;

    cv::calibrationMatrixValues(K, image_size, FIXED_CAMERA_SENSOR_WIDTH_MM, FIXED_CAMERA_SENSOR_HEIGHT_MM,
                               depth_cam_fov_x_deg, depth_cam_fov_y_deg, focal_length, principal_point, aspect_ratio);

    ROS_INFO("Computed new FOV for depth camera: [%.03f,%.03f] deg", depth_cam_fov_x_deg, depth_cam_fov_y_deg);

    needs_new_camera_calcs = false;
  }

  cv_bridge::CvImageConstPtr depth_img_cv = cv_bridge::toCvShare(depth_img);

  // If anyone is subscribing to the localization image, need to setup for the overlays
  bool needs_localization_img_published = false;
  cv_bridge::CvImagePtr localization_img_cv = nullptr;
  //const int localization_box_thickness = (int)std::ceil((img_cam_info->width * 0.0025f)); // .25% of width;
  if (localization_img_pub.getNumSubscribers() > 0)
  {
    needs_localization_img_published = true;
    localization_img_cv = cv_bridge::toCvCopy(img);
  }

  std::vector<std::array<int64_t,4>> target_boxes;

  // Now localize each bounding box and target centroid
  for (const auto& box : boxes->bounding_boxes)
  {
    // Initialize all fields to UNSET, then individually set them below
    nepi_ros_interfaces::TargetLocalization target_loc = INVALID_TARGET_LOCALIZATION;

    // A few of the fields are straight copies of the input bounding box
    target_loc.type = box.id;
    target_loc.name = box.Class;
    target_loc.type_confidence = box.probability;

    const double xmin_percent = (double)box.xmin / img_cam_info->width;
    const double xmax_percent = (double)box.xmax / img_cam_info->width;
    const double ymin_percent = (double)box.ymin / img_cam_info->height;
    const double ymax_percent = (double)box.ymax / img_cam_info->height;

    // Now define the sub-box over which we will do range calculations. It is based on the
    // configurable sub-box ratio applied to each dimensions separately
    const float subbox_ratio = range_avg_subbox_ratio;

    const size_t depth_box_xmin = (size_t)(std::floor(depth_cam_info->width * xmin_percent));
    const size_t depth_box_xmax = (size_t)(std::floor(depth_cam_info->width * xmax_percent));
    const size_t depth_box_halfwidth = (depth_box_xmax - depth_box_xmin) / 2;
    const size_t range_averaging_window_x_exclusion_count = (1.0f - subbox_ratio)*depth_box_halfwidth;
    const size_t range_averaging_window_xmin = depth_box_xmin + range_averaging_window_x_exclusion_count;
    const size_t range_averaging_window_xmax = depth_box_xmax - range_averaging_window_x_exclusion_count;

    const size_t depth_box_ymin = (size_t)(std::floor(depth_cam_info->height * ymin_percent));
    const size_t depth_box_ymax = (size_t)(std::floor(depth_cam_info->height * ymax_percent));
    const size_t depth_box_halfheight = (depth_box_ymax - depth_box_ymin) / 2;
    const size_t range_averaging_window_y_exclusion_count = (1.0f - subbox_ratio)*depth_box_halfheight;
    const size_t range_averaging_window_ymin = depth_box_ymin + range_averaging_window_y_exclusion_count;
    const size_t range_averaging_window_ymax = depth_box_ymax - range_averaging_window_y_exclusion_count;

    RunningStat range_stat_calculator;

    for (int i = range_averaging_window_xmin; i < range_averaging_window_xmax; ++i)
    {
      for (int j = range_averaging_window_ymin; j < range_averaging_window_ymax; ++j)
      {
        //const float range_at_pixel_m = depth_img_cv->image.at<float>(i, j);
        const float range_at_pixel_m = depth_img_cv->image.at<float>(j, i);
        if ((false == isfinite(range_at_pixel_m)) || (range_at_pixel_m < 0.0f) || (range_at_pixel_m > 1000.0f)) // skip non-numerical (NAN, +-INF) and impossible range pixels
        {
          continue;
        }
        // Calculator takes care of all calculations on an iterative basis
        range_stat_calculator.push(range_at_pixel_m);
      }
    }
    // Gather outputs from the stat calculator
    const size_t valid_pixel_count = range_stat_calculator.getCount();
    const double max_range_in_target_box = range_stat_calculator.max();
    const double min_range_in_target_box = range_stat_calculator.min();
    const double range_at_target_m = range_stat_calculator.mean();
    const double range_variance = range_stat_calculator.variance();
    if (valid_pixel_count == 0)
    {
      ROS_WARN_THROTTLE(2.0f, "Target \"%s\": invalid range computed\n", box.Class.c_str());
      // Will leave all localization fields in the UNSET state
    }
    else
    {
      target_loc.range_m = range_at_target_m;

      // Azimuth is measure positive clockwise according to standard conventions
      const double xmin_deg = (xmin_percent - 0.5) * depth_cam_fov_x_deg;
      const double xmax_deg = (xmax_percent - 0.5) * depth_cam_fov_x_deg;
      const double xcenter_deg = (xmax_deg + xmin_deg) / 2.0;
      target_loc.azimuth_deg = xcenter_deg;

      // Images count columns from top to bottom, but positive elevation should be up, so notions of min/max switch between pixels and degrees
      const double ymin_deg = (0.5 - ymax_percent) * depth_cam_fov_y_deg;
      const double ymax_deg = (0.5 - ymin_percent) * depth_cam_fov_y_deg;
      const double ycenter_deg = (ymax_deg + ymin_deg) / 2.0;
      target_loc.elevation_deg = ycenter_deg;

      // Debugging
      /*
      const size_t averaging_window_pix_count = (range_averaging_window_xmax - range_averaging_window_xmin) *
                                           (range_averaging_window_ymax - range_averaging_window_ymin);
      ROS_WARN_THROTTLE(2.0f, "Debug: %s: box = [%.2f,%.2f]x[%.2f,%.2f], max_range=%0.3f, min_range=%0.3f, avg_range=%0.3f, variance=%0.3f, valid_count=%zu/%zu",
                        box.Class.c_str(), xmin_deg, xmax_deg, ymin_deg, ymax_deg, max_range_in_target_box, min_range_in_target_box,
                        range_at_target_m, range_variance, valid_pixel_count, averaging_window_pix_count);
      */
      // End Debugging

      // TODO: Figure out how to calculate some kind of real variance/covariance here for azimuth and elevation
      // Maybe base it on the size of the box?
      target_loc.position_covariance[0] = range_variance;

      const double xmin_m = range_at_target_m * sin(DEG_TO_RAD*xmin_deg);
      const double xmax_m = range_at_target_m * sin(DEG_TO_RAD*xmax_deg);
      target_loc.box_width_m = xmax_m - xmin_m;

      const double ymin_m = range_at_target_m * sin(DEG_TO_RAD*ymin_deg);
      const double ymax_m = range_at_target_m * sin(DEG_TO_RAD*ymax_deg);
      target_loc.box_height_m = ymax_m - ymin_m;

      // TODO: Box rotation??? For now we just leave it as UNSET

      // TODO: Figure out how to calculate some kind of real covariance here. For now we just leave it as unset
    }

    // Now, add this target to the list
    localization_scene_msg.targets.push_back(target_loc);
    // And keep track of its box boundaries in the input image
    std::array<int64_t, 4> target_box = {box.xmin, box.ymin, box.xmax, box.ymax};
    target_boxes.push_back(target_box);
  }

  std::unordered_set<int32_t> ineligible_target_ids;
  const bool tracking_enabled = target_tracking_enabled;

  // Now iterate over all localized targets to add tracking info and draw image overlays if necessary
  for (size_t i = 0; i < localization_scene_msg.targets.size(); ++i)
  {
    auto& target_loc = localization_scene_msg.targets[i];
    const auto& box = target_boxes[i];
    int32_t target_tracking_id = -1; // Will update below if we identify the target or add it to the running list

    // Only do target tracking if we have valid positional data for this target.
    if ((true == tracking_enabled) &&
        (target_loc.range_m != INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET) &&
        (target_loc.elevation_deg != INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET) &&
        (target_loc.azimuth_deg != INVALID_TARGET_LOCALIZATION.FLOAT_FIELD_UNSET))
    {
      auto &tracked_targets = tracked_targets_by_type[target_loc.type]; // Could be new and empty vector or existing vector
      TrackedTarget *best_tracked_target = nullptr;
      double min_dist_m = 1e9;

      // Define some variables that lead to an easier-to-read distance equation
      const double r = target_loc.range_m;
      const double theta = DEG_TO_RAD*target_loc.elevation_deg;
      const double phi = DEG_TO_RAD*target_loc.azimuth_deg;

      for (TrackedTarget &tracked_target : tracked_targets)
      {
        // Check that this TrackedTarget's id hasn't already been assigned to some other target in the current scene
        if (ineligible_target_ids.find(tracked_target.id) != ineligible_target_ids.end())
        {
          continue;
        }
        // Compute the distance from the candidate target to this one -- basic spherical coordinate distance formula
        // TODO: Predictive position based on IMU inputs? Inertial frame of reference rather than camera frame?
        // Define some variables that lead to an easier-to-read distance equation
        const double r_prime = tracked_target.last_observed_range_m;
        const double theta_prime = DEG_TO_RAD*tracked_target.last_observed_elevation_deg;
        const double phi_prime = DEG_TO_RAD*tracked_target.last_observed_azimuth_deg;

        const double distance_m = sqrt((r*r) + (r_prime*r_prime) -
                                       (2*r*r_prime*((sin(theta)*sin(theta_prime)*cos(phi-phi_prime)) + (cos(theta)*cos(theta_prime)))));
        //ROS_WARN("Debug: %s - checking target type %d (tracking id = %d) at distance %.2fm", target_loc.name.c_str(), tracked_target.type, tracked_target.id, distance_m);
        // TODO: Other filtering? Max allowed distance? Max velocity (distance_m/delta_time_m)?
        if (distance_m < min_dist_m)
        {
          min_dist_m = distance_m;
          best_tracked_target = &tracked_target;
        }
      }

      // If no valid candidates were identified, just push this as a new target... note we are guaranteed from checks above
      // that this new target has a valid position, so we can assume validity of the position when this TrackedTarget is checked
      // in the future. That is important!
      if (best_tracked_target == nullptr)
      {
        //ROS_WARN("Debug: %s - found no tracking match. Setting new ID %d", target_loc.name.c_str(), next_tracked_targ_id);
        target_tracking_id = next_tracked_targ_id++;
        target_loc.tracking_id = target_tracking_id;
        target_loc.tracking_confidence = 0.5; // Just hard-code an initial value
        tracked_targets.push_back(TrackedTarget(target_tracking_id, localization_scene_msg.header.stamp, target_loc));
      }
      else // Otherwise, update the selected TrackedTarget
      {
        target_tracking_id = best_tracked_target->id;
        target_loc.tracking_id = target_tracking_id;
        // TODO: More sophisticated tracking confidence. For now it is a very simple saturating distance-based value
        target_loc.tracking_confidence = (min_dist_m < 0.9)? 1.0 - min_dist_m : 0.1;

        best_tracked_target->last_observed_time = localization_scene_msg.header.stamp;
        best_tracked_target->last_observed_range_m = target_loc.range_m;
        best_tracked_target->last_observed_azimuth_deg = target_loc.azimuth_deg;
        best_tracked_target->last_observed_elevation_deg = target_loc.elevation_deg;
      }

      // Whatever the case, mark this target id as ineligible for other targets in the current scene
      ineligible_target_ids.insert(target_tracking_id);
    }

    // Do image overlays if necessary
    if (true == needs_localization_img_published)
    {
      // Use darknet tools to create the box and label
      image darknet_helper_img = mat_to_image(localization_img_cv->image);
      const int box_thickness = darknet_helper_img.h * .006;
      const int color_classes = 40;
      const int color_offset = target_loc.type % color_classes;

      float red = get_color(2,color_offset,color_classes);
      float green = get_color(1,color_offset,color_offset);
      float blue = get_color(0,color_offset,color_offset);
      float rgb[3] = {red,green,blue};

      draw_box_width(darknet_helper_img, box[0], box[1], box[2], box[3], box_thickness, red, green, blue);

      if (label_alphabet) {
          char target_label[128];
          // Format of the label depends on whether tracking is enabled and if so, whether a valid tracked target was found
          if (tracking_enabled)
          {
            if (target_tracking_id != -1)
            {
              snprintf(target_label, 128, "%d:%s [r:%.02fm, az:%.01f*, el:%.01f*]",
                       target_tracking_id, target_loc.name.c_str(), target_loc.range_m, target_loc.azimuth_deg, target_loc.elevation_deg);
            }
            else
            {
              snprintf(target_label, 128, "?:%s [r:%.02fm, az:%.01f*, el:%.01f*]",
                       target_loc.name.c_str(), target_loc.range_m, target_loc.azimuth_deg, target_loc.elevation_deg);
            }
          }
          else // tracking disabled
          {
            snprintf(target_label, 128, "%s [r:%.02fm, az:%.01f*, el:%.01f*]",
                     target_loc.name.c_str(), target_loc.range_m, target_loc.azimuth_deg, target_loc.elevation_deg);
          }
          image label = get_label(label_alphabet, target_label, (darknet_helper_img.h*.03));

          int label_start_col = box[0];
          if (label_start_col + label.w > darknet_helper_img.w)
          {
            const int overlap = label.w - (box[2] - box[0]);
            label_start_col -= overlap;
            if (label_start_col < 0) label_start_col = 0;
          }

          int label_start_row = box[1] + box_thickness;
          /* Seems to be unnecessary
          if (label_start_row + label.h > darknet_helper_img.h)
          {
            label_start_row = box[3] - box_thickness;
            if (label_start_row < 0) label_start_row = 0;
          }
          */

          draw_label(darknet_helper_img, label_start_row, label_start_col, label, rgb);
          free_image(label);
      }

      localization_img_cv->image = image_to_mat(darknet_helper_img);
      free_image(darknet_helper_img);

      /* This is the original OpenCV-based box drawing... the text looked terrible, so used the darknet stuff above instead
      cv::Point upper_left(box.xmin, box.ymin);
      cv::Point lower_right(box.xmax, box.ymax);
      // TODO: Need an encoding-derived color so that other encodings will work here too -- this assumes bgra
      const cv::Scalar color(0,255,0,0); // Green
      const cv::Scalar text_color(0,0,0,0); // Black
      const int localization_box_thickness = 1;
      cv::rectangle(localization_img_cv->image, upper_left, lower_right, color, localization_box_thickness);

      int baseline = 0;
      const double localization_text_scale = 0.3;
      const cv::Size label_size = cv::getTextSize(target_label, cv::FONT_HERSHEY_SIMPLEX, localization_text_scale, localization_box_thickness, &baseline);
      cv::Point upper_left_text(box.xmin, box.ymin - label_size.height);
      cv::Point lower_right_text(box.xmin + label_size.width, box.ymin);
      cv::rectangle(localization_img_cv->image, upper_left_text, lower_right_text, color, cv::FILLED);
      cv::Point bottom_left_text(box.xmin, box.ymin);
      cv::putText(localization_img_cv->image, target_label, bottom_left_text, cv::FONT_HERSHEY_SIMPLEX, localization_text_scale,
                  text_color, localization_box_thickness, cv::LINE_AA);
      */
    }
  }

  if (targ_localization_scene_pub.getNumSubscribers() > 0)
  {
    targ_localization_scene_pub.publish(localization_scene_msg);
  }

  if (true == needs_localization_img_published)
  {
    localization_img_pub.publish(localization_img_cv->toImageMsg());
  }

  // TODO: Save data if necessary
}

void TargetLocalizer::startClassifierCallback(const nepi_ros_interfaces::ClassifierSelectionConstPtr& classifier)
{
  // Unsubscribe from all current topics -- will resubscribe in run method if necessary
  img_sub.unsubscribe();
  img_cam_info_sub.unsubscribe();
  depth_img_sub.unsubscribe();
  depth_cam_info_sub.unsubscribe();
  bounding_boxes_sub.unsubscribe();
  subscribed_to_input_topics = false;

  // Set the img_topic
  img_topic = classifier->img_topic;
  img_cam_info_topic = img_topic.substr(0, img_topic.find_last_of("/")) + "/camera_info";

  ROS_WARN("Debug: Start classifier yields topics %s and %s", img_topic.c_str(), img_cam_info_topic.c_str());

  // TODO: Could try to be more selective here about when we actually need to do the geometry calcs
  needs_new_camera_calcs = true;
}

void TargetLocalizer::purgeLostTargets()
{
  const ros::Time now = ros::Time::now();

  for (auto &tracked_targets : tracked_targets_by_type)
  {
    auto it = tracked_targets.second.begin();
    while (it != tracked_targets.second.end())
    {
      const double elapsed_time = (now - it->last_observed_time).toSec();
      const double lost_period = tracked_target_lost_period;
      if (elapsed_time > lost_period)
      {
        ROS_INFO("Purging lost tracked target %d -- no contact for %.02f secs", it->id, elapsed_time);
        it = tracked_targets.second.erase(it);
      }
      else
      {
        ++it;
      }
    }
  }
}

} // namespace Numurus

int main(int argc, char **argv)
{
  ros::init(argc, argv, NODE_NAME);

  // The class instance does the work
  Numurus::TargetLocalizer localizer;

  // Call init() before anything else to make sure the config params are properly initialized. They are needed in methods to follow
  localizer.init();

  ROS_INFO("Starting the %s Node", localizer.getUnqualifiedName().c_str());

  localizer.run();
  return 0;
}
