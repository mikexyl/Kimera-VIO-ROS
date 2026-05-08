/**
 * @file   RosVisualizer.h
 * @brief  Equivalent Kimera Visualizer but in ROS. Publishes 3D data to ROS.
 * @author Antoni Rosinol
 */

#pragma once

#include <atomic>
#include <cstdint>
#include <mutex>
#include <string>
#include <vector>

#define PCL_NO_PRECOMPILE  // Define this before you include any PCL headers
                           // to include the templated algorithms
#include <glog/logging.h>
#include <kimera-vio/backend/VioBackend-definitions.h>
#include <kimera-vio/frontend/FrontendOutputPacketBase.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/mesh/Mesher-definitions.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <opencv2/opencv.hpp>

#include "kimera_vio_ros/RosPublishers.h"
#include "kimera_vio_ros/RosRerunVisualizer.h"

namespace VIO {

/**
 * @brief The PointNormalUV struct holds mesh vertex data.
 */
struct PointNormalUV {
  PCL_ADD_POINT4D;
  PCL_ADD_NORMAL4D;
  float u;  // Texture coordinates.
  float v;
  EIGEN_MAKE_ALIGNED_OPERATOR_NEW
} EIGEN_ALIGN16;

class RosVisualizer : public Visualizer3D {
 public:
  KIMERA_POINTER_TYPEDEFS(RosVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(RosVisualizer);

 public:
  RosVisualizer(const VioParams& vio_params);
  virtual ~RosVisualizer() = default;

 public:
  /**
   * @brief spinOnce
   * Spins the display once to render the visualizer output.
   * @param viz_input
   */
  VisualizerOutput::UniquePtr spinOnce(
      const VisualizerInput& viz_input) override;

 protected:
  // Publish VIO outputs.
  virtual void publishBackendOutput(const BackendOutput::ConstPtr& output);

  virtual void publishFrontendOutput(
      const FrontendOutputPacketBase::ConstPtr& output) const;

  virtual void publishMesherOutput(const MesherOutput::ConstPtr& output) const;

 private:
  bool lookupExternalPoseFrameTransform(gtsam::Pose3* base_T_external,
                                        gtsam::Pose3* external_T_base);

 private:
  void publishTimeHorizonPointCloud(
      const BackendOutput::ConstPtr& output) const;

  void publishPerFrameMesh3D(const MesherOutput::ConstPtr& output) const;

  // void publishTimeHorizonMesh3D(const MesherOutput::ConstPtr& output) const;

  void publishState(const BackendOutput::ConstPtr& output) const;

  void publishFrontendStats(
      const FrontendOutputPacketBase::ConstPtr& output) const;

  void publishResiliency(
      const FrontendOutputPacketBase::ConstPtr& frontend_output,
      const BackendOutput::ConstPtr& backend_output) const;

  void publishImuBias(const BackendOutput::ConstPtr& output) const;

  void publishTf(const BackendOutput::ConstPtr& output);

  void publishRerunBackendOutput(const BackendOutput::ConstPtr& output);

  void publishDebugImage(const Timestamp& timestamp,
                         const cv::Mat& debug_image) const;

 private:
  // ROS handles
  ros::NodeHandle nh_;
  ros::NodeHandle nh_private_;

  // ROS publishers
  ros::Publisher pointcloud_pub_;
  //! Published 3d mesh per frame (not time horizon of opitimization!)
  ros::Publisher mesh_3d_frame_pub_;
  ros::Publisher odometry_pub_;
  ros::Publisher resiliency_pub_;
  ros::Publisher frontend_stats_pub_;
  ros::Publisher imu_bias_pub_;

  //! Define tf broadcaster for world to base_link (IMU) and to map (PGO).
  tf::TransformBroadcaster tf_broadcaster_;
  tf::TransformListener tf_listener_;

 private:
  //! Define frame ids for odometry message
  std::string odom_frame_id_;
  std::string base_link_frame_id_;
  std::string map_frame_id_;
  std::string cbs_external_pose_frame_id_;

  cv::Size image_size_;

  //! Define image publishers manager
  std::unique_ptr<ImagePublishers> image_publishers_;

  bool cbs_belief_bridge_enable_ = true;
  std::unique_ptr<RosRerunVisualizer> rerun_visualizer_;
  std::vector<gtsam::Pose3> rerun_trajectory_;
  int64_t rerun_last_kf_id_ = -1;
  bool rerun_factor_graph_enable_ = true;
  bool rerun_world_alignment_enable_ = true;
  std::atomic<size_t> rerun_cbs_beliefs_published_per_update_{0u};

  struct RerunTimedPose {
    uint64_t timestamp_ns = 0u;
    gtsam::Pose3 pose;
  };

  void updateRerunPoseHistory(uint64_t timestamp_ns,
                              const gtsam::Pose3& pose);
  bool maybeInitializeRerunWorldAlignment(
      uint64_t peer_timestamp_ns,
      const gtsam::Pose3& liorf_world_pose_body,
      const std::string& key_token);

  std::mutex rerun_world_alignment_mutex_;
  std::vector<RerunTimedPose> rerun_kimera_pose_history_;
  bool rerun_world_alignment_initialized_ = false;
  gtsam::Pose3 rerun_liorf_T_kimera_world_;
  double rerun_world_alignment_timestamp_delta_ms_ = 0.0;

  // Typedefs
  typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloudXYZRGB;
};

}  // namespace VIO

POINT_CLOUD_REGISTER_POINT_STRUCT(
    VIO::PointNormalUV,
    (float, x, x)(float, y, y)(float, x, z)(float, normal_x, normal_x)(
        float,
        normal_y,
        normal_y)(float, normal_z, normal_z)(float, u, u)(float, v, v))
