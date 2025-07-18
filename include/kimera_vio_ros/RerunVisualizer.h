#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <gtsam/slam/dataset.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <kimera_vio_ros/LoopClosureVisualizer.h>
#include <spdlog/fmt/fmt.h>

namespace VIO {

class RerunVisualizer : public Visualizer3D,
                        aria::viz::VisualizerRerun,
                        public LoopClosureVisualizer {
 public:
  RerunVisualizer(std::string base_link_frame_id = "baselink",
                  std::string odom_frame_id = "odom",
                  std::string map_frame_id = "map",
                  std::optional<std::string> recording_id = std::nullopt)
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        aria::viz::VisualizerRerun(aria::viz::VisualizerRerun::Params(
            "kimera_vio",
            recording_id,
            "rerun+http://172.17.0.1:9876/proxy")),
        baselink_(base_link_frame_id),
        map_(map_frame_id),
        odom_(odom_frame_id) {
    // draw the origin frame for visualization
    this->drawTf(map_, Pose3::Identity(), 0.3, true);
  }

  virtual ~RerunVisualizer() = default;

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    std::lock_guard<std::mutex> lock(rerun_mutex_);
    this->setTimeNSec(input.timestamp_);
    this->drawTf(map_ / odom_ / baselink_,
                 input.backend_output_->W_State_Blkf_.pose_,
                 0.3,
                 false);

    odom_traj_.push_back(input.backend_output_->W_State_Blkf_.pose_);
    this->drawTrajectory(map_ / odom_ / "trajectory",
                         odom_traj_,
                         aria::viz::ColorMap::kGreen,
                         0.5f,
                         false);

    cv::Mat tracking_image_clone =
        input.frontend_output_->getTrackingImage()->clone();

    if (not input.frontend_output_->getTrackingImage()->empty()) {
      this->drawImage(map_ / odom_ / baselink_ / "tracking_image",
                      tracking_image_clone,
                      true);
    }

    // visualizeGraphInSmoother(input);

    visualizeLandmarks(input);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void visualizeGraphInSmoother(const VIO::VisualizerInput& input) {
    this->drawPoints(map_ / odom_ / "smoother" / "values",
                     input.backend_output_->state_,
                     {aria::viz::ColorMap::kRed},
                     {2.},
                     {},
                     false);
    if (not input.backend_output_->debug_info_.graphBeforeOpt.empty()) {
      this->drawFactors(map_ / odom_ / "smoother" / "graph",
                        input.backend_output_->debug_info_.graphBeforeOpt,
                        input.backend_output_->state_,
                        aria::viz::ColorMap::kRed,
                        1.,
                        false,
                        true);
    }
  }

  void visualizeLandmarks(const VIO::VisualizerInput& input) {
    std::vector<Point3> landmarks;
    std::vector<long> ids;
    for (const auto& [id, lmk] :
         input.backend_output_->landmarks_with_id_map_) {
      landmarks.push_back(lmk);
      ids.push_back(id);
    }

    this->drawLandmarks(map_ / odom_ / "landmarks",
                        landmarks,
                        {},
                        {aria::viz::ColorMap::kBlue},
                        {0.1f},
                        {},
                        false);
  }

  void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output) override {
    std::lock_guard<std::mutex> lock(rerun_mutex_);

    this->setTimeNSec(lcd_output->timestamp_);
    this->drawTf(map_ / odom_, lcd_output->Map_Pose_Odom_, 0.3, false);

    CHECK(lcd_output);
    if (lcd_output->lcd_status_ == LCDStatus::LOOP_DETECTED) {
      std::string message =
          fmt::format("Loop closure detected: match id {}, recent id {} ",
                      lcd_output->id_match_,
                      lcd_output->id_recent_);
      this->rec()->log(
          "lcd_log",
          rerun::TextLog(message).with_level(rerun::TextLogLevel::Info));

      // save nfg to file
      gtsam::writeG2o(lcd_output->nfg_,
                      lcd_output->states_,
                      fmt::format("lcd_{}.g2o", lcd_output->timestamp_));

    } else {
      std::string message =
          fmt::format("No loop closure detected: status {}",
                      LoopResult::asString(lcd_output->lcd_status_));
      this->rec()->log(
          "lcd_log",
          rerun::TextLog(message).with_level(rerun::TextLogLevel::Warning));
    }

    auto opt_traj = lcd_output->states_;
    if (not opt_traj.empty()) {
      this->drawFactors(map_ / "pose_graph",
                        lcd_output->nfg_,
                        lcd_output->states_,
                        {aria::viz::ColorMap::kBlue},
                        0.5f,
                        false);
    }
  }

 private:
  std::filesystem::path baselink_;
  std::filesystem::path map_;
  std::filesystem::path odom_;

  std::vector<Pose3> odom_traj_{};

  std::mutex rerun_mutex_;
};

}  // namespace VIO