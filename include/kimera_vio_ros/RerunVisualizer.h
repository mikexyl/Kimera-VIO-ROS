#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <kimera-vio/visualizer/Visualizer3D.h>

namespace VIO {

class RerunVisualizer : public VIO::Visualizer3D, aria::viz::VisualizerRerun {
 public:
  RerunVisualizer(std::string base_link_frame_id = "baselink",
                  std::string odom_frame_id = "odom",
                  std::string map_frame_id = "map")
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        aria::viz::VisualizerRerun(aria::viz::VisualizerRerun::Params(
            "kimera_vio",
            std::nullopt,
            "rerun+http://172.17.0.1:9876/proxy")),
        baselink_(base_link_frame_id),
        map_(map_frame_id),
        odom_(odom_frame_id) {}

  virtual ~RerunVisualizer() = default;

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    this->setTimeNSec(input.timestamp_);
    this->drawTf(map_ / odom_ / baselink_,
                 input.backend_output_->W_State_Blkf_.pose_,
                 0.3,
                 false);

    cv::Mat tracking_image_clone =
        input.frontend_output_->getTrackingImage()->clone();

    if (not input.frontend_output_->getTrackingImage()->empty()) {
      this->drawImage(map_ / odom_ / baselink_ / "tracking_image",
                      tracking_image_clone,
                      true);
    }

    visualizeGraphInSmoother(input);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void visualizeGraphInSmoother(const VIO::VisualizerInput& input) {
    // No-op for RerunVisualizer.
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

 private:
  std::filesystem::path baselink_;
  std::filesystem::path map_;
  std::filesystem::path odom_;
};

}  // namespace VIO