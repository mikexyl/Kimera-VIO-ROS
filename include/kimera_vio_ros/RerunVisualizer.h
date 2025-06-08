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
        aria::viz::VisualizerRerun(
            aria::viz::VisualizerRerun::Params("kimera_vio")),
        base_link_frame_id_(base_link_frame_id),
        map_frame_id_(map_frame_id),
        odom_frame_id_(odom_frame_id) {}

  virtual ~RerunVisualizer() = default;

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    this->setTimeNSec(input.timestamp_);
    this->drawTf(odom_frame_id_ / map_frame_id_ / base_link_frame_id_,
                 input.backend_output_->W_State_Blkf_.pose_,
                 2.,
                 false);

    cv::Mat tracking_image_clone =
        input.frontend_output_->getTrackingImage()->clone();

    if (not input.frontend_output_->getTrackingImage()->empty()) {
      this->drawImage(odom_frame_id_ / map_frame_id_ / base_link_frame_id_ /
                          "tracking_image",
                      tracking_image_clone,
                      true);
    }

    return std::make_unique<VIO::VisualizerOutput>();
  }

 private:
  std::filesystem::path base_link_frame_id_;
  std::filesystem::path map_frame_id_;
  std::filesystem::path odom_frame_id_;
};

}  // namespace VIO