#pragma once

#include <aria_viz/visualizer_rerun.h>
#include <glog/logging.h>
#include <gtsam/slam/dataset.h>
#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>
#include <kimera-vio/visualizer/Visualizer3D.h>
#include <kimera_vio_ros/LoopClosureVisualizer.h>
#include <spdlog/fmt/fmt.h>

#include <chrono>
#include <future>

namespace VIO {

// Alias for the custom log handler signature
using GlogHandler = std::function<void(google::LogSeverity severity,
                                       const char* filename,
                                       int line,
                                       const char* message)>;

// Custom sink that forwards glog messages to the user-provided handler
class CustomLogSink : public google::LogSink {
 public:
  explicit CustomLogSink(GlogHandler handler) : handler_(std::move(handler)) {}

  void send(google::LogSeverity severity,
            const char* full_filename,
            const char* base_filename,
            int line,
            const struct tm* tm_time,
            const char* message,
            size_t message_len) override {
    // Build message string and forward to custom handler
    std::string msg(message, message_len);
    handler_(severity, base_filename, line, msg.c_str());
  }

 private:
  GlogHandler handler_;
};

// -----------------------------------------------------------------------------
// Redirect std::cout to glog at INFO level by installing a custom streambuf
// -----------------------------------------------------------------------------
class GlogStreamBuf : public std::streambuf {
 public:
  GlogStreamBuf() { setp(buffer_, buffer_ + sizeof(buffer_) - 1); }

 protected:
  int_type overflow(int_type ch) override {
    if (ch != traits_type::eof()) {
      *pptr() = static_cast<char>(ch);
      pbump(1);
    }
    if (ch == '\n' || pptr() >= epptr()) {
      flushBuffer();
    }
    return ch;
  }

  int sync() override {
    flushBuffer();
    return 0;
  }

 private:
  void flushBuffer() {
    std::ptrdiff_t len = pptr() - pbase();
    if (len <= 0) return;
    std::string msg(pbase(), len);
    LOG(INFO) << msg;
    pbump(-len);
  }

  char buffer_[1024];
};

enum class VisualizationProfile {
  Minimal = 0,  // Only pose and trajectory
  Standard,     // Add tracking images and landmarks
  Detailed,     // Add depth images, covariance, smoother states
  Debug         // Everything including graph visualization
};

class RerunVisualizer : public Visualizer3D, aria::viz::VisualizerRerun {
 public:
  struct Params {
    std::string base_link_frame_id = "baselink";
    std::string odom_frame_id = "odom";
    std::string map_frame_id = "map";
    std::string robot_name = "robot0";
    std::string gt_csv_file = "";
    std::optional<std::string> recording_id = std::nullopt;
    std::string result_dir = "rerun_results";
    VisualizationProfile profile = VisualizationProfile::Standard;
  };

  RerunVisualizer(const Params& params)
      : RerunVisualizer(params.base_link_frame_id,
                        params.odom_frame_id,
                        params.map_frame_id,
                        params.robot_name,
                        params.gt_csv_file,
                        params.recording_id,
                        params.result_dir,
                        params.profile) {}

  RerunVisualizer(std::string base_link_frame_id = "baselink",
                  std::string odom_frame_id = "odom",
                  std::string map_frame_id = "map",
                  std::string robot_name = "robot0",
                  std::string gt_csv_file = "",
                  std::optional<std::string> recording_id = std::nullopt,
                  std::string result_dir = "",
                  VisualizationProfile profile = VisualizationProfile::Standard)
      : VIO::Visualizer3D(VIO::VisualizationType::kNone),
        aria::viz::VisualizerRerun(aria::viz::VisualizerRerun::Params(
            "code-slam",
            recording_id,
            "rerun+http://172.17.0.1:9876/proxy")),
        baselink_(base_link_frame_id),
        map_("/vio/" + robot_name + "/" + map_frame_id),
        odom_(odom_frame_id),
        robot_name_(robot_name),
        result_dir_(result_dir),
        profile_(profile) {
    // draw the origin frame for visualization
    this->drawTf(map_, Pose3::Identity(), 0.3, true);

    if (not g_custom_sink) {
      AddGlogCustomSink([this](google::LogSeverity severity,
                               const char* filename,
                               int line,
                               const char* message) {
        logGlogMessages(
            "vio/" + robot_name_, severity, filename, line, message);
      });
      RedirectStdCoutToGlog();
    }

    auto landmark_color = aria::viz::ColorMap::kGray;
    landmark_color[3] = 50;

    if (not gt_csv_file.empty()) {
      gt_trajectory_ = loadTrajectoryMapFromCSV(gt_csv_file);
      std::vector<gtsam::Pose3> gt_traj;
      for (const auto& [key, pose] : gt_trajectory_) {
        gt_traj.push_back(pose);
      }
      this->drawTf(map_ / "gt", T_map_gt_, 1.0, false);
      this->drawTrajectory(
          map_ / "gt" / "trajectory", gt_traj, landmark_color, 1.f, false);
    }

    if (not result_dir_.empty()) {
      // check if folder exists
      if (not std::filesystem::exists(result_dir_)) {
        LOG(FATAL) << "Result directory does not exist: " << result_dir_;
      }
      LOG(INFO) << "RerunVisualizer result directory: " << result_dir_;
    } else {
      LOG(INFO) << "RerunVisualizer result disabled";
    }

    // Initialize timer for trajectory saving
    last_save_time_ = std::chrono::steady_clock::now();
    last_gt_draw_time_ = std::chrono::steady_clock::now();
  }

  virtual ~RerunVisualizer() = default;

  // Hold the active custom sink so it persists for the program lifetime.
  static std::unique_ptr<CustomLogSink> g_custom_sink;

  // Call this after google::InitGoogleLogging(), to attach your custom handler
  // in addition to glog's default sinks (stderr and/or log files).
  inline void AddGlogCustomSink(GlogHandler handler) {
    // Remove previous custom sink if installed
    if (g_custom_sink) {
      google::RemoveLogSink(g_custom_sink.get());
      g_custom_sink.reset();
    }

    // Create and install a new sink; default sinks remain active
    g_custom_sink = std::make_unique<CustomLogSink>(std::move(handler));
    google::AddLogSink(g_custom_sink.get());
  }

  // Preserve original buffer so we can restore cout
  static std::streambuf* g_original_cout_buf_;
  static GlogStreamBuf g_glog_streambuf_;

  // Call after InitGoogleLogging() to capture std::cout output
  inline void RedirectStdCoutToGlog() {
    if (!g_original_cout_buf_) {
      g_original_cout_buf_ = std::cout.rdbuf(&g_glog_streambuf_);
    }
  }

  // Restore original std::cout behavior
  inline void RestoreStdCout() {
    if (g_original_cout_buf_) {
      std::cout.rdbuf(g_original_cout_buf_);
      g_original_cout_buf_ = nullptr;
    }
  }

  // Optional: remove the custom sink
  inline void RemoveGlogCustomSink() {
    if (g_custom_sink) {
      google::RemoveLogSink(g_custom_sink.get());
      g_custom_sink.reset();
    }
  }

  void logGlogMessages(std::string entry,
                       google::LogSeverity severity,
                       const char* filename,
                       int line,
                       const char* message) {
    // glog severity to Rerun log level
    rerun::TextLogLevel level;
    switch (severity) {
      case google::GLOG_INFO:
        level = rerun::TextLogLevel::Info;
        break;
      case google::GLOG_WARNING:
        level = rerun::TextLogLevel::Warning;
        break;
      case google::GLOG_ERROR:
        level = rerun::TextLogLevel::Error;
        break;
      case google::GLOG_FATAL:
        level = rerun::TextLogLevel::Critical;
        break;
      default:
        level = rerun::TextLogLevel::Debug;  // Default to Debug for other
                                             // severities
    }

    // Forward glog messages to Rerun
    this->rec()->log(
        (entry + "/glog").c_str(),
        rerun::TextLog(fmt::format("{}", message)).with_level(level));
  }

  VIO::VisualizerOutput::UniquePtr spinOnce(
      const VIO::VisualizerInput& input) override {
    std::lock_guard<std::mutex> lock(rerun_mutex_);
    this->setTimeNSec(input.timestamp_);
    this->drawTf(map_ / odom_ / baselink_,
                 input.backend_output_->W_State_Blkf_.pose_,
                 1.0,
                 false);

    odom_traj_.push_back(input.backend_output_->W_State_Blkf_.pose_);
    odom_states_.insert(input.backend_output_->cur_kf_id_,
                        input.backend_output_->W_State_Blkf_.pose_);
    timestamp_map_.insert(
        {input.backend_output_->cur_kf_id_, input.timestamp_});
    this->drawTrajectory(map_ / odom_ / "trajectory",
                         odom_traj_,
                         aria::viz::ColorMap::kGreen,
                         1.f,
                         false);

    // Draw covariance for Detailed and Debug profiles
    if (profile_ >= VisualizationProfile::Detailed) {
      auto cur_cov = input.backend_output_->state_covariance_lkf_;
      auto half_green = aria::viz::ColorMap::kGreen;
      half_green[3] = 128;
      this->drawUncertainty(map_ / odom_ / baselink_ / "covariance",
                            Pose3::Identity(),
                            cur_cov.block<3, 3>(3, 3),
                            half_green,
                            0.1);
    }

    cv::Mat tracking_image_clone =
        input.frontend_output_->getTrackingImage()->clone();

    auto K = input.frontend_output_->getTrackingFrame()->cam_param_.K_;

    // only draw every 3 frames
    // if (input.backend_output_->cur_kf_id_ % 3 == 0) {
    //   this->drawCamera(input.backend_output_->cur_kf_id_,
    //                    input.backend_output_->W_State_Blkf_.pose_,
    //                    tracking_image_clone,
    //                    K,
    //                    false);
    // }

    // Draw tracking image for Standard and above
    if (profile_ >= VisualizationProfile::Standard &&
        not input.frontend_output_->getTrackingImage()->empty()) {
      cv::Mat small_image;
      // resize to width 320 while keeping aspect ratio
      double aspect_ratio = static_cast<double>(tracking_image_clone.cols) /
                            static_cast<double>(tracking_image_clone.rows);
      int new_width = 320;
      int new_height = static_cast<int>(new_width / aspect_ratio);
      cv::resize(
          tracking_image_clone, small_image, cv::Size(new_width, new_height));
      this->drawImage(
          (robot_name_ + "/tracking/image").c_str(), small_image, false);
    }

    Landmarks lmks_vec;
    // convert landmark id->landmark map to vector
    for (const auto& [id, landmark] :
         input.backend_output_->landmarks_in_local_window_) {
      lmks_vec.push_back(landmark);
    }

    // find the earliest pose
    FrameId earliest_frame = std::numeric_limits<FrameId>::max();
    for (auto key : input.backend_output_->state_.keys()) {
      Symbol symbol(key);
      if (symbol.chr() == kPoseSymbolChar) {
        if (symbol.index() < earliest_frame) {
          earliest_frame = symbol.index();
        }
      }
    }

    Pose3 T_smoother_pose = input.backend_output_->state_.at<Pose3>(
        gtsam::Symbol(kPoseSymbolChar, earliest_frame));
    Pose3 T_odom_pose = odom_states_.at<Pose3>(earliest_frame);
    Pose3 W_T_smoother = T_odom_pose * T_smoother_pose.inverse();

    // Draw smoother states for Detailed and above
    if (profile_ >= VisualizationProfile::Detailed) {
      smoother_states_.clear();
      smoother_states_.insert_or_assign(input.backend_output_->state_);
      visualizeLandmarks(
          map_ / odom_ / "smoother", lmks_vec, aria::viz::ColorMap::kRed);
      drawTf(map_ / odom_ / "smoother", W_T_smoother);
      drawPoints(map_ / odom_ / "smoother" / "states",
                 input.backend_output_->state_,
                 {aria::viz::ColorMap::kRed},
                 {0.5});
      pose_states_.clear();
      for (const auto& key : smoother_states_.keys()) {
        Symbol symbol(key);
        if (symbol.chr() == kPoseSymbolChar) {
          pose_states_.insert_or_assign(symbol.index(),
                                        smoother_states_.at<Pose3>(key));
        }
      }
    }

    // Draw stereo depth/disparity for Detailed and above
    if (profile_ >= VisualizationProfile::Detailed) {
      if (auto stereo_output = std::dynamic_pointer_cast<StereoFrontendOutput>(
              input.frontend_output_)) {
        // visualize the disparity map if available
        if (!stereo_output->stereo_frame_lkf_.left_disp_img_.empty()) {
          // Convert disparity to grayscale 8-bit image (clip to 0-255, no
          // normalization)
          cv::Mat disp_gray;
          stereo_output->stereo_frame_lkf_.left_disp_img_.convertTo(disp_gray,
                                                                    CV_8U);

          // resize to width 320 while keeping aspect ratio
          double aspect_ratio = static_cast<double>(disp_gray.cols) /
                                static_cast<double>(disp_gray.rows);
          int new_width = 320;
          int new_height = static_cast<int>(new_width / aspect_ratio);
          cv::resize(disp_gray, disp_gray, cv::Size(new_width, new_height));

          this->drawImage(
              (robot_name_ + "/stereo/disp_map").c_str(), disp_gray, false);
        }

        auto depth_image = stereo_output->stereo_frame_lkf_.left_depth_img_;
        // resize to width 320 while keeping aspect ratio
        double aspect_ratio = static_cast<double>(depth_image.cols) /
                              static_cast<double>(depth_image.rows);
        int new_width = 320;
        int new_height = static_cast<int>(new_width / aspect_ratio);
        cv::resize(depth_image, depth_image, cv::Size(new_width, new_height));

        auto T_bl_cam = *stereo_output->getBodyPoseCam();
        this->drawTf(
            map_ / odom_ / baselink_ / "left_cam", T_bl_cam, 0.5, false);

        // draw raw data
        this->drawDepthImage(
            map_ / odom_ / baselink_ / "left_cam", depth_image, false);
      }
    }

    // Check if it's time to save trajectories (every 10 seconds)
    this->checkAndSaveTrajectories(odom_states_);

    // Check if it's time to draw GT trajectory (every 5 seconds)
    this->checkAndDrawGtTraj(odom_states_, timestamp_map_);

    this->publishLcdOutput(input.lcd_output_);

    return std::make_unique<VIO::VisualizerOutput>();
  }

  void drawCamera(int id,
                  const Pose3& cam_pose,
                  const cv::Mat& image,
                  const cv::Mat& K,
                  bool is_static = true) {
    // draw the tf for this camera
    std::string cam_name = fmt::format("f{}", id);
    this->drawTf(map_ / odom_ / "camera" / cam_name, cam_pose, 0, is_static);

    if (not image.empty()) {
      cv::Mat rgba32;
      if (image.type() == CV_8UC3) {
        cv::cvtColor(image, rgba32, cv::COLOR_BGR2RGBA);
      } else if (image.type() == CV_8UC1) {
        cv::cvtColor(image, rgba32, cv::COLOR_GRAY2RGBA);
      } else if (image.type() == CV_8UC4) {
        rgba32 = image;
      } else {
        throw std::runtime_error("Unsupported image type");
      }
      std::array<float, 9> K_vec = {
          static_cast<float>(K.at<double>(0, 0)),  // fx
          0.f,
          static_cast<float>(K.at<double>(0, 2)),  // cx
          0.f,
          static_cast<float>(K.at<double>(1, 1)),  // fy
          static_cast<float>(K.at<double>(1, 2)),  // cy
          0.f,
          0.f,
          1.f};
      rerun::components::PinholeProjection pp(K_vec);

      this->rec()->log_with_static(
          (map_ / odom_ / "camera" / cam_name).c_str(),
          is_static,
          rerun::Image::from_rgba32(rgba32,
                                    {static_cast<uint32_t>(image.cols),
                                     static_cast<uint32_t>(image.rows)}));
      // draw the camera
      this->rec()->log_with_static(
          (map_ / odom_ / "camera" / cam_name).c_str(),
          is_static,
          rerun::Pinhole::from_focal_length_and_resolution(
              K_vec[0],
              {static_cast<float>(image.cols), static_cast<float>(image.rows)})
              .with_image_plane_distance(0.1f)
              .with_camera_xyz(rerun::components::ViewCoordinates::FRD));
      // .with_image_from_camera(pp));
    }
  }

  void drawGtTraj(gtsam::Values est_traj_values,
                  const FrameIDTimestampMap& timestamp_map) {
    if (not gt_trajectory_.empty()) {
      if (est_traj_values.size() - prev_alignment_size_ > 50) {
        prev_alignment_size_ = est_traj_values.size();
        // extract the poses from the gtsam::Values
        std::map<FrameId, gtsam::Pose3> est_poses;
        std::map<FrameId, gtsam::Pose3> gt_poses;
        for (const auto& key : est_traj_values.keys()) {
          if (timestamp_map.find(key) != timestamp_map.end()) {
            Timestamp timestamp = timestamp_map.at(key);
            // find the closest timestamp in gt_trajectory_
            auto it = gt_trajectory_.lower_bound(timestamp);
            if (it != gt_trajectory_.end()) {
              est_poses[key] = est_traj_values.at<gtsam::Pose3>(key);
              gt_poses[key] = it->second;
            }
          }
        }

        // align the poses to the map frame
        gtsam::Point3Pairs poses_to_align;
        for (const auto& [key, pose] : est_poses) {
          if (gt_poses.find(key) != gt_poses.end()) {
            poses_to_align.emplace_back(pose.translation(),
                                        gt_poses.at(key).translation());
          }
        }
        auto T_est_gt = gtsam::Pose3::Align(poses_to_align);
        if (T_est_gt) {
          T_map_gt_ = T_est_gt.value();
          rec()->log(
              (robot_name_ + "/gt_align").c_str(),
              rerun::TextLog(
                  fmt::format("Aligned {} pairs to GT with t_map_gt: {},{},{}",
                              poses_to_align.size(),
                              T_map_gt_.x(),
                              T_map_gt_.y(),
                              T_map_gt_.z()))
                  .with_level(rerun::TextLogLevel::Info));
        } else {
          rec()->log(
              (robot_name_ + "/gt_align").c_str(),
              rerun::TextLog("Failed to align estimated trajectory to GT.")
                  .with_level(rerun::TextLogLevel::Error));
        }

        this->drawTf(map_ / "gt", T_map_gt_, 1.0, false);

        std::vector<gtsam::Pose3> gt_traj;
        for (const auto& [key, pose] : gt_trajectory_) {
          gt_traj.push_back(pose);
        }
      }
    }
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

  void visualizeLandmarks(std::filesystem::path base_frame,
                          const Landmarks& landmarks,
                          Eigen::Vector4f color) {
    std::vector<Point3> lmk_points(landmarks.begin(), landmarks.end());

    this->drawPoints(
        base_frame / "landmarks", lmk_points, {color}, {0.001}, {}, false);
  }

  void visualizeCovisGraph(std::map<FrameId, FrameIdSet> covis_graph,
                           const Values& states) {
    NonlinearFactorGraph symbolic_graph;
    for (const auto& [key, neighbors] : covis_graph) {
      for (const auto& neighbor : neighbors) {
        symbolic_graph.add(
            boost::make_shared<BetweenFactor<Pose3>>(key, neighbor, Pose3{}));
      }
    }

    auto gray = aria::viz::ColorMap::kGray;

    this->drawFactors(map_ / "covis_graph",
                      symbolic_graph,
                      states,
                      aria::viz::ColorMap::kGreen,
                      1.f,
                      false,
                      true);
  }

  void visualizeLCDQueryAndCandidates(FrameId query_id,
                                      FrameIdSet candidates,
                                      const Values& states) {
    Values query_state;
    query_state.insert(query_id, states.at(query_id));

    Values candidates_states;
    for (const auto& candidate_id : candidates) {
      candidates_states.insert(candidate_id, states.at(candidate_id));
    }

    this->drawPoints(map_ / "lcd" / "query",
                     query_state,
                     {aria::viz::ColorMap::kRed},
                     {100},
                     {},
                     false);

    this->drawPoints(map_ / "lcd" / "global_candidates",
                     candidates_states,
                     {aria::viz::ColorMap::kGreen},
                     {100},
                     {},
                     false);
  }

  void checkAndSaveTrajectories(const gtsam::Values& states = gtsam::Values()) {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_save_time_ >= save_interval_) {
      if (!states.empty() && !timestamp_map_.empty()) {
        this->saveTUMTrajFile(states, timestamp_map_, "DE-VIO");
      }
      last_save_time_ = current_time;
    }
  }

  void checkAndDrawGtTraj(const gtsam::Values& states,
                          const FrameIDTimestampMap& timestamp_map) {
    auto current_time = std::chrono::steady_clock::now();
    if (current_time - last_gt_draw_time_ >= gt_draw_interval_) {
      if (!states.empty() && !timestamp_map.empty()) {
        this->drawGtTraj(states, timestamp_map);
      }
      last_gt_draw_time_ = current_time;
    }
  }

  void forceSaveTrajectories() {
    if (!odom_states_.empty() && !timestamp_map_.empty()) {
      this->saveTUMTrajFile(odom_states_, timestamp_map_, "DE-VIO");
    }
    last_save_time_ = std::chrono::steady_clock::now();
  }

  void saveTUMTrajFile(const gtsam::Values& states,
                       const FrameIDTimestampMap& timestamp_map,
                       std::string name) {
    if (result_dir_.empty()) {
      return;
    }
    if (save_traj_futures_.find(name) != save_traj_futures_.end() &&
        save_traj_futures_[name].valid() &&
        save_traj_futures_[name].wait_for(std::chrono::seconds(0)) !=
            std::future_status::ready) {
      return;  // Previous save is still in progress
    } else {
      save_traj_futures_.emplace(name, std::future<void>());
    }

    std::string filename = fmt::format("{}/{}.txt", result_dir_, name);

    auto write_traj = [&](const std::string& filename,
                          const gtsam::Values& states,
                          const FrameIDTimestampMap& timestamp_map) {
      // open file
      std::ofstream ofs(filename);
      if (!ofs.is_open()) {
        throw std::runtime_error("Failed to open file: " + filename);
      }

      for (auto const& [key, value] : states) {
        auto it = timestamp_map.find(key);
        Timestamp timestamp;
        if (it != timestamp_map.end()) {
          timestamp = it->second;
        } else {
          continue;
        }
        double x, y, z, qx, qy, qz, qw;
        Pose3 pose = value.cast<Pose3>();
        x = pose.x();
        y = pose.y();
        z = pose.z();
        auto quat = pose.rotation().toQuaternion();
        qx = quat.x();
        qy = quat.y();
        qz = quat.z();
        qw = quat.w();

        double seconds =
            static_cast<double>(timestamp) / 1e9;  // Convert ns to s

        ofs << std::fixed << std::setprecision(6) << seconds << " " << x << " "
            << y << " " << z << " " << qx << " " << qy << " " << qz << " " << qw
            << std::endl;
      }
      // close file
      ofs.close();
    };

    save_traj_futures_[name] = std::async(
        std::launch::async, write_traj, filename, states, timestamp_map);
  }

  std::vector<Eigen::Vector4f> getColorsFromFactorsType(
      const NonlinearFactorGraph& factors) {
    std::vector<Eigen::Vector4f> colors(factors.size(),
                                        aria::viz::ColorMap::kBlack);
    for (size_t i = 0; i < factors.size(); ++i) {
      const auto& factor = factors[i];
      if (factor == nullptr) {
        continue;
      }
      auto keys = factor->keys();
      if (keys.size() > 2) continue;
      uint64_t diff =
          (keys[0] > keys[1]) ? (keys[0] - keys[1]) : (keys[1] - keys[0]);
      if (diff == 1 or keys.size() == 1) {                // odometry edge
        colors[i] = Eigen::Vector4f(255, 255, 0.0, 255);  // yellow
        continue;
      } else if (diff > 1 and keys.size() == 2) {  // loop closure edge
        auto between_factor =
            boost::dynamic_pointer_cast<gtsam::BetweenFactor<gtsam::Pose3>>(
                factor);
        if (not between_factor) {
          factor->print();
          LOG(FATAL) << "Factor is not a BetweenFactor";
        }
        auto noise = between_factor->noiseModel();
        CHECK(noise);
        auto gauss =
            boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(noise);
        if (not gauss) {
          auto robust =
              boost::dynamic_pointer_cast<gtsam::noiseModel::Robust>(noise);
          gauss = boost::dynamic_pointer_cast<gtsam::noiseModel::Gaussian>(
              robust->noise());
        }
        CHECK(gauss);
        auto info = gauss->information();
        double trans_precision = info.block<3, 3>(3, 3).norm();
        // if trans precision too small, then rot only factor
        if (trans_precision < 1e-6) {
          colors[i] = aria::viz::ColorMap::kRed;
        } else {
          colors[i] = aria::viz::ColorMap::kGreen;
        }
      }
    }
    return colors;
  }

  void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output) override {
    if (lcd_output == nullptr) {
      return;
    }
    this->setTimeNSec(lcd_output->timestamp_);
    this->drawTf(map_ / odom_, lcd_output->Map_Pose_Odom_, 0.3, false);

    // Draw LCD landmarks for Standard and above
    if (profile_ >= VisualizationProfile::Standard) {
      visualizeLandmarks(map_ / odom_,
                         lcd_output->landmarks_,
                         Eigen::Vector4f(255, 255, 255, 150));
    }

    // Draw pose graph for Debug profile
    auto opt_traj = lcd_output->states_;
    if (not opt_traj.empty() && profile_ >= VisualizationProfile::Debug) {
      this->drawFactors(map_ / "pose_graph",
                        lcd_output->nfg_,
                        lcd_output->states_,
                        getColorsFromFactorsType(lcd_output->nfg_),
                        1.f,
                        false,
                        true);

      visualizeCovisGraph(lcd_output->covis_graph_, lcd_output->states_);
    }

    // Visualize LCD keypoints
    if (!lcd_output->keypoints_3d_.empty()) {
      // Draw 3D keypoints in the map frame
      std::vector<Point3> kpts_3d(lcd_output->keypoints_3d_.begin(),
                                  lcd_output->keypoints_3d_.end());
      auto cyan_color = Eigen::Vector4f(0, 255, 255, 255);
      this->drawPoints(map_ / "lcd" / "keypoints_3d",
                       kpts_3d,
                       {cyan_color},
                       {2.0},
                       {},
                       false);

      VLOG(3) << "Visualized " << lcd_output->keypoints_3d_.size()
              << " LCD 3D keypoints";
    }

    if (!lcd_output->keypoints_2d_.empty()) {
      // Draw 2D keypoints as image points
      std::vector<rerun::Position2D> kpts_2d_positions;
      kpts_2d_positions.reserve(lcd_output->keypoints_2d_.size());
      std::vector<rerun::Color> colors;

      auto dist_to_color = [](double distance) {
        // Map distance to color (closer = blue, farther = red)
        uint8_t r = static_cast<uint8_t>(std::min(255.0, distance * 3.0));
        uint8_t g = 0;
        uint8_t b = static_cast<uint8_t>(std::max(0.0, 255.0 - distance * 3.0));
        return rerun::Color(r, g, b);
      };

      for (int i = 0; i < lcd_output->keypoints_2d_.size(); ++i) {
        const auto& kpt = lcd_output->keypoints_2d_[i];
        kpts_2d_positions.emplace_back(kpt.x, kpt.y);

        auto kp_3d = lcd_output->keypoints_3d_.at(i);
        double distance = kp_3d.norm();
        colors.push_back(dist_to_color(distance));
      }

      this->rec()->log((robot_name_ + "/lcd/keypoints_2d").c_str(),
                       rerun::Points2D(kpts_2d_positions)
                           .with_radii({3.0f})
                           .with_colors(colors));

      VLOG(3) << "Visualized " << lcd_output->keypoints_2d_.size()
              << " LCD 2D keypoints";
    }
  }

  std::map<Timestamp, gtsam::Pose3> loadTrajectoryMapFromCSV(
      const std::string& filename) {
    std::map<int64_t, gtsam::Pose3> trajectory;
    std::ifstream file(filename);
    if (!file.is_open()) {
      throw std::runtime_error("Failed to open file: " + filename);
    }

    std::string line;
    while (std::getline(file, line)) {
      if (line.empty()) continue;

      std::istringstream iss(line);
      std::vector<double> values;
      std::string token;

      while (std::getline(iss, token, ',')) {
        try {
          values.push_back(std::stod(token));
        } catch (const std::invalid_argument&) {
          std::cerr << "Invalid number in line: " << line << std::endl;
          values.clear();
          break;
        }
      }

      if (values.size() != 8) {
        std::cerr << "Skipping malformed line: " << line << std::endl;
        continue;
      }

      // Convert timestamp to int64_t (assume it's in seconds, multiply to get
      // nanoseconds)
      int64_t timestamp_ns = static_cast<int64_t>(values[0] * 1e9);

      double x = values[1];
      double y = values[2];
      double z = values[3];
      double qx = values[4];
      double qy = values[5];
      double qz = values[6];
      double qw = values[7];

      gtsam::Rot3 R = gtsam::Rot3::Quaternion(qw, qx, qy, qz);
      gtsam::Point3 t(x, y, z);
      trajectory[timestamp_ns] = gtsam::Pose3(R, t);
    }

    return trajectory;
  }

 private:
  std::filesystem::path baselink_;
  std::filesystem::path map_;
  std::filesystem::path odom_;
  std::string robot_name_;

  std::vector<Pose3> odom_traj_{};
  gtsam::Values odom_states_;

  std::future<void> draw_gt_traj_future_;
  std::map<std::string, std::future<void>> save_traj_futures_;
  std::future<void> lcd_output_future_;

  gtsam::Values smoother_states_;
  gtsam::Values pose_states_;

  std::map<Timestamp, Pose3> gt_trajectory_;
  Pose3 T_map_gt_ = Pose3::Identity();
  size_t prev_alignment_size_ = 0;

  std::string result_dir_{};

  PointsWithIdMap landmarks_in_odom_;

  FrameIDTimestampMap timestamp_map_;

  std::optional<std::pair<FrameId, FrameId>> last_odom_pair_{std::nullopt};
  ISAM2 isam2_;

  std::mutex rerun_mutex_;

  // Visualization profile
  VisualizationProfile profile_;

  // Timer for saving trajectory files
  std::chrono::steady_clock::time_point last_save_time_;
  static constexpr std::chrono::seconds save_interval_{10};

  // Timer for GT trajectory drawing
  std::chrono::steady_clock::time_point last_gt_draw_time_;
  static constexpr std::chrono::seconds gt_draw_interval_{5};
};

}  // namespace VIO