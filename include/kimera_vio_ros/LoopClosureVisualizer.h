#pragma once

#include <kimera-vio/loopclosure/LoopClosureDetector-definitions.h>
#include <kimera-vio/loopclosure/LoopClosureDetector.h>

#include <memory>

namespace VIO {

class LoopClosureVisualizer {
 public:
  KIMERA_POINTER_TYPEDEFS(LoopClosureVisualizer);
  KIMERA_DELETE_COPY_CONSTRUCTORS(LoopClosureVisualizer);

  LoopClosureVisualizer() = default;
  virtual ~LoopClosureVisualizer() = default;

  // Main interface for publishing LCD output
  virtual void publishLcdOutput(const LcdOutput::ConstPtr& lcd_output) = 0;
};

}  // namespace VIO
