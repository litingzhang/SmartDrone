#pragma once

namespace smartdrone::adapters::slam {

struct DpvoGraphRuntimeOptions {
  bool persistentEdges{false};
  bool keyframeRemovalEnabled{false};
  bool capRebuiltEdges{false};
  int maxActiveEdges{1024};
};

struct DpvoStereoDepthOptions {
  int maxDisparity{36};
  float minScore{0.05f};
  float minMargin{0.01f};
};

DpvoGraphRuntimeOptions LoadDpvoGraphRuntimeOptions();
DpvoStereoDepthOptions LoadDpvoStereoDepthOptions(int rightWidth);

} // namespace smartdrone::adapters::slam
