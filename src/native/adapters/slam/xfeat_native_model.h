#pragma once

#include <tuple>

#include <torch/torch.h>

namespace smartdrone::adapters::slam {

struct XFeatBasicLayerImpl : torch::nn::Module {
    torch::nn::Sequential layer;

    XFeatBasicLayerImpl(int inChannels, int outChannels, int kernelSize, int stride, int padding);
    torch::Tensor forward(torch::Tensor x);
};

TORCH_MODULE(XFeatBasicLayer);

struct XFeatModelImpl : torch::nn::Module {
    torch::nn::InstanceNorm2d norm{nullptr};
    torch::nn::Sequential skip1{nullptr};
    torch::nn::Sequential block1{nullptr};
    torch::nn::Sequential block2{nullptr};
    torch::nn::Sequential block3{nullptr};
    torch::nn::Sequential block4{nullptr};
    torch::nn::Sequential block5{nullptr};
    torch::nn::Sequential blockFusion{nullptr};
    torch::nn::Sequential heatmapHead{nullptr};
    torch::nn::Sequential keypointHead{nullptr};
    torch::nn::Sequential fineMatcher{nullptr};

    XFeatModelImpl();
    torch::Tensor Unfold2d(torch::Tensor x, int windowSize = 2);
    std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> forward(torch::Tensor x);
};

TORCH_MODULE(XFeatModel);

class XFeatSparseInterpolator : public torch::nn::Module {
  public:
    explicit XFeatSparseInterpolator(std::string mode = "bilinear", bool alignCorners = false);

    torch::Tensor forward(torch::Tensor x, torch::Tensor positions, int height, int width);

  private:
    torch::Tensor NormalizeGrid(torch::Tensor x, int height, int width);

    std::string m_mode;
    bool m_alignCorners{false};
};

} // namespace smartdrone::adapters::slam
