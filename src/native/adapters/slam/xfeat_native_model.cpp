#include "adapters/slam/xfeat_native_model.h"

#include <cstdlib>
#include <iostream>
#include <utility>
#include <vector>

namespace smartdrone::adapters::slam {

XFeatBasicLayerImpl::XFeatBasicLayerImpl(int inChannels, int outChannels, int kernelSize, int stride, int padding)
{
    layer = torch::nn::Sequential(
        torch::nn::Conv2d(torch::nn::Conv2dOptions(inChannels, outChannels, kernelSize)
                              .padding(padding)
                              .stride(stride)
                              .dilation(1)
                              .bias(false)),
        torch::nn::BatchNorm2d(torch::nn::BatchNorm2dOptions(outChannels).affine(false)),
        torch::nn::ReLU(torch::nn::ReLUOptions().inplace(true)));
    register_module("layer", layer);
}

torch::Tensor XFeatBasicLayerImpl::forward(torch::Tensor x) { return layer->forward(std::move(x)); }

XFeatModelImpl::XFeatModelImpl()
{
    norm = torch::nn::InstanceNorm2d(1);

    skip1 = torch::nn::Sequential(
        torch::nn::AvgPool2d(torch::nn::AvgPool2dOptions(4).stride(4)),
        torch::nn::Conv2d(torch::nn::Conv2dOptions(1, 24, 1).stride(1).padding(0)));

    block1 = torch::nn::Sequential(
        XFeatBasicLayer(1, 4, 3, 1, 1), XFeatBasicLayer(4, 8, 3, 2, 1), XFeatBasicLayer(8, 8, 3, 1, 1),
        XFeatBasicLayer(8, 24, 3, 2, 1));

    block2 = torch::nn::Sequential(XFeatBasicLayer(24, 24, 3, 1, 1), XFeatBasicLayer(24, 24, 3, 1, 1));

    block3 = torch::nn::Sequential(
        XFeatBasicLayer(24, 64, 3, 2, 1), XFeatBasicLayer(64, 64, 3, 1, 1), XFeatBasicLayer(64, 64, 1, 1, 0));

    block4 = torch::nn::Sequential(
        XFeatBasicLayer(64, 64, 3, 2, 1), XFeatBasicLayer(64, 64, 3, 1, 1), XFeatBasicLayer(64, 64, 3, 1, 1));

    block5 = torch::nn::Sequential(XFeatBasicLayer(64, 128, 3, 2, 1), XFeatBasicLayer(128, 128, 3, 1, 1),
                                   XFeatBasicLayer(128, 128, 3, 1, 1), XFeatBasicLayer(128, 64, 1, 1, 0));

    blockFusion = torch::nn::Sequential(
        XFeatBasicLayer(64, 64, 3, 1, 1), XFeatBasicLayer(64, 64, 3, 1, 1),
        torch::nn::Conv2d(torch::nn::Conv2dOptions(64, 64, 1).padding(0)));

    heatmapHead = torch::nn::Sequential(
        XFeatBasicLayer(64, 64, 1, 1, 0), XFeatBasicLayer(64, 64, 1, 1, 0),
        torch::nn::Conv2d(torch::nn::Conv2dOptions(64, 1, 1)), torch::nn::Sigmoid());

    keypointHead = torch::nn::Sequential(
        XFeatBasicLayer(64, 64, 1, 1, 0), XFeatBasicLayer(64, 64, 1, 1, 0), XFeatBasicLayer(64, 64, 1, 1, 0),
        torch::nn::Conv2d(torch::nn::Conv2dOptions(64, 65, 1)));

    fineMatcher = torch::nn::Sequential(
        torch::nn::Linear(128, 512), torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(512).affine(false)),
        torch::nn::ReLU(torch::nn::ReLUOptions().inplace(true)), torch::nn::Linear(512, 512),
        torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(512).affine(false)),
        torch::nn::ReLU(torch::nn::ReLUOptions().inplace(true)), torch::nn::Linear(512, 512),
        torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(512).affine(false)),
        torch::nn::ReLU(torch::nn::ReLUOptions().inplace(true)), torch::nn::Linear(512, 512),
        torch::nn::BatchNorm1d(torch::nn::BatchNorm1dOptions(512).affine(false)),
        torch::nn::ReLU(torch::nn::ReLUOptions().inplace(true)), torch::nn::Linear(512, 64));

    register_module("norm", norm);
    register_module("skip1", skip1);
    register_module("block1", block1);
    register_module("block2", block2);
    register_module("block3", block3);
    register_module("block4", block4);
    register_module("block5", block5);
    register_module("block_fusion", blockFusion);
    register_module("heatmap_head", heatmapHead);
    register_module("keypoint_head", keypointHead);
    register_module("fine_matcher", fineMatcher);
}

torch::Tensor XFeatModelImpl::Unfold2d(torch::Tensor x, int windowSize)
{
    const auto shape = x.sizes();
    const int batch = static_cast<int>(shape[0]);
    const int channels = static_cast<int>(shape[1]);
    const int height = static_cast<int>(shape[2]);
    const int width = static_cast<int>(shape[3]);
    x = x.unfold(2, windowSize, windowSize)
            .unfold(3, windowSize, windowSize)
            .reshape({batch, channels, height / windowSize, width / windowSize, windowSize * windowSize});
    return x.permute({0, 1, 4, 2, 3}).reshape({batch, -1, height / windowSize, width / windowSize});
}

std::tuple<torch::Tensor, torch::Tensor, torch::Tensor> XFeatModelImpl::forward(torch::Tensor x)
{
    torch::NoGradGuard noGrad;
    x = x.mean(1, true);
    x = norm->forward(std::move(x));

    torch::Tensor x1 = block1->forward(x);
    torch::Tensor x2 = block2->forward(x1 + skip1->forward(x));
    torch::Tensor x3 = block3->forward(x2);
    torch::Tensor x4 = block4->forward(x3);
    torch::Tensor x5 = block5->forward(x4);

    const std::vector<int64_t> targetSize = {x3.size(2), x3.size(3)};
    x4 = torch::nn::functional::interpolate(
        x4, torch::nn::functional::InterpolateFuncOptions().size(targetSize).mode(torch::kBilinear).align_corners(
                false));
    x5 = torch::nn::functional::interpolate(
        x5, torch::nn::functional::InterpolateFuncOptions().size(targetSize).mode(torch::kBilinear).align_corners(
                false));
    torch::Tensor features = blockFusion->forward(x3 + x4 + x5);

    torch::Tensor heatmap = heatmapHead->forward(features);
    torch::Tensor keypoints = keypointHead->forward(Unfold2d(x, 8));
    return std::make_tuple(features, keypoints, heatmap);
}

XFeatSparseInterpolator::XFeatSparseInterpolator(std::string mode, bool alignCorners)
    : m_mode(std::move(mode)), m_alignCorners(alignCorners)
{
}

torch::Tensor XFeatSparseInterpolator::NormalizeGrid(torch::Tensor x, int height, int width)
{
    torch::Tensor sizeTensor = torch::tensor({width - 1, height - 1}, x.options());
    return 2.0 * (x / sizeTensor) - 1.0;
}

torch::Tensor XFeatSparseInterpolator::forward(torch::Tensor x, torch::Tensor positions, int height, int width)
{
    torch::Tensor grid = NormalizeGrid(positions, height, width).unsqueeze(-2).to(x.dtype());
    if (m_mode == "bilinear") {
        x = torch::nn::functional::grid_sample(
            x, grid,
            torch::nn::functional::GridSampleFuncOptions().mode(torch::kBilinear).align_corners(m_alignCorners));
    } else if (m_mode == "nearest") {
        x = torch::nn::functional::grid_sample(
            x, grid,
            torch::nn::functional::GridSampleFuncOptions().mode(torch::kNearest).align_corners(m_alignCorners));
    } else {
        std::cerr << "unsupported sparse interpolation mode: " << m_mode << std::endl;
        std::exit(EXIT_FAILURE);
    }
    return x.permute({0, 2, 3, 1}).squeeze(-2);
}

} // namespace smartdrone::adapters::slam
