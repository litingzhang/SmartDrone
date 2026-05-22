#pragma once

#include <cstdint>
#include <string>
#include <vector>

#include <opencv2/core.hpp>

namespace SmartDrone::Core::Ports {

struct VisualWordWeight {
    uint32_t wordId{0};
    double weight{0.0};
};

struct VisualBowVector {
    std::vector<VisualWordWeight> words;
};

struct VisualFeatureNode {
    uint32_t nodeId{0};
    std::vector<uint32_t> featureIndices;
};

struct VisualFeatureIndex {
    std::vector<VisualFeatureNode> nodes;
};

struct VisualVocabularyTransform {
    VisualBowVector bow;
    VisualFeatureIndex featureIndex;
};

class IVisualVocabulary {
  public:
    virtual ~IVisualVocabulary() = default;

    virtual bool Load(const std::string &path) = 0;
    virtual bool Loaded() const = 0;
    virtual bool Transform(const cv::Mat &descriptors,
                           VisualVocabularyTransform &out,
                           int levelsUp = 4) const = 0;
    virtual double Score(const VisualBowVector &left,
                         const VisualBowVector &right) const = 0;
};

} // namespace SmartDrone::Core::Ports
