#pragma once

#include <memory>
#include <string>

#include "core/ports/visual_place_recognition.h"

namespace SmartDrone::Adapters::Slam {

using VisualBowVector = Core::Ports::VisualBowVector;
using VisualFeatureIndex = Core::Ports::VisualFeatureIndex;
using VisualVocabularyTransform = Core::Ports::VisualVocabularyTransform;
using VisualWordWeight = Core::Ports::VisualWordWeight;

class OrbVisualVocabulary final : public Core::Ports::IVisualVocabulary {
  public:
    OrbVisualVocabulary();
    ~OrbVisualVocabulary() override;

    bool Load(const std::string &path) override;
    bool Loaded() const override;
    bool Transform(const cv::Mat &descriptors, VisualVocabularyTransform &out,
                   int levelsUp = 4) const override;
    double Score(const VisualBowVector &left, const VisualBowVector &right) const override;

  private:
    struct Impl;
    std::unique_ptr<Impl> m_impl;
};

} // namespace SmartDrone::Adapters::Slam
