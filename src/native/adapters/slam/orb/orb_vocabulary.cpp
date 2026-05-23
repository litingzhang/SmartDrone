#include "adapters/slam/orb/orb_vocabulary.h"
#include "adapters/slam/orb/orb_slam3/features/orb_vocabulary_bridge.h"

#include <algorithm>
#include <iostream>
#include <memory>
#include <sys/stat.h>
#include <utility>
#include <vector>

#include "ORBVocabulary.h"

namespace SmartDrone::Adapters::Slam {

namespace {

bool GetFileMTime(const std::string &path, time_t &outTime)
{
    struct stat st {};
    if (::stat(path.c_str(), &st) != 0) {
        return false;
    }
    outTime = st.st_mtime;
    return true;
}

std::vector<cv::Mat> ToDescriptorRows(const cv::Mat &descriptors)
{
    std::vector<cv::Mat> rows;
    rows.reserve(static_cast<size_t>(std::max(0, descriptors.rows)));
    for (int row = 0; row < descriptors.rows; ++row) {
        rows.push_back(descriptors.row(row));
    }
    return rows;
}

VisualBowVector ToVisualBowVector(const DBoW2::BowVector &bow)
{
    VisualBowVector out;
    out.words.reserve(bow.size());
    for (const auto &entry : bow) {
        out.words.push_back({static_cast<uint32_t>(entry.first), entry.second});
    }
    return out;
}

DBoW2::BowVector ToDbowBowVector(const VisualBowVector &bow)
{
    DBoW2::BowVector out;
    for (const VisualWordWeight &word : bow.words) {
        out[static_cast<DBoW2::WordId>(word.wordId)] = word.weight;
    }
    return out;
}

VisualFeatureIndex ToVisualFeatureIndex(const DBoW2::FeatureVector &features)
{
    VisualFeatureIndex out;
    out.nodes.reserve(features.size());
    for (const auto &entry : features) {
        Core::Ports::VisualFeatureNode node;
        node.nodeId = static_cast<uint32_t>(entry.first);
        node.featureIndices.reserve(entry.second.size());
        for (unsigned int index : entry.second) {
            node.featureIndices.push_back(static_cast<uint32_t>(index));
        }
        out.nodes.push_back(std::move(node));
    }
    return out;
}

} // namespace

class OrbVisualVocabulary::Impl {
  public:
    Impl()
        : vocabulary(std::make_unique<ORB_SLAM3::ORBVocabulary>())
    {
    }

    std::unique_ptr<ORB_SLAM3::ORBVocabulary> vocabulary;
};

bool LoadOrbVocabularyWithCache(const std::string &textPath, ORB_SLAM3::ORBVocabulary &vocabulary)
{
    const std::string binFile = textPath + ".bin";
    time_t textTime = 0;
    time_t binTime = 0;
    const bool hasText = GetFileMTime(textPath, textTime);
    const bool hasBin = GetFileMTime(binFile, binTime);
    if (hasBin) {
        const bool useBin = !hasText || binTime >= textTime;
        if (useBin && vocabulary.loadFromBinaryFile(binFile)) {
            std::cerr << "Vocabulary loaded from binary cache: " << binFile << std::endl
                      << std::endl;
            return true;
        }
        std::cerr << "Binary vocabulary cache load failed, fallback to text: " << binFile << std::endl;
    }

    if (!hasText) {
        return false;
    }
    if (!vocabulary.loadFromTextFile(textPath)) {
        return false;
    }
    vocabulary.saveToBinaryFile(binFile);
    std::cerr << "Vocabulary cached to binary file: " << binFile << std::endl;
    return true;
}

bool TransformOrbDescriptors(const ORB_SLAM3::ORBVocabulary &vocabulary, const cv::Mat &descriptors,
                             DBoW2::BowVector &bow, DBoW2::FeatureVector &features, int levelsUp)
{
    bow.clear();
    features.clear();
    if (vocabulary.empty() || descriptors.empty() || descriptors.type() != CV_8U) {
        return false;
    }

    const std::vector<cv::Mat> rows = ToDescriptorRows(descriptors);
    vocabulary.transform(rows, bow, features, levelsUp);
    return !bow.empty();
}

bool TransformOrbDescriptors(const ORB_SLAM3::ORBVocabulary &vocabulary, const cv::Mat &descriptors,
                             VisualVocabularyTransform &out, int levelsUp)
{
    out = {};
    DBoW2::BowVector bow;
    DBoW2::FeatureVector features;
    if (!TransformOrbDescriptors(vocabulary, descriptors, bow, features, levelsUp)) {
        return false;
    }
    out.bow = ToVisualBowVector(bow);
    out.featureIndex = ToVisualFeatureIndex(features);
    return true;
}

OrbVisualVocabulary::OrbVisualVocabulary()
    : m_impl(std::make_unique<Impl>())
{
}

OrbVisualVocabulary::~OrbVisualVocabulary() = default;

bool OrbVisualVocabulary::Load(const std::string &path)
{
    if (path.empty()) {
        return false;
    }
    auto vocabulary = std::make_unique<ORB_SLAM3::ORBVocabulary>();
    if (!LoadOrbVocabularyWithCache(path, *vocabulary)) {
        return false;
    }
    if (!m_impl) {
        m_impl = std::make_unique<Impl>();
    }
    m_impl->vocabulary = std::move(vocabulary);
    return true;
}

bool OrbVisualVocabulary::Loaded() const
{
    return m_impl != nullptr && m_impl->vocabulary != nullptr && !m_impl->vocabulary->empty();
}

bool OrbVisualVocabulary::Transform(const cv::Mat &descriptors, VisualVocabularyTransform &out,
                                    int levelsUp) const
{
    return Loaded() && TransformOrbDescriptors(*m_impl->vocabulary, descriptors, out, levelsUp);
}

double OrbVisualVocabulary::Score(const VisualBowVector &left, const VisualBowVector &right) const
{
    if (!Loaded()) {
        return 0.0;
    }
    return m_impl->vocabulary->score(ToDbowBowVector(left), ToDbowBowVector(right));
}

} // namespace SmartDrone::Adapters::Slam
