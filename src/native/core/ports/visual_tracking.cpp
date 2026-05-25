#include "core/ports/visual_tracking.h"

namespace SmartDrone::Core::Ports {

void CopyStereoTracksToOutput(const std::vector<StereoTrack> &tracks,
                              SlamOutput &out)
{
    out.leftFeatures.reserve(out.leftFeatures.size() + tracks.size());
    out.rightFeatures.reserve(out.rightFeatures.size() + tracks.size());
    for (const StereoTrack &track : tracks) {
        out.leftFeatures.push_back(track.left);
        out.rightFeatures.push_back(track.right);
    }
}

} // namespace SmartDrone::Core::Ports
