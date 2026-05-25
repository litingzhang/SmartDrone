#include "common/epg/epg_trigger_modes.h"

namespace Epg {

bool IsQueueTriggeredMode(TriggerMode mode)
{
    return mode == TriggerMode::AnyQueueReady ||
           mode == TriggerMode::AllQueueReady ||
           mode == TriggerMode::PeriodicOrAnyQueueReady;
}

} // namespace Epg
