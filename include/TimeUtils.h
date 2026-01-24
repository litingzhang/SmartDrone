
class TimeUtils {
  public:
    static uint64_t NowNsMonotonic()
    {
        struct timespec ts;
        clock_gettime(CLOCK_MONOTONIC, &ts);
        return static_cast<uint64_t>(ts.tv_sec) * 1000000000ULL + static_cast<uint64_t>(ts.tv_nsec);
    }
    static double NsToSec(uint64_t ns) { return static_cast<double>(ns) * 1e-9; }
};
