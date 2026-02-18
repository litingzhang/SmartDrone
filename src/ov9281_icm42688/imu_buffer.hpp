

struct ImuSample {
    int64_t t_ns{};
    float ax{}, ay{}, az{}; // m/s^2
    float gx{}, gy{}, gz{}; // rad/s
};

struct ImuScale {
    float accel_lsb_per_g{2048.0f}; // default for 16g
    float gyro_lsb_per_dps{16.4f};  // default for 2000 dps
};

class ImuBuffer {
  public:
    void Push(const ImuSample &s)
    {
        std::lock_guard<std::mutex> lk(mu_);
        q_.push_back(s);

        const int64_t keep_ns = keep_sec_ * 1000000000LL;
        while (!q_.empty() && (q_.back().t_ns - q_.front().t_ns) > keep_ns) {
            q_.pop_front();
            if (last_used_idx_ > 0)
                last_used_idx_--;
        }
    }

    std::vector<ORB_SLAM3::IMU::Point> PopBetweenNs(int64_t t0_ns, int64_t t1_ns,
                                                    int64_t slack_before_ns, int64_t slack_after_ns)
    {
        std::vector<ORB_SLAM3::IMU::Point> out;
        std::lock_guard<std::mutex> lk(mu_);

        if (q_.empty())
            return out;
        if (t1_ns < t0_ns)
            return out;

        const int64_t a0 = t0_ns - slack_before_ns;
        const int64_t a1 = t1_ns + slack_after_ns;

        size_t i = std::min(last_used_idx_, q_.size());
        while (i < q_.size() && q_[i].t_ns <= a0)
            i++;

        size_t j = i;
        while (j < q_.size() && q_[j].t_ns <= a1) {
            const auto &s = q_[j];
            if (s.t_ns > t0_ns && s.t_ns <= t1_ns) {
                const double ts = (double)s.t_ns * 1e-9;
                out.emplace_back(cv::Point3f(s.ax, s.ay, s.az), cv::Point3f(s.gx, s.gy, s.gz), ts);
            }
            j++;
        }

        if (!out.empty()) {
            size_t k = i;
            while (k < q_.size() && q_[k].t_ns <= t1_ns)
                k++;
            last_used_idx_ = k;
        }

        const int64_t purge_before = t0_ns - purge_margin_ns_;
        while (!q_.empty() && q_.front().t_ns < purge_before) {
            q_.pop_front();
            if (last_used_idx_ > 0)
                last_used_idx_--;
        }

        return out;
    }

    size_t Size() const
    {
        std::lock_guard<std::mutex> lk(mu_);
        return q_.size();
    }

    bool PeekFirstLast(int64_t &t_first, int64_t &t_last) const
    {
        std::lock_guard<std::mutex> lk(mu_);
        if (q_.empty())
            return false;
        t_first = q_.front().t_ns;
        t_last = q_.back().t_ns;
        return true;
    }

  private:
    mutable std::mutex mu_;
    std::deque<ImuSample> q_;
    size_t last_used_idx_{0};

    int keep_sec_{5};
    int64_t purge_margin_ns_{20000000}; // 20ms
};
