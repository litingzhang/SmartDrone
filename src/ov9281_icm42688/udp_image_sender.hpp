#include <arpa/inet.h>

#include <netinet/in.h>
#include <sys/socket.h>
// ---------------- UDP image sender ----------------
class UdpImageSender {
  public:
    struct Job {
        int cam_index;  // 0=L, 1=R
        uint32_t seq;   // sequence for debug
        double frame_t; // seconds
        cv::Mat gray;   // CV_8UC1
    };

    bool Open(const std::string &ip, int port, int jpeg_quality = 80,
              int max_payload = 1200, // safe under typical MTU (1500)
              int max_queue = 4)
    {
        jpeg_quality_ = std::max(10, std::min(95, jpeg_quality));
        max_payload_ = std::max(400, max_payload);
        max_queue_ = std::max(1, max_queue);

        sock_ = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (sock_ < 0) {
            std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
            return false;
        }

        memset(&dst_, 0, sizeof(dst_));
        dst_.sin_family = AF_INET;
        dst_.sin_port = htons((uint16_t)port);
        if (::inet_pton(AF_INET, ip.c_str(), &dst_.sin_addr) != 1) {
            std::cerr << "[udp] inet_pton failed for " << ip << "\n";
            ::close(sock_);
            sock_ = -1;
            return false;
        }

        running_.store(true);
        th_ = std::thread([&] { Loop(); });
        return true;
    }

    void Close()
    {
        running_.store(false);
        cv_.notify_all();
        if (th_.joinable())
            th_.join();
        if (sock_ >= 0) {
            ::close(sock_);
            sock_ = -1;
        }
        {
            std::lock_guard<std::mutex> lk(mu_);
            q_.clear();
        }
    }

    // called from SLAM thread (non-blocking-ish)
    void Enqueue(int cam_index, uint32_t seq, double frame_t, const cv::Mat &gray)
    {
        if (sock_ < 0)
            return;
        Job job;
        job.cam_index = cam_index;
        job.seq = seq;
        job.frame_t = frame_t;
        job.gray = gray.clone(); // IMPORTANT: own data (libcamera buffer will be reused)

        {
            std::lock_guard<std::mutex> lk(mu_);
            q_.push_back(std::move(job));
            while ((int)q_.size() > max_queue_)
                q_.pop_front(); // drop old to keep real-time
        }
        cv_.notify_one();
    }

  private:
#pragma pack(push, 1)
    struct PacketHeader {
        uint32_t magic;      // 'VSIM' 0x5643494D (or any)
        uint16_t version;    // 1
        uint8_t cam_index;   // 0/1
        uint8_t flags;       // reserved
        uint32_t seq;        // camera sequence
        double frame_t;      // seconds
        uint32_t frame_id;   // incremental id for this sender
        uint16_t chunk_idx;  // 0..chunk_cnt-1
        uint16_t chunk_cnt;  // total chunks
        uint32_t total_size; // jpeg bytes total
        uint32_t chunk_size; // bytes in this packet payload
    };
#pragma pack(pop)

    void Loop()
    {
        std::vector<uchar> jpeg;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, jpeg_quality_};

        while (running_.load()) {
            Job job;
            {
                std::unique_lock<std::mutex> lk(mu_);
                cv_.wait(lk, [&] { return !running_.load() || !q_.empty(); });
                if (!running_.load())
                    break;
                job = std::move(q_.front());
                q_.pop_front();
            }

            // encode
            jpeg.clear();
            try {
                cv::imencode(".jpg", job.gray, jpeg, params);
            } catch (const std::exception &e) {
                std::cerr << "[udp] imencode exception: " << e.what() << "\n";
                continue;
            }
            if (jpeg.empty())
                continue;

            const uint32_t total = (uint32_t)jpeg.size();
            const uint16_t chunks = (uint16_t)((total + max_payload_ - 1) / max_payload_);
            const uint32_t fid = frame_id_.fetch_add(1, std::memory_order_relaxed);

            for (uint16_t ci = 0; ci < chunks; ++ci) {
                const uint32_t off = (uint32_t)ci * (uint32_t)max_payload_;
                const uint32_t left = total - off;
                const uint32_t pay =
                    (left > (uint32_t)max_payload_) ? (uint32_t)max_payload_ : left;

                PacketHeader h{};
                h.magic = 0x5643494D; // 'VCIM' just a tag
                h.version = 1;
                h.cam_index = (uint8_t)job.cam_index;
                h.flags = 0;
                h.seq = job.seq;
                h.frame_t = job.frame_t;
                h.frame_id = fid;
                h.chunk_idx = ci;
                h.chunk_cnt = chunks;
                h.total_size = total;
                h.chunk_size = pay;

                // build packet buffer: header + payload
                std::vector<uint8_t> pkt(sizeof(PacketHeader) + pay);
                memcpy(pkt.data(), &h, sizeof(PacketHeader));
                memcpy(pkt.data() + sizeof(PacketHeader), jpeg.data() + off, pay);

                ssize_t sent =
                    ::sendto(sock_, pkt.data(), pkt.size(), 0, (sockaddr *)&dst_, sizeof(dst_));
                (void)sent; // ignore drop; UDP is best-effort
            }
        }
    }

    int sock_{-1};
    sockaddr_in dst_{};

    int jpeg_quality_{80};
    int max_payload_{1200};
    int max_queue_{4};

    std::atomic<bool> running_{false};
    std::thread th_;
    std::mutex mu_;
    std::condition_variable cv_;
    std::deque<Job> q_;

    std::atomic<uint32_t> frame_id_{1};
};
