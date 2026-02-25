#include <arpa/inet.h>

#include <netinet/in.h>
#include <sys/socket.h>
// ---------------- UDP image sender ----------------
class UdpImageSender {
  public:
    struct Job {
        int camIndex;  // 0=L, 1=R
        uint32_t seq;   // sequence for debug
        double frameTime; // seconds
        cv::Mat gray;   // CV_8UC1
    };

    bool Open(const std::string &ip, int port, int jpegQuality = 80,
              int maxPayload = 1200, // safe under typical MTU (1500)
              int maxQueue = 4)
    {
        m_jpegQuality = std::max(10, std::min(95, jpegQuality));
        m_maxPayload = std::max(400, maxPayload);
        m_maxQueue = std::max(1, maxQueue);

        m_sock = ::socket(AF_INET, SOCK_DGRAM, 0);
        if (m_sock < 0) {
            std::cerr << "[udp] socket() failed: " << strerror(errno) << "\n";
            return false;
        }

        memset(&m_dst, 0, sizeof(m_dst));
        m_dst.sin_family = AF_INET;
        m_dst.sin_port = htons((uint16_t)port);
        if (::inet_pton(AF_INET, ip.c_str(), &m_dst.sin_addr) != 1) {
            std::cerr << "[udp] inet_pton failed for " << ip << "\n";
            ::close(m_sock);
            m_sock = -1;
            return false;
        }

        m_running.store(true);
        m_th = std::thread([&] { Loop(); });
        return true;
    }

    void Close()
    {
        m_running.store(false);
        m_cv.notify_all();
        if (m_th.joinable())
            m_th.join();
        if (m_sock >= 0) {
            ::close(m_sock);
            m_sock = -1;
        }
        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_q.clear();
        }
    }

    // called from SLAM thread (non-blocking-ish)
    void Enqueue(int camIndex, uint32_t seq, double frameTime, const cv::Mat &gray)
    {
        if (m_sock < 0)
            return;
        Job job;
        job.camIndex = camIndex;
        job.seq = seq;
        job.frameTime = frameTime;
        job.gray = gray.clone(); // IMPORTANT: own data (libcamera buffer will be reused)

        {
            std::lock_guard<std::mutex> lk(m_mu);
            m_q.push_back(std::move(job));
            while ((int)m_q.size() > m_maxQueue)
                m_q.pop_front(); // drop old to keep real-time
        }
        m_cv.notify_one();
    }

  private:
#pragma pack(push, 1)
    struct PacketHeader {
        uint32_t magic;      // 'VSIM' 0x5643494D (or any)
        uint16_t version;    // 1
        uint8_t camIndex;   // 0/1
        uint8_t flags;       // reserved
        uint32_t seq;        // camera sequence
        double frameTime;      // seconds
        uint32_t frameId;   // incremental id for this sender
        uint16_t chunkIdx;  // 0..chunkCnt-1
        uint16_t chunkCnt;  // total chunks
        uint32_t totalSize; // jpeg bytes total
        uint32_t chunkSize; // bytes in this packet payload
    };
#pragma pack(pop)

    void Loop()
    {
        std::vector<uchar> jpeg;
        std::vector<int> params = {cv::IMWRITE_JPEG_QUALITY, m_jpegQuality};

        while (m_running.load()) {
            Job job;
            {
                std::unique_lock<std::mutex> lk(m_mu);
                m_cv.wait(lk, [&] { return !m_running.load() || !m_q.empty(); });
                if (!m_running.load())
                    break;
                job = std::move(m_q.front());
                m_q.pop_front();
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
            const uint16_t chunks = (uint16_t)((total + m_maxPayload - 1) / m_maxPayload);
            const uint32_t fid = m_frameId.fetch_add(1, std::memory_order_relaxed);

            for (uint16_t ci = 0; ci < chunks; ++ci) {
                const uint32_t off = (uint32_t)ci * (uint32_t)m_maxPayload;
                const uint32_t left = total - off;
                const uint32_t pay =
                    (left > (uint32_t)m_maxPayload) ? (uint32_t)m_maxPayload : left;

                PacketHeader h{};
                h.magic = 0x5643494D; // 'VCIM' just a tag
                h.version = 1;
                h.camIndex = (uint8_t)job.camIndex;
                h.flags = 0;
                h.seq = job.seq;
                h.frameTime = job.frameTime;
                h.frameId = fid;
                h.chunkIdx = ci;
                h.chunkCnt = chunks;
                h.totalSize = total;
                h.chunkSize = pay;

                // build packet buffer: header + payload
                std::vector<uint8_t> pkt(sizeof(PacketHeader) + pay);
                memcpy(pkt.data(), &h, sizeof(PacketHeader));
                memcpy(pkt.data() + sizeof(PacketHeader), jpeg.data() + off, pay);

                ssize_t sent =
                    ::sendto(m_sock, pkt.data(), pkt.size(), 0, (sockaddr *)&m_dst, sizeof(m_dst));
                (void)sent; // ignore drop; UDP is best-effort
            }
        }
    }

    int m_sock{-1};
    sockaddr_in m_dst{};

    int m_jpegQuality{80};
    int m_maxPayload{1200};
    int m_maxQueue{4};

    std::atomic<bool> m_running{false};
    std::thread m_th;
    std::mutex m_mu;
    std::condition_variable m_cv;
    std::deque<Job> m_q;

    std::atomic<uint32_t> m_frameId{1};
};
