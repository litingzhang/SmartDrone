#include <linux/spi/spidev.h>

static constexpr uint8_t SPI_READ_MASK = 0x80;

class SpiDev {
  public:
    explicit SpiDev(std::string dev) : dev_(std::move(dev)) {}

    bool Open(uint32_t speed_hz, uint8_t mode, uint8_t bits_per_word)
    {
        fd_ = ::open(dev_.c_str(), O_RDWR);
        if (fd_ < 0) {
            std::cerr << "open " << dev_ << " failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(fd_, SPI_IOC_WR_MODE, &mode) < 0 || ioctl(fd_, SPI_IOC_RD_MODE, &mode) < 0) {
            std::cerr << "SPI set mode failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(fd_, SPI_IOC_WR_BITS_PER_WORD, &bits_per_word) < 0 ||
            ioctl(fd_, SPI_IOC_RD_BITS_PER_WORD, &bits_per_word) < 0) {
            std::cerr << "SPI set bits failed: " << strerror(errno) << "\n";
            return false;
        }
        if (ioctl(fd_, SPI_IOC_WR_MAX_SPEED_HZ, &speed_hz) < 0 ||
            ioctl(fd_, SPI_IOC_RD_MAX_SPEED_HZ, &speed_hz) < 0) {
            std::cerr << "SPI set speed failed: " << strerror(errno) << "\n";
            return false;
        }
        speed_hz_ = speed_hz;
        mode_ = mode;
        bits_ = bits_per_word;
        return true;
    }

    ~SpiDev()
    {
        if (fd_ >= 0)
            ::close(fd_);
    }

    bool WriteReg(uint8_t reg, uint8_t val)
    {
        uint8_t tx[2] = {uint8_t(reg & 0x7F), val};
        uint8_t rx[2] = {0, 0};
        return Transfer(tx, rx, sizeof(tx));
    }

    bool ReadReg(uint8_t reg, uint8_t &val)
    {
        uint8_t tx[2] = {uint8_t(SPI_READ_MASK | (reg & 0x7F)), 0x00};
        uint8_t rx[2] = {0, 0};
        if (!Transfer(tx, rx, sizeof(tx)))
            return false;
        val = rx[1];
        return true;
    }

    bool ReadRegs(uint8_t start_reg, uint8_t *out, size_t len)
    {
        std::vector<uint8_t> tx(len + 1, 0x00);
        std::vector<uint8_t> rx(len + 1, 0x00);
        tx[0] = uint8_t(SPI_READ_MASK | (start_reg & 0x7F));
        if (!Transfer(tx.data(), rx.data(), rx.size()))
            return false;
        memcpy(out, rx.data() + 1, len);
        return true;
    }

  private:
    bool Transfer(const uint8_t *tx, uint8_t *rx, size_t len)
    {
        spi_ioc_transfer tr{};
        tr.tx_buf = (unsigned long)tx;
        tr.rx_buf = (unsigned long)rx;
        tr.len = (uint32_t)len;
        tr.speed_hz = speed_hz_;
        tr.bits_per_word = bits_;
        tr.delay_usecs = 0;
        if (ioctl(fd_, SPI_IOC_MESSAGE(1), &tr) < 0) {
            std::cerr << "SPI transfer failed: " << strerror(errno) << "\n";
            return false;
        }
        return true;
    }

    std::string dev_;
    int fd_{-1};
    uint32_t speed_hz_{8000000};
    uint8_t mode_{SPI_MODE_0};
    uint8_t bits_{8};
};
