#include <gpiod.h>

class DrdyGpio {
  public:
    bool Open(const std::string &chip_path, unsigned line_offset)
    {
        chip_ = gpiod_chip_open(chip_path.c_str());
        if (!chip_) {
            std::cerr << "gpiod_chip_open(" << chip_path << ") failed: " << strerror(errno) << "\n";
            return false;
        }

        gpiod_line_settings *settings = gpiod_line_settings_new();
        if (!settings)
            return false;
        gpiod_line_settings_set_direction(settings, GPIOD_LINE_DIRECTION_INPUT);
        gpiod_line_settings_set_edge_detection(settings, GPIOD_LINE_EDGE_RISING);
        gpiod_line_settings_set_bias(settings, GPIOD_LINE_BIAS_PULL_UP);

        gpiod_line_config *line_cfg = gpiod_line_config_new();
        if (!line_cfg) {
            gpiod_line_settings_free(settings);
            return false;
        }

        unsigned offsets[1] = {line_offset};
        int rc = gpiod_line_config_add_line_settings(line_cfg, offsets, 1, settings);
        gpiod_line_settings_free(settings);
        if (rc < 0) {
            gpiod_line_config_free(line_cfg);
            return false;
        }

        gpiod_request_config *req_cfg = gpiod_request_config_new();
        if (!req_cfg) {
            gpiod_line_config_free(line_cfg);
            return false;
        }
        gpiod_request_config_set_consumer(req_cfg, "icm42688_drdy");

        request_ = gpiod_chip_request_lines(chip_, req_cfg, line_cfg);
        gpiod_request_config_free(req_cfg);
        gpiod_line_config_free(line_cfg);

        if (!request_) {
            std::cerr << "gpiod_chip_request_lines failed: " << strerror(errno) << "\n";
            return false;
        }

        evbuf_ = gpiod_edge_event_buffer_new(256);
        return evbuf_ != nullptr;
    }

    ~DrdyGpio()
    {
        if (evbuf_)
            gpiod_edge_event_buffer_free(evbuf_);
        if (request_)
            gpiod_line_request_release(request_);
        if (chip_)
            gpiod_chip_close(chip_);
    }

    bool WaitTs(int timeout_ms, int64_t &ts_ns_out)
    {
        int64_t timeout_ns = (timeout_ms < 0) ? -1 : (int64_t)timeout_ms * 1000000LL;
        int ret = gpiod_line_request_wait_edge_events(request_, timeout_ns);
        if (ret <= 0)
            return false;

        int n = gpiod_line_request_read_edge_events(request_, evbuf_, 1);
        if (n <= 0)
            return false;

        struct gpiod_edge_event *ev = gpiod_edge_event_buffer_get_event(evbuf_, 0);
        if (!ev)
            return false;

        ts_ns_out = (int64_t)gpiod_edge_event_get_timestamp_ns(ev);
        return true;
    }

  private:
    gpiod_chip *chip_{nullptr};
    gpiod_line_request *request_{nullptr};
    gpiod_edge_event_buffer *evbuf_{nullptr};
};
