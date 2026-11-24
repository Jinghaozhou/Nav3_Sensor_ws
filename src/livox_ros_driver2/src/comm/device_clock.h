#pragma once
#include <algorithm>
#include <cstdint>

namespace livox_ros {
// Type-0 timestamps are nanoseconds since sensor power-on. Anchor the shared
// LiDAR/IMU clock once, retaining intervals despite SDK delivery bursts.
class DeviceClock {
 public:
  uint64_t Map(uint64_t device_ns, uint64_t host_ns) {
    if (!initialized_ || (latest_device_ns_ > device_ns && latest_device_ns_ - device_ns > 1000000000ULL)) {
      device_anchor_ = device_ns;
      host_anchor_ = host_ns;
      latest_device_ns_ = device_ns;
      initialized_ = true;
    }
    latest_device_ns_ = std::max(latest_device_ns_, device_ns);
    return static_cast<uint64_t>(static_cast<int64_t>(host_anchor_) +
        static_cast<int64_t>(device_ns) - static_cast<int64_t>(device_anchor_));
  }
 private:
  bool initialized_{false};
  uint64_t device_anchor_{0}, host_anchor_{0}, latest_device_ns_{0};
};
}
