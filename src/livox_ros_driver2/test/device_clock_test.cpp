#include "comm/device_clock.h"
#include <cassert>
int main() {
  livox_ros::DeviceClock clock;
  const uint64_t epoch=1700000000000000000ULL,boot=10000000000ULL;
  assert(clock.Map(boot,epoch)==epoch);
  // A batched IMU callback must keep its 5 ms acquisition interval.
  assert(clock.Map(boot+5000000,epoch+22000000)==epoch+5000000);
  assert(clock.Map(boot+10000000,epoch+22001000)==epoch+10000000);
  // The interleaved LiDAR packet shares the same epoch, even out of order.
  assert(clock.Map(boot+8000000,epoch+23000000)==epoch+8000000);
  // Reboot resets the epoch, without interpreting older packets as huge times.
  assert(clock.Map(1000000,epoch+100000000)==epoch+100000000);
  assert(clock.Map(6000000,epoch+140000000)==epoch+105000000);
  livox_ros::DeviceClock other;
  assert(other.Map(900000000,epoch+1)==epoch+1);
}
