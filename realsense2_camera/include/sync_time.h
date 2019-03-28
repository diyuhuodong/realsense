#ifndef REALSENSE_TIME_SYNCHRONIZER_H_
#define REALSENSE_TIME_SYNCHRONIZER_H_

#include <map>
#include <mutex>

#include <Eigen/Core>
//#include <glog/logging.h>

namespace realsense2_camera {

class ImuSynchronizer {
 public:
  struct ImuData {
    double timestamp;
    Eigen::Vector3d angular_velocity;
    Eigen::Vector3d acceleration;
  };

  void registerGyroMeasurement(
      const double timestamp, const Eigen::Vector3d& angular_velocity,
      std::vector<ImuData>* synced_imu) {
    //CHECK_NOTNULL(synced_imu)->clear();
      if (synced_imu != NULL){
        synced_imu->clear();
      }

    std::lock_guard<std::mutex> lock(data_mutex_);
    //CHECK_GT(timestamp, last_angular_velocity_.first);
    /*if (timestamp <= last_angular_velocity_.first){
      return;
    }*/
    

    const double delta_gyro = timestamp - last_angular_velocity_.first;

    // Process all accelerometer measurements that happened in the meantime.
    for (const ImuMeasurement& acc_meas : acc_measurements_since_last_gyro_) {
      if (acc_meas.first < last_angular_velocity_.first ||
          acc_meas.first > timestamp) {
        // Timestamp out of range, continue.
        continue;
      }

      const double delta_t_interp =
          (acc_meas.first - last_angular_velocity_.first) / delta_gyro;

      ImuData imu_data;
      imu_data.timestamp = acc_meas.first;
      imu_data.acceleration = acc_meas.second;

      // Interpolate the gyro.
      imu_data.angular_velocity =
          last_angular_velocity_.second * (1 - delta_t_interp) +
          angular_velocity * delta_t_interp;
      synced_imu->push_back(imu_data);
    }
    acc_measurements_since_last_gyro_.clear();

    last_angular_velocity_.first = timestamp;
    last_angular_velocity_.second = angular_velocity;
  }

  void registerAccelerometerMeasurement(
      const double timestamp, const Eigen::Vector3d& data) {
    std::lock_guard<std::mutex> lock(data_mutex_);
    //CHECK_GT(timestamp, last_acc_timestamp_);
    /*if (timestamp <= last_acc_timestamp_){
      return;
    }*/

    acc_measurements_since_last_gyro_.emplace_back(timestamp, data);

    last_acc_timestamp_ = timestamp;
  }

 private:
  double last_gyro_timestamp_;
  double last_acc_timestamp_;

  typedef std::pair<double, Eigen::Vector3d> ImuMeasurement;
  std::vector<ImuMeasurement> acc_measurements_since_last_gyro_;
  ImuMeasurement last_angular_velocity_;

  std::mutex data_mutex_;
};

}  // namespace maplab_realsense

#endif  // REALSENSE_TIME_SYNCHRONIZER_H_
