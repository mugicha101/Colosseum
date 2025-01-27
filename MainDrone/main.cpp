// Copyright (c) Microsoft Corporation. All rights reserved.
// Licensed under the MIT License.

#include "common/common_utils/StrictMode.hpp"
STRICT_MODE_OFF
#ifndef RPCLIB_MSGPACK
#define RPCLIB_MSGPACK clmdep_msgpack
#endif // !RPCLIB_MSGPACK
#include "rpc/rpc_error.h"
STRICT_MODE_ON

#include "vehicles/multirotor/api/MultirotorRpcLibClient.hpp"
#include "common/common_utils/FileSystem.hpp"
#include <iostream>
#include <chrono>
#include <tuple>
#include <utility>
#include <random>

int main()
{
    using namespace msr::airlib;

    msr::airlib::MultirotorRpcLibClient client;
    typedef ImageCaptureBase::ImageRequest ImageRequest;
    typedef ImageCaptureBase::ImageResponse ImageResponse;
    typedef ImageCaptureBase::ImageType ImageType;
    typedef common_utils::FileSystem FileSystem;

    try {
        client.confirmConnection();
        srand((unsigned int)time(0));

        auto randf = [](float lo = 0.f, float hi = 1.f) {
            constexpr float M = 1.f / (float)RAND_MAX;
            return lo + (float)rand() * M * (hi - lo);
        };

        std::cout << "Press Enter to arm the drone" << std::endl;
        std::cin.get();

        client.enableApiControl(true);
        client.armDisarm(true);

        auto barometer_data = client.getBarometerData();
        std::cout << "Barometer data \n"
                  << "barometer_data.time_stamp \t" << barometer_data.time_stamp << std::endl
                  << "barometer_data.altitude \t" << barometer_data.altitude << std::endl
                  << "barometer_data.pressure \t" << barometer_data.pressure << std::endl
                  << "barometer_data.qnh \t" << barometer_data.qnh << std::endl;

        auto imu_data = client.getImuData();
        std::cout << "IMU data \n"
                  << "imu_data.time_stamp \t" << imu_data.time_stamp << std::endl
                  << "imu_data.orientation \t" << imu_data.orientation << std::endl
                  << "imu_data.angular_velocity \t" << imu_data.angular_velocity << std::endl
                  << "imu_data.linear_acceleration \t" << imu_data.linear_acceleration << std::endl;

        auto gps_data = client.getGpsData();
        std::cout << "GPS data \n"
                  << "gps_data.time_stamp \t" << gps_data.time_stamp << std::endl
                  << "gps_data.gnss.time_utc \t" << gps_data.gnss.time_utc << std::endl
                  << "gps_data.gnss.geo_point \t" << gps_data.gnss.geo_point << std::endl
                  << "gps_data.gnss.eph \t" << gps_data.gnss.eph << std::endl
                  << "gps_data.gnss.epv \t" << gps_data.gnss.epv << std::endl
                  << "gps_data.gnss.velocity \t" << gps_data.gnss.velocity << std::endl
                  << "gps_data.gnss.fix_type \t" << gps_data.gnss.fix_type << std::endl;

        auto magnetometer_data = client.getMagnetometerData();
        std::cout << "Magnetometer data \n"
                  << "magnetometer_data.time_stamp \t" << magnetometer_data.time_stamp << std::endl
                  << "magnetometer_data.magnetic_field_body \t" << magnetometer_data.magnetic_field_body << std::endl;
        // << "magnetometer_data.magnetic_field_covariance" << magnetometer_data.magnetic_field_covariance // not implemented in sensor

        std::cout << "Takeoff" << std::endl;
        float takeoff_timeout = 3;
        client.takeoffAsync(takeoff_timeout)->waitOnLastTask();

        // switch to explicit hover mode so that this is the fall back when
        // move* commands are finished.
        std::this_thread::sleep_for(std::chrono::duration<double>(5));
        client.hoverAsync()->waitOnLastTask();

        std::vector<msr::airlib::Vector3r> points(100);
        for (int i = 0; i < (int)points.size(); ++i) {
            auto& p = points[i];
            p.x() = randf(-20.f, 20.f);
            p.y() = randf(-20.f, 20.f);
            p.z() = randf(-20.f, -10.f);
        }

        client.enableApiControl(true);

        auto origin = client.getMultirotorState().getPosition();
        // float z = position.z(); // current position (NED coordinate system).
        constexpr float speed = 15.0f;
        // constexpr float size = 10.0f;
        // constexpr float duration = size / speed;
        // DrivetrainType drivetrain = DrivetrainType::MaxDegreeOfFreedom;
        // YawMode yaw_mode(true, 0);

        // move to absolute position
        auto moveTo = [&client](float x, float y, float z, float speed) {
            return client.moveToPositionAsync(x, y, z, speed); 
        };

        for (auto& p : points) {
            std::cout << "Move to " << origin.x() + p.x() << ", " << origin.y() + p.y() << ", " << origin.z() + p.z() << std::endl;
            moveTo(origin.x() + p.x(), origin.y() + p.y(), origin.z() + p.z(), speed)->waitOnLastTask();
            auto pos = client.getMultirotorState().getPosition();
            std::cout << "At " << pos.x() << ", " << pos.y() << ", " << pos.z() << std::endl;
        }
        std::cout << "Move to origin" << std::endl;
        moveTo(origin.x(), origin.y(), origin.z(), speed * 0.25f)->waitOnLastTask();

        std::cout << "Landing" << std::endl;
        client.landAsync()->waitOnLastTask();
        client.armDisarm(false);
    }
    catch (rpc::rpc_error& e) {
        const auto msg = e.get_error().as<std::string>();
        std::cout << "Exception raised by the API, something went wrong." << std::endl
                  << msg << std::endl;
    }

    return 0;
}
