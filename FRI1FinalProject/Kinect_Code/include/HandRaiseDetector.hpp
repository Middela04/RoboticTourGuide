#ifndef HAND_RAISE_DETECTOR_H
#define HAND_RAISE_DETECTOR_H

#include <k4abttypes.h>
#include <chrono>
#include <unordered_map>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <cmath>
#include <geometry_msgs/PoseStamped.h>



using namespace std;

struct dataPos {
    // Translation components
    float tx, ty, tz, tq;

    // Quaternion components for rotation
    //float qx, qy, qz, qw;

    // Constructor to initialize the Pose
    //qx(qx), qy(qy), qz(qz), qw(qw)
    dataPos(float tx, float ty, float tz, float tq)
        : tx(tx), ty(ty), tz(tz), tq(tq) {}
};

class HandRaiseDetector
{
public:
    unordered_map<uint32_t, std::chrono::microseconds> b_time_map; // Static member variable
    bool toFollow(k4abt_body_t body, chrono::microseconds micro_time);
    void updateBody(k4abt_body_t body);
    void distanceData(const k4a_float3_t& jointPosition);
    void checkFramePosition(const k4a_float3_t& jointPosition, const k4a_calibration_t* calibration, k4a_calibration_type_t camera);


protected: 
    bool isHandRaised(k4abt_body_t body);
    //do i need to use this instead k4a_calibration_extrinsics_t ->tranlation
    //const geometry_msgs::PoseStamped::ConstPtr& msg;
    //const k4a_quaternion_t::_wxyz rot;
};

#endif
