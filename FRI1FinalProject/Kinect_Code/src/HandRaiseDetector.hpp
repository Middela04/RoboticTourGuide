#ifndef HAND_RAISE_DETECTOR_H
#define HAND_RAISE_DETECTOR_H

#include <k4abttypes.h>
#include <chrono>
#include <unordered_map>
#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include <cmath>

using namespace std;

struct dataPos {
    // Translation components
    float tx, ty, tz;

    dataPos(float tx, float ty, float tz)
        : tx(tx), ty(ty), tz(tz) {}
};

class HandRaiseDetector
{
public:
    unordered_map<uint32_t, chrono::microseconds> b_time_map; // Static member variable
    bool toFollow(k4abt_body_t body, chrono::microseconds micro_time);
    void updateBody(k4abt_body_t body);
    void distanceData(const k4a_float3_t& jointPosition);
    float checkFramePosition(const k4a_float3_t& jointPosition, const k4a_calibration_t* calibration, k4a_calibration_type_t camera);
    //15 indicates that no changes to orientation are needed when the robot is close to the person.
    int leftOrRight = 15;
protected: 
    bool isHandRaised(k4abt_body_t body);
    //do i need to use this instead k4a_calibration_extrinsics_t ->tranlation
    //const geometry_msgs::PoseStamped::ConstPtr& msg;
    //const k4a_quaternion_t::_wxyz rot;
};

#endif
