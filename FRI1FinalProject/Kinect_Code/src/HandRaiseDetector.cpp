#include "HandRaiseDetector.hpp"
#include <iostream>
#include <cmath>


bool HandRaiseDetector::isHandRaised(k4abt_body_t body) {
    k4a_float3_t leftWristJoint = body.skeleton.joints[K4ABT_JOINT_WRIST_LEFT].position;
    k4a_float3_t rightWristJoint = body.skeleton.joints[K4ABT_JOINT_WRIST_RIGHT].position;
    k4a_float3_t neckJoint = body.skeleton.joints[K4ABT_JOINT_NECK].position;
    // Notice: y direction is pointing towards the ground! So jointA.y < jointB.y means jointA is higher than jointB
    bool handRaised = leftWristJoint.xyz.y < neckJoint.xyz.y ||
        rightWristJoint.xyz.y < neckJoint.xyz.y;
    return handRaised;
}
//Pre-condition: body is already inside b_time_map
bool HandRaiseDetector::toFollow(k4abt_body_t body, chrono::microseconds micro_time) {
    if (isHandRaised(body)) {
        if (b_time_map[body.id] == chrono::microseconds::zero()) {
            //set the current time
            b_time_map[body.id] = micro_time;
            return false;
        }
        else {
            chrono::microseconds diff = micro_time - b_time_map[body.id];
            chrono::microseconds threeSec(3000000);
            chrono::seconds diffSecs = chrono::duration_cast<chrono::seconds>(diff);
            printf("Time hand has been raised: %lu\n", diffSecs.count());
            if(diff > threeSec){
                b_time_map[body.id] = chrono::microseconds::zero();
                return true;
            }
            else {return false;}
        }
    }
    else {
        b_time_map[body.id] = chrono::microseconds::zero();
    }
    return false;
}

void HandRaiseDetector::updateBody(k4abt_body_t body) {
    if (b_time_map.find(body.id) == b_time_map.end()) {
        b_time_map[body.id] = chrono::microseconds::zero();
    }
}

float HandRaiseDetector::checkFramePosition(const k4a_float3_t& jointPosition, const k4a_calibration_t* calibration, k4a_calibration_type_t camera) {
    const int frameWidth = calibration->depth_camera_calibration.resolution_width;
    //3d to 2d
    k4a_float2_t tp2d;
    k4a_float2_t* target_point_2d = &tp2d;
    int* valid = new int;
    k4a_calibration_3d_to_2d(calibration, &jointPosition, camera, camera, target_point_2d, valid);
   
    float edgeThreshold = frameWidth * 0.325f;
    if (target_point_2d->xy.x < edgeThreshold) {
        //left
        leftOrRight = 0;
    }
    else if (target_point_2d->xy.x > frameWidth - edgeThreshold) {
        //right
        leftOrRight = 1;
    }
    else {
        leftOrRight = 15;
    }
    float distance = sqrt(((jointPosition.xyz.x/1000.0) * (jointPosition.xyz.x/1000.0) +
        (jointPosition.xyz.y/1000.0) * (jointPosition.xyz.y/1000.0) +
        (jointPosition.xyz.z/1000.0) * (jointPosition.xyz.z/1000.0)));
    //printf("Distance from person: %f\n", distance);
    //50 indicates that the robot is close enough and should be in orientation fixing mode only now. 
    return distance;

}
