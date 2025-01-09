#include <assert.h>
#include <cstdio>
#include <iostream>

#include <k4a/k4a.hpp>
#include <k4abt.hpp>
#include "Client.hpp"
#include "HandRaiseDetector.hpp"

using namespace std::chrono;

int main()
{
    try
    {
        k4a_device_configuration_t device_config = K4A_DEVICE_CONFIG_INIT_DISABLE_ALL;
        device_config.depth_mode = K4A_DEPTH_MODE_NFOV_UNBINNED;
        device_config.camera_fps = K4A_FRAMES_PER_SECOND_15;

        k4a::device device = k4a::device::open(0);
        device.start_cameras(&device_config);

        k4a::calibration sensor_calibration = device.get_calibration(device_config.depth_mode, device_config.color_resolution);


        //source camera  
        k4a_calibration_type_t camera = K4A_CALIBRATION_TYPE_DEPTH;

        k4abt::tracker tracker = k4abt::tracker::create(sensor_calibration);
        uint32_t to_follow_id = -1;
        chrono::microseconds m_handRaisedTimeSpan = std::chrono::microseconds::zero();
        chrono::microseconds m_previousTimestamp = std::chrono::microseconds::zero();
        HandRaiseDetector hrd;

        int clientSocket = startClient();
        do
        {
            k4a::capture sensor_capture;
            if (device.get_capture(&sensor_capture, std::chrono::milliseconds(K4A_WAIT_INFINITE)))
            {
                if (!tracker.enqueue_capture(sensor_capture))
                {
                    // It should never hit timeout when K4A_WAIT_INFINITE is set.
                    std::cout << "Error! Add capture to tracker process queue timeout!" << std::endl;
                    break;
                }
                k4abt::frame body_frame = tracker.pop_result();
                if (body_frame != nullptr) {
                    uint32_t num_bodies = body_frame.get_num_bodies();
                    for (uint32_t i = 0; i < num_bodies; i++)
                    {
                        k4abt_body_t body = body_frame.get_body(i);
                        hrd.updateBody(body);
                        if (hrd.toFollow(body, body_frame.get_device_timestamp())) {
                            to_follow_id = body.id;
                            std::cout << "Following Person ID: " << to_follow_id << std::endl;
                        }
                        if(to_follow_id == body.id){                         
                            k4a_float3_t ChestPos = body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position;
                            float message[5];
                            message[0] = ChestPos.xyz.x/1000.0;
                            message[1] = ChestPos.xyz.y/1000.0;
                            message[2] = ChestPos.xyz.z/1000.0;
                            float distance = hrd.checkFramePosition(body.skeleton.joints[K4ABT_JOINT_SPINE_CHEST].position, &sensor_calibration, camera);
                            message[3] = distance;
                            message[4] = hrd.leftOrRight;
                            if((distance == 50 && hrd.leftOrRight != 15) || distance != 50){
                                printf("Sending goal [%f,%f,%f] distance: %f, L or R: %i\n", ChestPos.xyz.x/1000.0, ChestPos.xyz.y/1000.0, ChestPos.xyz.z/1000.0, distance, hrd.leftOrRight);
                                send(clientSocket, (char *)message, 6 * sizeof(float), 0);
                            }
                        }
                    }
                }
                else {
                    //  It should never hit timeout when K4A_WAIT_INFINITE is set.
                    std::cout << "Error! Pop body frame result time out!" << std::endl;
                    break;
                }
            }
            else
            {
                // It should never hit time out when K4A_WAIT_INFINITE is set.
                std::cout << "Error! Get depth frame time out!" << std::endl;
                break;
            }
        } while (true);
    }
    catch (const std::exception& e)
    {
        std::cerr << "Failed with exception:" << std::endl
            << "    " << e.what() << std::endl;
        return 1;
    }

    return 0;
}