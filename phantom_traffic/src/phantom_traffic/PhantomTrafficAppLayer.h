#pragma once

#include "base/PhantomTrafficBaseAppLayer.h"
#include "phantom_traffic/phantom_traffic.h"

#include <vector>

namespace phantom_traffic {

/**
 * @brief
 * A tutorial demo for TraCI. When the car is stopped for longer than 10 seconds
 * it will send a message out to other cars containing the blocked road id.
 * Receiving cars will then trigger a reroute via TraCI.
 * When channel switching between SCH and CCH is enabled on the MAC, the message is
 * instead send out on a service channel following a Service Advertisement
 * on the CCH.
 *
 * @author Christoph Sommer : initial DemoApp
 * @author David Eckhoff : rewriting, moving functionality to DemoBaseApplLayer, adding WSA
 *
 */

class PHANTOM_TRAFFIC_API PhantomTrafficAppLayer : public PhantomTrafficBaseAppLayer {
public:
    void initialize(int stage) override;

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;
    bool drivingChange = false;
    
    //Research parameters
    double seconds_gap = 2;             //keep a 2/3s gap with predecesor
    double v_a_threshold = 81 / 3.6;    //average speed the cars need to drop below for the system to activate (22.5 m/s)
    double beacon_time = 1;             //time interval used to calculate v_a (also duration beacons get stored)
    double c_time = 30;                 //time interval in which c-values are considered relevant (also duration c-values get stored)

    struct SBeaconData{
        double time;
        double speed;
        Coord position;
        double acceleration;
        int lane;

        SBeaconData(
                    double s_time,
                    double s_speed,
                    Coord s_position,
                    double s_acceleration,
                    double s_lane)
                    :time(s_time),
                    speed(s_speed),
                    position(s_position),
                    acceleration(s_acceleration),
                    lane(s_lane)
                {}
    
    };
    
    std::vector<struct SBeaconData>beaconData;

protected:
    void onPTM(PhantomTrafficMessage* ptm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};

} // namespace phantom_traffic
