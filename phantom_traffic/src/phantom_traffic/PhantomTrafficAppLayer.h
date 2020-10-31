#pragma once

#include "base/PhantomTrafficBaseAppLayer.h"
#include "phantom_traffic/phantom_traffic.h"

#include <vector>

namespace phantom_traffic {

class PHANTOM_TRAFFIC_API PhantomTrafficAppLayer : public PhantomTrafficBaseAppLayer {
public:
    void initialize(int stage) override;

protected:
    simtime_t lastDroveAt;
    bool sentMessage;
    int currentSubscribedServiceId;
    bool drivingChange = false;
    
    //Research parameters
    double seconds_gap = 10;             //keep a 2/3s gap with predecesor
    double v_a_threshold = 81 / 3.6;    //average speed the cars need to drop below for the system to activate (22.5 m/s)
    double beacon_time = 1;             //time interval used to calculate v_a (also duration beacons get stored)
    double c_time = 10;                 //time interval in which c-values are considered relevant (also duration c-values get stored)
    double update_range = 50;           //Range in which a new cs/ct should be considered the same
    double forward_range = 500;
    double congestion_range = 1000;

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

    const veins::TraCIColor thresholdColor = veins::TraCIColor(0,255,255,255);
    const veins::TraCIColor driveChangedColor = veins::TraCIColor(182,44,217,255);
    const veins::TraCIColor normalColor = veins::TraCIColor(255,140,0,255);

protected:
    void onPTM(PhantomTrafficMessage* ptm) override;
    void onWSM(veins::BaseFrame1609_4* wsm) override;
    void onWSA(veins::DemoServiceAdvertisment* wsa) override;

    void handleSelfMsg(cMessage* msg) override;
    void handlePositionUpdate(cObject* obj) override;
};

} // namespace phantom_traffic
