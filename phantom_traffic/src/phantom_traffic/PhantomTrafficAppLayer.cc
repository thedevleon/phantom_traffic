#include "PhantomTrafficAppLayer.h"

#include "phantom_traffic/ApplicationLayerTestMessage_m.h"
#include "phantom_traffic/PhantomTrafficMessage_m.h"

using namespace veins;

namespace phantom_traffic {

Define_Module(PhantomTrafficAppLayer);

void PhantomTrafficAppLayer::initialize(int stage)
{
    PhantomTrafficBaseAppLayer::initialize(stage);
    if (stage == 0) {
        sentMessage = false;
        lastDroveAt = simTime();
        currentSubscribedServiceId = -1;
    }
}

void PhantomTrafficAppLayer::onPTM(PhantomTrafficMessage* ptm)
{
    //store new beacon here
    beaconData.emplace_back(ptm->getSenderTime(),
                            ptm->getSenderSpeed(),
                            ptm->getSenderPos(),
                            ptm->getSenderAccel(),
                            ptm->getSenderLane()
    );


    for(int j = 0; j < cxsize; j++){
        auto item = ptm->getPtmItems(j);

        if(item.id != -1){
            bool update = false;

            for(int i = 0; i < ptmItems.size(); i++){
                if(item.id == ptmItems[i].id)
                {
                    update = true;
                    ptmItems[i].cs = item.cs;
                    ptmItems[i].ct = item.ct;
                    ptmItems[i].cl = item.cl;
                    break;
                }
            }

            if(!update)
            {
                ptmItems.emplace_back(item.id, item.cs, item.ct, item.cl);
            }
        }
    }

    //discard old beacons here
    //Store - basic info for 1s - cs, cl and ct for 30s
    for(int i = 0; i < beaconData.size(); i++) {
        if(simTime().dbl() - beaconData[i].time > beacon_time) {
            beaconData.erase(beaconData.begin() + i--);
        }
    }

    //Discard old ptmItems
    for(int i = 0; i < ptmItems.size(); i++) {
        if(simTime().dbl() - ptmItems[i].ct > c_time) {
            ptmItems.erase(ptmItems.begin() + i--);
        }
    }
    
    //After each beacon received:
    //calculate the average speed (v_a(t)) of vehicles ahead of itself (in the same lane) using the last 1s of beacon information
    double v_a = 0;
    int count = 0;
    for(auto curBeacon : beaconData) {
        if(curBeacon.lane == traciVehicle->getLaneIndex()) {
            double distance = traci->getDistance(mobility->getPositionAt(simTime()), curBeacon.position, true);
            if(distance > 0) {
                v_a = v_a + curBeacon.speed;
                count++;
            }
        }
    }
    if(count > 0) v_a = v_a / count; //now we have average speed of predecessor cars

    vCount.record(count);
    avgSpeed.record(v_a);

    //if v_a(t) < 81 km/h -> store c_t <- time and c_s <- currentPosition (+ communication range (150m))
    //start adding these to the beacon
    if(v_a < v_a_threshold && count > 0) {
        aboveThreshold.record(1);
        //To make sure we dont store the same congestion to many many times we first
        //check each current c-vector - if cs is within 5m of the potentially new cs and it's 
        //in the same lane, just update the ct (the congestion is still present)

        bool update = false;
        for(int i = 0; i < ptmItems.size(); i++) {
            double distance = traci->getDistance(mobility->getPositionAt(simTime()), ptmItems[i].cs, true);
            if(distance < update_range && ptmItems[i].cl == traciVehicle->getLaneIndex())
            {
                ptmItems[i].cs = mobility->getPositionAt(simTime());
                ptmItems[i].ct = simTime().dbl();
                update = true;
                updateCsCt.record(true);
            }
        }

        if(!update) {
            //create new ptmItem
            auto randId = intrand(1000);
            ptmItems.emplace_back(randId, mobility->getPositionAt(simTime()), simTime().dbl(), traciVehicle->getLaneIndex());
            updateCsCt.record(false);
            newCsCt.record(true);
        }

    }
    else
    {
        aboveThreshold.record(0);
    }
    
    drivingChange = false;

    //if 0 < c_s - currentPos < 3km set B (change driving behaviour) to true, only for cars in the same lane
    for(int i = 0; i < ptmItems.size(); i++) {
        if(ptmItems[i].cl == traciVehicle->getLaneIndex()) {
            if(0 < traci->getDistance(mobility->getPositionAt(simTime()), ptmItems[i].cs, true) && traci->getDistance(mobility->getPositionAt(simTime()), ptmItems[i].cs, true) < 3000) {
                drivingChange = true;
               break;
            }
        }
    }

    bdSize.record(beaconData.size());
    ptmItemsSize.record(ptmItems.size());
}


void PhantomTrafficAppLayer::onWSA(DemoServiceAdvertisment* wsa)
{
    /*
    if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(static_cast<Channel>(wsa->getTargetChannel()));
        currentSubscribedServiceId = wsa->getPsid();
        if (currentOfferedServiceId != wsa->getPsid()) {
            stopService();
            startService(static_cast<Channel>(wsa->getTargetChannel()), wsa->getPsid(), "Mirrored Traffic Service"); //This will start a "service" that mirrors the WSA it just received and sending it into the networking, triggering the service on other cars as well, flooding the network.
        }
    }
    */
}

void PhantomTrafficAppLayer::onWSM(BaseFrame1609_4* frame)
{
    /*
    ApplicationLayerTestMessage* wsm = check_and_cast<ApplicationLayerTestMessage*>(frame);

    findHost()->getDisplayString().setTagArg("i", 1, "green");

    if (mobility->getRoadId()[0] != ':') traciVehicle->changeRoute(wsm->getDemoData(), 9999);
    if (!sentMessage) {
        sentMessage = true;
        // repeat the received traffic update once in 2 seconds plus some random delay
        wsm->setSenderAddress(myId);
        wsm->setSerial(3);
        scheduleAt(simTime() + 2 + uniform(0.01, 0.2), wsm->dup());
    }
    */
}

void PhantomTrafficAppLayer::handleSelfMsg(cMessage* msg)
{
    /*
    if (ApplicationLayerTestMessage* wsm = dynamic_cast<ApplicationLayerTestMessage*>(msg)) {
        // send this message on the service channel until the counter is 3 or higher.
        // this code only runs when channel switching is enabled
        sendDown(wsm->dup());
        wsm->setSerial(wsm->getSerial() + 1);
        if (wsm->getSerial() >= 3) {
            // stop service advertisements
            stopService();
            delete (wsm);
        }
        else {
            scheduleAt(simTime() + 1, wsm);
        }
    }
    else {
    */
        PhantomTrafficBaseAppLayer::handleSelfMsg(msg);
    //}
}


//IMPORTANT: This method is called every step of the simulation by TraCI to inform the application layer of the vehicles position
void PhantomTrafficAppLayer::handlePositionUpdate(cObject* obj)
{
    PhantomTrafficBaseAppLayer::handlePositionUpdate(obj);

    //While b = true:
    if(drivingChange) {
        drvChange.record(1);
        bool stopAccel = true;
        for(int i = 0; i < beaconData.size(); i++) {
            //from all beaconData check which cars are in front of you. Meaning is the distance positive and on the same lane.
            if(beaconData[i].lane == traciVehicle->getLaneIndex() && (traci->getDistance(mobility->getPositionAt(simTime()), beaconData[i].position, true)) > 0) {
                double cur_gap = traci->getDistance(mobility->getPositionAt(simTime()), beaconData[i].position, true);
                double gap_n = beaconData[i].speed * seconds_gap + 0.5 * pow(beaconData[i].acceleration, seconds_gap);
                if(cur_gap < gap_n) {
                    stopAccel = false;
                }
            }
        }

        //Don't accelerate until you have a big enough gap (gap_n) between itself and next car. (distance predecessor travels in 2s)
        //To calculate gap_n (all in m/s): curSpeedPred * 2 + 0.5 * (curAccelPred ^ 2)
        //curPositionPred - curPosition (cur_gap) > gap_n
        //While cur_gap < gap_n (<= 3km) change the car's "decel" value from 4.5 to something higher (to simulate unnecessairily strong breaks)
        //While gap_n < cur_gap (<= 3km) change the car's "decel" value back to 4.5
        if(stopAccel) {
            stopAcc.record(0);
            traciVehicle->setParameter("accel", "0");
            traciVehicle->setParameter("decel", "9");
            traciVehicle->setColor(driveChangedColor);
        }
        else {
            stopAcc.record(1);
            traciVehicle->setParameter("accel", "2.5");
            traciVehicle->setParameter("decel", "4.5");
            traciVehicle->setColor(normalColor);
        }
    }
    else {
        drvChange.record(0);
        traciVehicle->setParameter("accel", "2.5");
        traciVehicle->setParameter("decel", "4.5");
        traciVehicle->setColor(normalColor);
    }
}

}
