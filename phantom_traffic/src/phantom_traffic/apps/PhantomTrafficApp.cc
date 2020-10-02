#include "PhantomTrafficApp.h"

#include "../protocols/PhantomTrafficProtocol.h"
#include "veins/modules/messages/BaseFrame1609_4_m.h"
#include "veins/base/messages/MacPkt_m.h"
#include "veins/modules/mac/ieee80211p/Mac1609_4.h"
#include "veins/base/utils/FindModule.h"

#include "plexe/PlexeManager.h"

using namespace veins;

namespace phantom_traffic {

Define_Module(PhantomTrafficApp);

void PhantomTrafficApp::initialize(int stage)
{

    BaseApplLayer::initialize(stage);

    if (stage == 0) {
        // set names for output vectors
        // distance from front vehicle
        distanceOut.setName("distance");
        // relative speed w.r.t. front vehicle
        relSpeedOut.setName("relativeSpeed");
        // vehicle id
        nodeIdOut.setName("nodeId");
        // current speed
        speedOut.setName("speed");
        // vehicle position
        posxOut.setName("posx");
        posyOut.setName("posy");
        // vehicle acceleration
        accelerationOut.setName("acceleration");
        controllerAccelerationOut.setName("controllerAcceleration");
    }

    if (stage == 1) {
        mobility = veins::TraCIMobilityAccess().get(getParentModule());
        traci = mobility->getCommandInterface();
        traciVehicle = mobility->getVehicleCommandInterface();
        auto plexe = FindModule<plexe::PlexeManager*>::findGlobalModule();
        ASSERT(plexe);
        plexeTraci = plexe->getCommandInterface();
        plexeTraciVehicle.reset(new plexe::traci::CommandInterface::Vehicle(plexeTraci, mobility->getExternalId()));
        positionHelper = FindModule<plexe::BasePositionHelper*>::findSubModule(getParentModule());
        protocol = FindModule<PhantomTrafficProtocol*>::findSubModule(getParentModule());
        myId = positionHelper->getId();

        // connect application to protocol
        protocol->registerApplication(PhantomTrafficProtocol::BEACON_TYPE, gate("lowerLayerIn"), gate("lowerLayerOut"), gate("lowerControlIn"), gate("lowerControlOut"));

        recordData = new cMessage("recordData");
        // init statistics collection. round to 0.1 seconds
        SimTime rounded = SimTime(floor(simTime().dbl() * 1000 + 100), SIMTIME_MS);
        scheduleAt(rounded, recordData);
    }
}

PhantomTrafficApp::~PhantomTrafficApp()
{
    cancelAndDelete(recordData);
    recordData = nullptr;
    cancelAndDelete(stopSimulation);
    stopSimulation = nullptr;
}

void PhantomTrafficApp::handleLowerMsg(cMessage* msg)
{
    BaseFrame1609_4* unicast = check_and_cast<BaseFrame1609_4*>(msg);

    cPacket* enc = unicast->decapsulate();
    ASSERT2(enc, "received a UnicastMessage with nothing inside");

    if (enc->getKind() == PhantomTrafficProtocol::BEACON_TYPE) {
        onPhantomTrafficBeacon(check_and_cast<PhantomTrafficBeacon*>(enc));
    }
    else {
        error("received unknown message type");
    }

    delete unicast;
}

void PhantomTrafficApp::logVehicleData(bool crashed)
{
    // get distance and relative speed w.r.t. front vehicle
    double distance, relSpeed;
    plexe::VEHICLE_DATA data;
    plexeTraciVehicle->getRadarMeasurements(distance, relSpeed);
    plexeTraciVehicle->getVehicleData(&data);
    if (crashed) {
        distance = 0;
        stopSimulation = new cMessage("stopSimulation");
        scheduleAt(simTime() + SimTime(1, SIMTIME_MS), stopSimulation);
    }
    // write data to output files
    distanceOut.record(distance);
    relSpeedOut.record(relSpeed);
    nodeIdOut.record(myId);
    accelerationOut.record(data.acceleration);
    controllerAccelerationOut.record(data.u);
    speedOut.record(data.speed);
    posxOut.record(data.positionX);
    posyOut.record(data.positionY);
}

void PhantomTrafficApp::handleLowerControl(cMessage* msg)
{
    delete msg;
}

void PhantomTrafficApp::sendUnicast(cPacket* msg, int destination)
{
    BaseFrame1609_4* unicast = new BaseFrame1609_4();
    unicast->setRecipientAddress(destination);
    unicast->encapsulate(msg);
    sendDown(unicast);
}

void PhantomTrafficApp::handleSelfMsg(cMessage* msg)
{
    if (msg == recordData) {
        // log mobility data
        logVehicleData(plexeTraciVehicle->isCrashed());
        // re-schedule next event
        scheduleAt(simTime() + SimTime(100, SIMTIME_MS), recordData);
    }
    if (msg == stopSimulation) {
        endSimulation();
    }
}

void PhantomTrafficApp::onPhantomTrafficBeacon(const PhantomTrafficBeacon* pb)
{
    //TODO put our algorithm here

    /*
    if (positionHelper->isInSamePlatoon(pb->getVehicleId())) {
        // if the message comes from the leader
        if (pb->getVehicleId() == positionHelper->getLeaderId()) {
            plexeTraciVehicle->setLeaderVehicleData(pb->getControllerAcceleration(), pb->getAcceleration(), pb->getSpeed(), pb->getPositionX(), pb->getPositionY(), pb->getTime());
        }
        // if the message comes from the vehicle in front
        if (pb->getVehicleId() == positionHelper->getFrontId()) {
            plexeTraciVehicle->setFrontVehicleData(pb->getControllerAcceleration(), pb->getAcceleration(), pb->getSpeed(), pb->getPositionX(), pb->getPositionY(), pb->getTime());
        }
        // send data about every vehicle to the CACC. this is needed by the consensus controller
        struct VEHICLE_DATA vehicleData;
        vehicleData.index = positionHelper->getMemberPosition(pb->getVehicleId());
        vehicleData.acceleration = pb->getAcceleration();
        vehicleData.length = pb->getLength();
        vehicleData.positionX = pb->getPositionX();
        vehicleData.positionY = pb->getPositionY();
        vehicleData.speed = pb->getSpeed();
        vehicleData.time = pb->getTime();
        vehicleData.u = pb->getControllerAcceleration();
        vehicleData.speedX = pb->getSpeedX();
        vehicleData.speedY = pb->getSpeedY();
        vehicleData.angle = pb->getAngle();
        // send information to CACC
        plexeTraciVehicle->setVehicleData(&vehicleData);
    }
    */
    delete pb;
}

} // namespace phantom_traffic
