#include "PhantomTrafficBaseAppLayer.h"

using namespace veins;

namespace phantom_traffic {

void PhantomTrafficBaseAppLayer::initialize(int stage)
{
    BaseApplLayer::initialize(stage);

    if (stage == 0) {

        // initialize pointers to other modules
        if (FindModule<TraCIMobility*>::findSubModule(getParentModule())) {
            mobility = TraCIMobilityAccess().get(getParentModule());
            traci = mobility->getCommandInterface();
            traciVehicle = mobility->getVehicleCommandInterface();
        }
        else {
            traci = nullptr;
            mobility = nullptr;
            traciVehicle = nullptr;
        }

        annotations = AnnotationManagerAccess().getIfExists();
        ASSERT(annotations);

        mac = FindModule<BaseApplLayerToMac1609_4Interface*>::findSubModule(getParentModule());
        ASSERT(mac);

        // read parameters
        headerLength = par("headerLength");
        sendBeacons = par("sendBeacons").boolValue();
        beaconLengthBits = par("beaconLengthBits");
        beaconUserPriority = par("beaconUserPriority");
        beaconInterval = par("beaconInterval");

        dataLengthBits = par("dataLengthBits");
        dataOnSch = par("dataOnSch").boolValue();
        dataUserPriority = par("dataUserPriority");

        wsaInterval = par("wsaInterval").doubleValue();
        currentOfferedServiceId = -1;

        isParked = false;

        findHost()->subscribe(BaseMobility::mobilityStateChangedSignal, this);
        findHost()->subscribe(TraCIMobility::parkingStateChangedSignal, this);

        sendBeaconEvt = new cMessage("beacon evt", SEND_BEACON_EVT);
        sendWSAEvt = new cMessage("wsa evt", SEND_WSA_EVT);

        generatedBSMs = 0;
        generatedWSAs = 0;
        generatedWSMs = 0;
        generatedPTMs = 0;
        receivedBSMs = 0;
        receivedWSAs = 0;
        receivedWSMs = 0;
        receivedPTMs = 0;

        //    cOutVector stopAcc, drvChange, bdSize, csSize, clSize, ctSize;
        stopAcc.setName("stopAcceleration");
        drvChange.setName("driveChange");
        bdSize.setName("beaconSize");
        csSize.setName("csSize");
        clSize.setName("clSize");
        ctSize.setName("ctSize");
        aboveThreshold.setName("aboveThreshold");
        vCount.setName("vehicleCount");
        avgSpeed.setName("averageSpeed");
    }
    else if (stage == 1) {

        // store MAC address for quick access
        myId = mac->getMACAddress();

        // simulate asynchronous channel access

        if (dataOnSch == true && !mac->isChannelSwitchingActive()) {
            dataOnSch = false;
            EV_ERROR << "App wants to send data on SCH but MAC doesn't use any SCH. Sending all data on CCH" << std::endl;
        }
        simtime_t firstBeacon = simTime();

        if (par("avoidBeaconSynchronization").boolValue() == true) {

            simtime_t randomOffset = dblrand() *  beaconInterval;
            firstBeacon = simTime() + randomOffset;

            if (mac->isChannelSwitchingActive() == true) {
                if (beaconInterval.raw() % (mac->getSwitchingInterval().raw() * 2)) {
                    EV_ERROR << "The beacon interval (" << beaconInterval << ") is smaller than or not a multiple of  one synchronization interval (" << 2 * mac->getSwitchingInterval() << "). This means that beacons are generated during SCH intervals" << std::endl;
                }
                firstBeacon = computeAsynchronousSendingTime(beaconInterval, ChannelType::control);
            }

            if (sendBeacons) {
                scheduleAt(firstBeacon, sendBeaconEvt);
            }
        }
    }
}

simtime_t PhantomTrafficBaseAppLayer::computeAsynchronousSendingTime(simtime_t interval, ChannelType chan)
{

    /*
     * avoid that periodic messages for one channel type are scheduled in the other channel interval
     * when alternate access is enabled in the MAC
     */

    simtime_t randomOffset = dblrand() * beaconInterval;
    simtime_t firstEvent;
    simtime_t switchingInterval = mac->getSwitchingInterval(); // usually 0.050s
    simtime_t nextCCH;

    /*
     * start event earliest in next CCH (or SCH) interval. For alignment, first find the next CCH interval
     * To find out next CCH, go back to start of current interval and add two or one intervals
     * depending on type of current interval
     */

    if (mac->isCurrentChannelCCH()) {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval * 2;
    }
    else {
        nextCCH = simTime() - SimTime().setRaw(simTime().raw() % switchingInterval.raw()) + switchingInterval;
    }

    firstEvent = nextCCH + randomOffset;

    // check if firstEvent lies within the correct interval and, if not, move to previous interval

    if (firstEvent.raw() % (2 * switchingInterval.raw()) > switchingInterval.raw()) {
        // firstEvent is within a sch interval
        if (chan == ChannelType::control) firstEvent -= switchingInterval;
    }
    else {
        // firstEvent is within a cch interval, so adjust for SCH messages
        if (chan == ChannelType::service) firstEvent += switchingInterval;
    }

    return firstEvent;
}

void PhantomTrafficBaseAppLayer::populateWSM(BaseFrame1609_4* wsm, LAddress::L2Type rcvId, int serial)
{
    //If rcvId = null, then LAddress::L2Type rcvId = LAddress::L2BROADCAST()

    wsm->setRecipientAddress(rcvId);
    wsm->setBitLength(headerLength);

    //TODO phantom trafffic beacon
    if (PhantomTrafficMessage* ptm = dynamic_cast<PhantomTrafficMessage*>(wsm)) {
        ptm->setSenderPos(curPosition);
        ptm->setSenderSpeed(curSpeed);
        ptm->setSenderAccel(curAccel);
        ptm->setSenderTime(curTime);
        ptm->setSenderLane(curLane);

        for(int i = 0; i < cxsize; i++)
        {
            if(i< cs.size())
            {
                ptm->setSender_addr(i, sender_addr[i]);
                ptm->setSender_cs(i, cs[i]);
                ptm->setSender_ct(i, ct[i]);
                ptm->setSender_cl(i, cl[i]);
            }
            else
            {
                ptm->setSender_addr(i, LAddress::L2BROADCAST());
                ptm->setSender_cs(i, Coord(0,0,0));
                ptm->setSender_ct(i, -1);
                ptm->setSender_cl(i, -1);
            }           
        }

        ptm->setPsid(-1);
        ptm->setChannelNumber(static_cast<int>(Channel::cch));
        ptm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
        wsm->setBitLength(ptm->getBitLength());
    }
    /*
    else if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        bsm->setSenderPos(curPosition);
        bsm->setSenderSpeed(curSpeed);
        bsm->setPsid(-1);
        bsm->setChannelNumber(static_cast<int>(Channel::cch));
        bsm->addBitLength(beaconLengthBits);
        wsm->setUserPriority(beaconUserPriority);
    }
    
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        wsa->setChannelNumber(static_cast<int>(Channel::cch));
        wsa->setTargetChannel(static_cast<int>(currentServiceChannel));
        wsa->setPsid(currentOfferedServiceId);
        wsa->setServiceDescription(currentServiceDescription.c_str());
    }
    */
    else {
        if (dataOnSch)
            wsm->setChannelNumber(static_cast<int>(Channel::sch1)); // will be rewritten at Mac1609_4 to actual Service Channel. This is just so no controlInfo is needed
        else
            wsm->setChannelNumber(static_cast<int>(Channel::cch));
        wsm->addBitLength(dataLengthBits);
        wsm->setUserPriority(dataUserPriority);
    }
}

void PhantomTrafficBaseAppLayer::receiveSignal(cComponent* source, simsignal_t signalID, cObject* obj, cObject* details)
{
    Enter_Method_Silent();
    if (signalID == BaseMobility::mobilityStateChangedSignal) {
        handlePositionUpdate(obj);
    }
    else if (signalID == TraCIMobility::parkingStateChangedSignal) {
        handleParkingUpdate(obj);
    }
}

void PhantomTrafficBaseAppLayer::handlePositionUpdate(cObject* obj)
{
    //ChannelMobilityPtrType const mobility = check_and_cast<ChannelMobilityPtrType>(obj);
    curPosition = mobility->getPositionAt(simTime());
    
    curTime = simTime().dbl();
    curSpeed = mobility->getSpeed();
    
    curAccel = (curSpeed - prevSpeed) / (curTime - prevTime);

    prevSpeed = curSpeed;
    prevTime = curTime;

    curLane =  traciVehicle->getLaneIndex();
}

void PhantomTrafficBaseAppLayer::handleParkingUpdate(cObject* obj)
{
    isParked = mobility->getParkingState();
}

void PhantomTrafficBaseAppLayer::handleLowerMsg(cMessage* msg)
{

    BaseFrame1609_4* wsm = dynamic_cast<BaseFrame1609_4*>(msg);
    ASSERT(wsm);

    if (PhantomTrafficMessage* ptm = dynamic_cast<PhantomTrafficMessage*>(wsm)) {
        receivedPTMs++;
        onPTM(ptm);
    }
    if (DemoSafetyMessage* bsm = dynamic_cast<DemoSafetyMessage*>(wsm)) {
        receivedBSMs++;
        onBSM(bsm);
    }
    else if (DemoServiceAdvertisment* wsa = dynamic_cast<DemoServiceAdvertisment*>(wsm)) {
        receivedWSAs++;
        onWSA(wsa);
    }
    else {
        receivedWSMs++;
        onWSM(wsm);
    }

    delete (msg);
}


//This will send out a message
//The self message is generated by the veins mobility module to trigger an accident
void PhantomTrafficBaseAppLayer::handleSelfMsg(cMessage* msg)
{
    switch (msg->getKind()) {
    case SEND_BEACON_EVT: {
        PhantomTrafficMessage* ptm = new PhantomTrafficMessage();
        populateWSM(ptm);
        sendDown(ptm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    /*
    case SEND_BEACON_EVT: {
        DemoSafetyMessage* bsm = new DemoSafetyMessage();
        populateWSM(bsm);
        sendDown(bsm);
        scheduleAt(simTime() + beaconInterval, sendBeaconEvt);
        break;
    }
    */
    case SEND_WSA_EVT: {
        DemoServiceAdvertisment* wsa = new DemoServiceAdvertisment();
        populateWSM(wsa);
        sendDown(wsa);
        scheduleAt(simTime() + wsaInterval, sendWSAEvt);
        break;
    }
    default: {
        if (msg) EV_WARN << "APP: Error: Got Self Message of unknown kind! Name: " << msg->getName() << endl;
        break;
    }
    }
}

void PhantomTrafficBaseAppLayer::finish()
{
    recordScalar("generatedWSMs", generatedWSMs);
    recordScalar("receivedWSMs", receivedWSMs);

    recordScalar("generatedBSMs", generatedBSMs);
    recordScalar("receivedBSMs", receivedBSMs);

    recordScalar("generatedWSAs", generatedWSAs);
    recordScalar("receivedWSAs", receivedWSAs);
}

PhantomTrafficBaseAppLayer::~PhantomTrafficBaseAppLayer()
{
    cancelAndDelete(sendBeaconEvt);
    cancelAndDelete(sendWSAEvt);
    findHost()->unsubscribe(BaseMobility::mobilityStateChangedSignal, this);
}

void PhantomTrafficBaseAppLayer::startService(Channel channel, int serviceId, std::string serviceDescription)
{
    if (sendWSAEvt->isScheduled()) {
        throw cRuntimeError("Starting service although another service was already started");
    }

    mac->changeServiceChannel(channel);
    currentOfferedServiceId = serviceId;
    currentServiceChannel = channel;
    currentServiceDescription = serviceDescription;

    simtime_t wsaTime = computeAsynchronousSendingTime(wsaInterval, ChannelType::control);
    scheduleAt(wsaTime, sendWSAEvt);
}

void PhantomTrafficBaseAppLayer::stopService()
{
    cancelEvent(sendWSAEvt);
    currentOfferedServiceId = -1;
}

void PhantomTrafficBaseAppLayer::sendDown(cMessage* msg)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDown(msg);
}

void PhantomTrafficBaseAppLayer::sendDelayedDown(cMessage* msg, simtime_t delay)
{
    checkAndTrackPacket(msg);
    BaseApplLayer::sendDelayedDown(msg, delay);
}

void PhantomTrafficBaseAppLayer::checkAndTrackPacket(cMessage* msg)
{
    if (dynamic_cast<PhantomTrafficMessage*>(msg)) {
        EV_TRACE << "sending down a PTM" << std::endl;
        generatedPTMs++;
    }
    else if (dynamic_cast<DemoSafetyMessage*>(msg)) {
        EV_TRACE << "sending down a BSM" << std::endl;
        generatedBSMs++;
    }
    else if (dynamic_cast<DemoServiceAdvertisment*>(msg)) {
        EV_TRACE << "sending down a WSA" << std::endl;
        generatedWSAs++;
    }
    else if (dynamic_cast<BaseFrame1609_4*>(msg)) {
        EV_TRACE << "sending down a wsm" << std::endl;
        generatedWSMs++;
    }
}

}
