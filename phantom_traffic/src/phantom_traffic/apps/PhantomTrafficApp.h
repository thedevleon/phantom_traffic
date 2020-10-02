//
// Copyright (C) 2012-2019 Michele Segata <segata@ccs-labs.org>
//
// SPDX-License-Identifier: GPL-2.0-or-later
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef PHANTOMTRAFFICAPP_H_
#define PHANTOMTRAFFICAPP_H_

#include "veins/base/modules/BaseApplLayer.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"

#include "plexe/CC_Const.h"
#include "phantom_traffic/messages/PhantomTrafficBeacon_m.h"
#include "plexe/mobility/CommandInterface.h"
#include "plexe/utilities/BasePositionHelper.h"

namespace phantom_traffic {

class PhantomTrafficProtocol;

class PhantomTrafficApp : public veins::BaseApplLayer {

public:
    virtual void initialize(int stage) override;

protected:
    // id of this vehicle
    int myId;

    veins::TraCIMobility* mobility;
    veins::TraCICommandInterface* traci;
    veins::TraCICommandInterface::Vehicle* traciVehicle;
    plexe::traci::CommandInterface* plexeTraci;
    std::unique_ptr<plexe::traci::CommandInterface::Vehicle> plexeTraciVehicle;

    // determines position and role of each vehicle
    plexe::BasePositionHelper* positionHelper;

    // lower layer protocol
    PhantomTrafficProtocol* protocol;

    /**
     * Log data about vehicle
     */
    virtual void logVehicleData(bool crashed = false);

    // output vectors for mobility stats
    // id of the vehicle
    cOutVector nodeIdOut;
    // distance and relative speed
    cOutVector distanceOut, relSpeedOut;
    // speed and position
    cOutVector speedOut, posxOut, posyOut;
    // real acceleration and controller acceleration
    cOutVector accelerationOut, controllerAccelerationOut;

    // messages for scheduleAt
    cMessage* recordData;
    // message to stop the simulation in case of collision
    cMessage* stopSimulation;

public:
    PhantomTrafficApp()
    {
        recordData = 0;
        stopSimulation = nullptr;
    }
    virtual ~PhantomTrafficApp();

    /**
     * Sends a unicast message
     *
     * @param msg message to be encapsulated into the unicast message
     * @param destination id of the destination
     */
    void sendUnicast(cPacket* msg, int destination);

protected:
    virtual void handleLowerMsg(cMessage* msg) override;
    virtual void handleSelfMsg(cMessage* msg) override;
    virtual void handleLowerControl(cMessage* msg) override;

    /**
     * Handles PlatoonBeacons
     */
    virtual void onPhantomTrafficBeacon(const PhantomTrafficBeacon* pb);
};

} // namespace phantom_traffic

#endif /* PHANTOMTRAFFICAPP_H_ */
