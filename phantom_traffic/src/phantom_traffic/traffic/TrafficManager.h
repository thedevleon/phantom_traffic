//
// This program is free software: you can redistribute it and/or modify
// it under the terms of the GNU Lesser General Public License as published by
// the Free Software Foundation, either version 3 of the License, or
// (at your option) any later version.
// 
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU Lesser General Public License for more details.
// 
// You should have received a copy of the GNU Lesser General Public License
// along with this program.  If not, see http://www.gnu.org/licenses/.
// 

#ifndef __PHANTOM_TRAFFIC_TRAFFICMANAGER_H_
#define __PHANTOM_TRAFFIC_TRAFFICMANAGER_H_

#include <omnetpp.h>

#include "veins/veins.h"

#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"

using namespace omnetpp;

/**
 * TODO - Generated class
 */
class TrafficManager : public cSimpleModule
{

  protected:
    virtual void initialize();
    virtual void handleMessage(cMessage *msg);

    /** reference to the simulations ScenarioManager */
    mutable veins::TraCIScenarioManager* manager;
    /** reference to the simulations traffic light-specific TraCI command interface */
    mutable veins::TraCICommandInterface* commandInterface;

    virtual veins::TraCIScenarioManager* getManager() const
    {
        if (!manager) {
            manager = veins::TraCIScenarioManagerAccess().get();
        }
        return manager;
    }
    virtual veins::TraCICommandInterface* getCommandInterface() const
    {
        if (!commandInterface) {
            commandInterface = getManager()->getCommandInterface();
        }
        return commandInterface;
    }
};

#endif
