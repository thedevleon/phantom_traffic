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
#include "veins/modules/utility/SignalManager.h"

using namespace omnetpp;

class TrafficManager : public cSimpleModule
{
public:
    int stopVehiclesAt = 50;
    int stopVehiclesDuration = 5;
    int numberOfVehicles = 100;
    double percentageOfSmartCars = 0.03;

  protected:
    virtual void initialize(int stage);
    virtual void handleMessage(cMessage *msg);
    virtual void traciLoaded();
    virtual void timestep();

    /** reference to the simulations ScenarioManager */
    mutable veins::TraCIScenarioManager* manager;
    /** reference to the simulations traffic light-specific TraCI command interface */
    mutable veins::TraCICommandInterface* commandInterface;
    //Signal Manager
    veins::SignalManager signalManager;
    bool vehiclesStopped;
    bool vehiclesResumed;
    const veins::TraCIColor stoppedColor = veins::TraCIColor(255,0,0,255);
    const veins::TraCIColor normalColor = veins::TraCIColor(0,255,0,255);
};

#endif
