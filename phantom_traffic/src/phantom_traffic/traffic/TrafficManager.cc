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

//TODO take a look at PlatoonsPlusHumanTraffic and RingTrafficManager to insert traffic into the ring

#include "TrafficManager.h"

Define_Module(TrafficManager);

void TrafficManager::initialize(int stage)
{
    cSimpleModule::initialize(stage);

    if(stage == 0)
    {
        //Read Parameters
        stopVehiclesAt = par("stopVehiclesAt");
        stopVehiclesDuration = par("stopVehiclesDuration");
        numberOfVehicles = par("numberOfVehicles");
        percentageOfSmartCars = par("percentageOfSmartCars");
        percentageOfBrakingCars = par("percentageOfBrakingCars");


        manager = veins::TraCIScenarioManagerAccess().get();
        ASSERT(manager);

        // subscribe to signals
        auto init = [this](veins::SignalPayload<bool>) { traciLoaded(); };
        signalManager.subscribeCallback(manager, veins::TraCIScenarioManager::traciInitializedSignal, init);

        auto timestep_traci = [this](veins::SignalPayload<simtime_t const&>) { timestep(); };
        signalManager.subscribeCallback(manager, veins::TraCIScenarioManager::traciTimestepEndSignal, timestep_traci);
    }
}

void TrafficManager::traciLoaded()
{
    commandInterface = manager->getCommandInterface();

    for(int i = 0; i < numberOfVehicles; i++)
    {
        auto type = (i < percentageOfSmartCars*numberOfVehicles) ? "smart_car" : "human_car";

        if(i % 2 == 0){
            commandInterface->addVehicle(std::to_string(i), type, "r1", 0, i * 6, 27.77f, i%3);
        }
        else
        {
            commandInterface->addVehicle(std::to_string(i), type, "r2", 0, i * 6, 27.77f, i%3);
        }

        auto vehicleCommandInterface = new veins::TraCICommandInterface::Vehicle(commandInterface->vehicle(std::to_string(i)));
        vehicleCommandInterface->setLangeChangeMode(0b001000000000);
        //commandInterface->addVehicle(std::to_string(i), type, "r1", 0, i/100, 27.77f, 0);
    }
}

void TrafficManager::timestep()
{
    if(simTime() > stopVehiclesAt && !vehiclesStopped)
    {
        vehiclesStopped = true;

        for(int i = 0; i < numberOfVehicles; i++)
        {
            //bool shouldStop = (i % (int) ((numberOfVehicles/(percentageOfBrakingCars*numberOfVehicles)) + 1) == 0);
            bool shouldStop = i < (percentageOfBrakingCars*numberOfVehicles*2) && i % 2 == 0;

            if(shouldStop)
            {
                auto vehicleCommandInterface = new veins::TraCICommandInterface::Vehicle(commandInterface->vehicle(std::to_string(numberOfVehicles-1-i)));
                vehicleCommandInterface->setSpeed(0);
                vehicleCommandInterface->setColor(stoppedColor);

            }
        }
    }

    if(simTime() > stopVehiclesAt + stopVehiclesDuration && !vehiclesResumed)
    {
        vehiclesResumed = true;

        for(int i = 0; i < numberOfVehicles; i++)
        {
            //bool shouldStop = (i % (int) ((numberOfVehicles/(percentageOfBrakingCars*numberOfVehicles)) + 1) == 0);
            bool shouldStop = i < (percentageOfBrakingCars*numberOfVehicles*2) && i % 2 == 0;;

            if(shouldStop)
            {
                auto vehicleCommandInterface = new veins::TraCICommandInterface::Vehicle(commandInterface->vehicle(std::to_string(numberOfVehicles-1-i)));
                vehicleCommandInterface->setSpeed(27.77f);
                vehicleCommandInterface->setColor(normalColor);
            }
        }
    }
}


void TrafficManager::handleMessage(cMessage *msg)
{
}
