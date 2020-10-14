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

void TrafficManager::initialize()
{
    const auto scenarioManager = veins::TraCIScenarioManagerAccess().get();
    ASSERT(scenarioManager);

    const auto commandInterface = scenarioManager->getCommandInterface();

    //for(int i = 0; i < 50; i++)
    //{
    //    commandInterface->addVehicle("addedCar"+i, "human_car", "r1", simTime(), i, 27.77f, 0);
    //}
}


void TrafficManager::handleMessage(cMessage *msg)
{
    // TODO - Generated method body
}
