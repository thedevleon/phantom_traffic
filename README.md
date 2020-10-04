# NES Phantom Traffic Dissipation with V2V

Source Repository for NES Project.

## Software Stack
- OMNet++ (latest)
- SUMO (v1_7_0)
- Veins (master)
- Veins 5.0 (see <http://veins.car2x.org/>)
- OMNeT++ 5.5.1 (see <https://omnetpp.org/>)
- Plexe

## Resources
### General
- OMNet++, VEINS, SUMO Overview - https://www.youtube.com/playlist?list=PLJfaBuRXBgpd6Qg4hqS2Ul9blAYYFx5XG

### SUMO
- Simulates traffic, has built in algorithms for cruise control, lane switching, etc...
- Documentation: https://sumo.dlr.de/docs/index.html 
- Build a Highway: https://sumo.dlr.de/docs/Tutorials/Autobahn.html 
- Import Map: https://sumo.dlr.de/docs/Tutorials/Import_from_OpenStreetMap.html#convert_the_map_in_a_sumo_network 
- Create Map from OSM: https://sumo.dlr.de/docs/Tutorials/OSMWebWizard.html 
- New controller could be implemented here

### Veins
- Documentation: https://veins.car2x.org/
- Implements IEEE 801.11p for MAC
- Provides relaistic PHY simulation based on antenna modelling, obstacles, etc...
- More info here: http://veins.car2x.org/documentation/modules/

### Plexe
- TODO

## Setup

### 1. Build and install OMNet++
See Chapter 4. Linux in https://doc.omnetpp.org/omnetpp/InstallGuide.pdf.
Make sure that `omnetpp` opens the omnet IDE.

### 2. Build SUMO and add to Path
Instructions: https://sumo.dlr.de/docs/Installing/Linux_Build.html 
```bash
sudo apt-get install cmake python g++ libxerces-c-dev libfox-1.6-dev libgdal-dev libproj-dev ibgl2ps-dev swig
cd sumo
export SUMO_HOME="$PWD"
mkdir build/cmake-build && cd build/cmake-build
cmake ../..
make -j$(nproc)
```
Then add sumo to your path. Make sure `sumo-gui` opens the SUMO GUI.

### 3. Build Project
```bash
./configure
make
```

### 5. Open Project in Eclipse
You should be able to import the project into your own workspace in eclipse now.
Simply choose this folder, and you should be able to import phantom_traffic, plexe-veins, and veins.

## Running Examples

Make sure that the sumo-launchd is running.
```bash
cd veins/
./sumo-launchd.py -vv -c sumo-gui
```
Then you can run the simulations.

```bash
cd phantom_traffic/simulations/ring

./run -u Cmdenv -c Default
```

To see the Qtenv (which shows communications and events), run with `Qtenv` instead of `Cmdenv`.

```bash
./run -u Qtenv -c Default
```

Hit run in Qtenv and then sumo-gui should pop up, hit run there as well.

Or try running it directly in OMNet++ by clicking on the omnetpp.ini and hitting run. Might work. Might not work. Not tested enough.


# Source Study Fidings
- Veins provides 802.11p PHY and MAC layer (src/veins/model/phy and src/veins/model/mac) - there's also an demo application layer, however it's quite complex.

- Cookiecutter template for Veins which implements this layer: https://github.com/veins/cookiecutter-veins-project

- Minimum Base for Application layer can be found in BaseApplLayer. Both DemoBaseApplLayer, ApplicationLayerTest, and the application layers of Plexe are based on it.

- Example application (layer) is implemented in
    - DemoBaseApplLayer, with an interface to the MAC layer via the DemoBaseApplLayerToMac1609_4Interface
    - Messages: 
        - DemoSafetyMessage
        - DemoServiceAdvertisment

- Another example is in ApplicationLayerTest, an extension of DemoBaseApplLayer and basis for the cookiecutter template.

- The veinsmobility module in the car (veins/nodes/Car.ned), which is the TraCIMobility class, will initiate an accident after a set time.
The omnetpp.ini only sets the accidentCount to 1 in the first node `*.node[*0].veinsmobility.accidentCount = 1`.
Thus, only the veinsmobility module of the first car will set the speed to 0 (in `TraCIMobility::handleSelfMsg`) at the accident time specified.
The application layer get's a position update every step of the simulation from TraCI and if the application layer realizes the car has been stopped for more than 10 seconds, it will start broadcasting a WSA, which will trigger other cars that have received that message to broadcast it as well, flooding the network. 

- All other cars will continue to relay the WSA sent from the car with the accident.


- TraCI allows us to get values directly from SUMO (vehicle position, roads, etc...) and allows us to perform lane changing, breaking, accelleration, etc... aka god mode
    - for more see https://sumo.dlr.de/docs/TraCI.html
    - VEINS built a custom module for this, see http://veins.car2x.org/documentation/modules/

- Another example with TraCI is in (src/veins/model/application/traci)
    - TraCIDemo11p (CAR)
    - TraCIDemoRSU11p (RSU)

    It emulates multiple cars and a roadside unit. After a little while an car will stop (make an accident) and the RSU will inform other cars, which will avoid that route and try to drive around it.

    
- Plexe provides a simple protocol, found in BaseProtocol and is extended to a slotted Beaconing protocl in SlottedBeaconing. The Base Protocol has messages with vehicle information that are unicasted.

- Plexe also provides a Base Application layer that uses the Base Protocol. Could be modified and extended for our purposes.

# TODO
- Make our own Application Layer based on the provided application layer and by by combining TraCIDemo11p and TraCIDemoRSU11p for accessing vehicle data
- Make our own Message based on the PlatooningBeacon
