# ESP32 Wireless Client (ESP-NOW) for ESP32 LinuxCNC Ethernet Motion Controller
Connects to the ESP32 Motion Controller for LinuxCNC via a low-latency point-to-point wireless link (ESP-NOW)

### Features
* Motion Controller is still able to act as a WiFi hotspot (softAp) or client (station) simultaneously
* Example project that provides bi-directional communication of both with little-to-no impact to motion controller
* Motion Controller sends its full feedback packet to this client every 1s or 200ms if an axis is moving
* Feedback packet includes IO (Input and Output) states, all axis position and velocitiee, machine control states
* Example data struct included which can be sent from this client based on any event (GPIO etc)
* Motion Controller and LinuxCNC HAL driver can be extended to support wireless client to host control with low latency

## Use Cases / Ideas
* Wireless control panel for LinuxCNC
* LCD/DRO status
* Custom MPG pendant
* Wireless VFD control (modbus/analog) to help remove any VFD control noise
* Extra GPIO for non-time-critical control aspects
* Wireless sensor data streaming directly into LinuxCNC HAL (Temp sensors, tool changer, probes etc)


### Setup
1) Clone this repo into Visual Studio Code PlatformIO
1) Find WiFi MAC address of Motion Controller from its serial console at startup
2) Modify remotePeerAddress variable in main.cpp with the mac address from above
3) Flash to an ESP32 board of your choosing

### TODO
* Probably lots but it's a start
