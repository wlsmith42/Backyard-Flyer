
# Backyard Flyer Project
Udacity Flying Car and Autonomous Flight Engineer Nanodegree
---

The goals / steps of this project are the following:

* Program a quadrotor to fly autonomously in a box using Python.
* Use event-driven programming to program a quadrotor's flight computer.
* Become familiar with the DroneAPI for sending and receiving data from the drone.


[//]: # (Image References)

[image1]: ./images/backyard_flyer.gif "flyer"

---
# Project Overview
This project includes a state machine using event-driven programming to autonomously fly a quadcopter in a Unity simulator. 

The python code is similar to how the drone would be controlled from a ground station computer or an onboard flight computer. Since communication with the drone is done using MAVLink, this code can be used to control an PX4 quadcopter autopilot with very little modification!

## Step 1: Download the Simulator
If you haven't already, download the version of the simulator that's appropriate for your operating system [from this repository](https://github.com/udacity/FCND-Simulator-Releases/releases).

## Step 2: Set up your Python Environment
If you haven't already, set up your Python environment and get all the relevant packages installed using Anaconda following instructions in [this repository](https://github.com/udacity/FCND-Term1-Starter-Kit)

## Task
The required task is to command the drone to fly a 10 meter box at a 3 meter altitude. You'll fly this path in two ways: first using manual control and then under autonomous control.

Manual control of the drone is done using the instructions found with the simulator.

Autonomous control will be done using an event-driven state machine. Each callback will check against transition criteria dependent on the current state. If the transition criteria are met, it will transition to the next state and pass along any required commands to the drone.

This final flight plan can be seen in the gif below:

![alt text][image1]

## Drone API

To communicate with the simulator (and a real drone), you will be using the [UdaciDrone API](https://udacity.github.io/udacidrone/).  This API handles all the communication between Python and the drone simulator.  A key element of the API is the `Drone` superclass that contains the commands to be passed to the simulator and allows you to register callbacks/listeners on changes to the drone's attributes.  The goal of this project is to design a subclass from the Drone class implementing a state machine to autonomously fly a box. A subclass is started for you in `backyard_flyer.py`

### Drone Attributes

The `Drone` class contains the following attributes that you may find useful for this project:

 - `self.armed`: boolean for the drone's armed state
 - `self.guided`: boolean for the drone's guided state (if the script has control or not)
 - `self.local_position`: a vector of the current position in NED coordinates
 - `self.local_velocity`: a vector of the current velocity in NED coordinates

For a detailed list of all of the attributes of the `Drone` class [check out the UdaciDrone API documentation](https://udacity.github.io/udacidrone/).


### Outgoing Commands

The UdaciDrone API's `Drone` class also contains function to be able to send commands to the drone.  Here is a list of commands that you may find useful during the project:

 - `connect()`: Starts receiving messages from the drone. Blocks the code until the first message is received
 - `start()`: Start receiving messages from the drone. If the connection is not threaded, this will block the code.
 - `arm()`: Arms the motors of the quad, the rotors will spin slowly. The drone cannot takeoff until armed first
 - `disarm()`: Disarms the motors of the quad. The quadcopter cannot be disarmed in the air
 - `take_control()`: Set the command mode of the quad to guided
 - `release_control()`: Set the command mode of the quad to manual
 - `cmd_position(north, east, down, heading)`: Command the drone to travel to the local position (north, east, down). Also commands the quad to maintain a specified heading
 - `takeoff(target_altitude)`: Takeoff from the current location to the specified global altitude
 - `land()`: Land in the current position
 - `stop()`: Terminate the connection with the drone and close the telemetry log

These can be called directly from other methods within the drone class:

```python
self.arm() # Seends an arm command to the drone
```

### Manual Flight

The drone can be manually started/stopped from a python/ipython shell:

```python
from udacidrone import Drone
from udacidrone.connection import MavlinkConnection
conn = MavlinkConnection('tcp:127.0.0.1:5760', threaded=False)
drone = Drone(conn,tlog_name="TLog-manual.txt")
drone.start()
```

If `threaded` is set to `False`, the code will block and the drone logging can only be stopped by terminating the simulation. If the connection is threaded, the drone can be commanded using the commands described above, and the connection can be stopped (and the log properly closed) using:

```python
drone.stop()
```

When starting the drone manually from a python/ipython shell you have the option to provide a desired filename for the telemetry log file (such as "TLog-manual.txt" as shown above).  This allows you to customize the telemetry log name as desired to help keep track of different types of log files you might have. 

### Message Logging

The telemetry data is automatically logged in "Logs\TLog.txt" or "Logs\TLog-manual.txt" for logs created when running `python drone.py`. Each row contains a comma seperated representation of each message. The first row is the incoming message type. The second row is the time. The rest of the rows contains all the message properties. The types of messages relevant to this project are:

* `MsgID.STATE`: time (ms), armed (bool), guided (bool)
* `MsgID.GLOBAL_POSITION`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.GLOBAL_HOME`: time (ms), longitude (deg), latitude (deg), altitude (meter)
* `MsgID.LOCAL_POSITION`: time (ms), north (meter), east (meter), down (meter)
* `MsgID.LOCAL_VELOCITY`: time (ms), north (meter), east (meter), down (meter) 


#### Reading Telemetry Logs

Logs can be read using:

```python
t_log = Drone.read_telemetry_data(filename)
```

The data is stored as a dictionary of message types. For each message type, there is a list of numpy arrays. For example, to access the longitude and latitude from a `MsgID.GLOBAL_POSITION`:

```python
# Time is always the first entry in the list
time = t_log['MsgID.GLOBAL_POSITION'][0][:]
longitude = t_log['MsgID.GLOBAL_POSITION'][1][:]
latitude = t_log['MsgID.GLOBAL_POSITION'][2][:]
```

The data between different messages will not be time synced since they are recorded at different times.


## Autonomous Control State Machine

The state machine is run continuously until either the mission is ended or the Mavlink connection is lost.

The six states predefined for the state machine:

* MANUAL: the drone is being controlled by the user
* ARMING: the drone is in guided mode and being armed
* TAKEOFF: the drone is taking off from the ground
* WAYPOINT: the drone is flying to a specified target position
* LANDING: the drone is landing on the ground
* DISARMING: the drone is disarming
    

### Running the State Machine

To run the mission, use the following command:

```sh
python backyard_flyer.py
```

Similar to the manual flight, the GPS data is automatically logged to the specified log file.


### Reference Frames

Two different reference frames are used. Global positions are defined [longitude, latitude, altitude (pos up)]. Local reference frames are defined [North, East, Down (pos down)] and is relative to a nearby global home provided. Both reference frames are defined in a proper right-handed reference frame . The global reference frame is what is provided by the Drone's GPS, but degrees are difficult to work with on a small scale. Conversion to a local frame allows for easy calculation of m level distances. Two convenience function are provided to convert between the two frames. These functions are wrappers on `utm` library functions.

```python
# Convert a local position (north, east, down) relative to the home position to a global position (lon, lat, up)
def local_to_global(local_position, global_home):

# Convert a global position (lon, lat, up) to a local position (north, east, down) relative to the home position
def global_to_local(global_position, global_home):
```

