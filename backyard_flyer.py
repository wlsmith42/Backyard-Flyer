import argparse
import time
from enum import Enum

import numpy as np

from udacidrone import Drone
from udacidrone.connection import MavlinkConnection, WebSocketConnection  # noqa: F401
from udacidrone.messaging import MsgID


class States(Enum):
    MANUAL = 0
    ARMING = 1
    TAKEOFF = 2
    WAYPOINT = 3
    LANDING = 4
    DISARMING = 5


class BackyardFlyer(Drone):

    def __init__(self, connection):
        super().__init__(connection)
        self.target_position = np.array([0.0, 0.0, 0.0])
        self.all_waypoints = []
        self.in_mission = True
        self.check_state = {}

        # initial state
        self.flight_state = States.MANUAL

        # TODO: Register all your callbacks here
        self.register_callback(MsgID.LOCAL_POSITION, self.local_position_callback)
        self.register_callback(MsgID.LOCAL_VELOCITY, self.velocity_callback)
        self.register_callback(MsgID.STATE, self.state_callback)

    def local_position_callback(self):
        #This triggers when `MsgID.LOCAL_POSITION` is received and self.local_position contains new data

        if self.flight_state == States.TAKEOFF:
            if -1.0 * self.local_position[2] > 0.95 * self.target_position[2]:
                self.all_waypoints = self.calculate_box()
                self.waypoint_transition()
        elif self.flight_state == States.WAYPOINT:
            if np.linalg.norm(self.target_position[0:2] - self.local_position[0:2]) < 1.0:
                if len(self.all_waypoints) > 0:
                    self.waypoint_transition()
                else:
                    if np.linalg.norm(self.local_velocity[0:2]) < 1.0:
                        self.landing_transition()

    def velocity_callback(self):
        #This triggers when `MsgID.LOCAL_VELOCITY` is received and self.local_velocity contains new data

        if self.flight_state == States.LANDING:
            if self.global_position[2] - self.global_home[2] < 0.1:
                if abs(self.local_position[2]) < 0.01:
                    self.disarming_transition()

    def state_callback(self):
        #This triggers when `MsgID.STATE` is received and self.armed and self.guided contain new data

        if self.in_mission:
            if self.flight_state == States.MANUAL:
                self.arming_transition()
            elif self.flight_state == States.ARMING:
                if self.armed:
                    self.takeoff_transition()
            elif self.flight_state == States.DISARMING:
                if ~self.armed & ~self.guided:
                    self.manual_transition()

    def calculate_box(self):
        #Return waypoints to fly a box

        waypoints = [[10.0, 0.0, 3.0], [10.0, 10.0, 3.0], [0.0, 10.0, 3.0], [0.0, 0.0, 3.0]]
        return waypoints

    def arming_transition(self):

        print("arming transition")

        #Take control of the Drone
        self.take_control()
        #Pass an arming Command
        self.arm()
        #Set the home location to current position
        self.set_home_position(self.global_position[0], self.global_position[1], self.global_position[2])
        #Transition to the ARMING state
        self.flight_state = States.ARMING

    def takeoff_transition(self):

        print("takeoff transition")

        #Set target_position altitute to 3.0m
        target_altitude = 3.0
        self.target_position[2] = target_altitude
        #Command a takeoff to 3.0m
        self.takeoff(target_altitude)
        #Transition to the TAKEOFF state
        self.flight_state = States.TAKEOFF

    def waypoint_transition(self):

        print("waypoint transition")

        #Command the next waypoint position
        self.target_position = self.all_waypoints.pop(0)
        print('target position', self.target_position)
        self.cmd_position(self.target_position[0], self.target_position[1], self.target_position[2], 0.0)
        #Transition to WAYPOINT state
        self.flight_state = States.WAYPOINT

    def landing_transition(self):

        print("landing transition")

        #Command the drone to land
        self.land()
        #Transition to the LANDING state
        self.flight_state = States.LANDING

    def disarming_transition(self):

        print("disarm transition")

        #Command the drone to disarm
        self.disarm()
        self.release_control()
        #Transition to the DISARMING state
        self.flight_state = States.DISARMING

    def manual_transition(self):

        print("manual transition")

        #Releases control of the drone
        self.release_control()
        #Stops the connection (and telemetry log)
        self.stop()
        #Ends the mission
        self.in_mission = False
        #Transitions to the MANUAL state
        self.flight_state = States.MANUAL

    def start(self):

        print("Creating log file")

        #Opens a log file
        self.start_log("Logs", "NavLog.txt")
        print("starting connection")
        #Starts the drone connection
        self.connection.start()
        print("Closing log file")
        #Closes the log file
        self.stop_log()


if __name__ == "__main__":
    parser = argparse.ArgumentParser()
    parser.add_argument('--port', type=int, default=5760, help='Port number')
    parser.add_argument('--host', type=str, default='127.0.0.1', help="host address, i.e. '127.0.0.1'")
    args = parser.parse_args()

    conn = MavlinkConnection('tcp:{0}:{1}'.format(args.host, args.port), threaded=False, PX4=False)
    #conn = WebSocketConnection('ws://{0}:{1}'.format(args.host, args.port))
    drone = BackyardFlyer(conn)
    time.sleep(2)
    drone.start()
