from dronekit import connect, Command, LocationGlobal, VehicleMode
from pymavlink import mavutil

import math, time

CONNECT_STRING = '/dev/ttyACM0'
CAMERA_ANGLE = math.pi/4 #smallest angle of view

class UAV:
    def __init__(self):
        self.vehicle = None
        self.connect_string = CONNECT_STRING

        self.home_position_set = False
        self.offboard_mode_set = False

    def connect2vehicle(self):
        self.vehicle = connect(connection_string, wait_ready=True)
        while not self.home_position_set:
            print "Waiting for home position..."
            time.sleep(1)

        # Display basic vehicle state
        print " Type: %s" % self.vehicle._vehicle_type
        print " Armed: %s" % self.vehicle.armed
        print " System status: %s" % self.vehicle.system_status.state
        print " GPS: %s" % self.vehicle.gps_0
        print " Alt: %s" % self.vehicle.location.global_relative_frame.alt

    @self.vehicle.on_message('HOME_POSITION')
    def hp_listener(self, name, home_position): 
        self.home_position_set = True

    def move2saferegion(self, matrix): #matrix is np.matrix with value 0 or 1
        alt = self.vehicle.location.global_relative_frame.alt
        heading = self.vehicle.heading
        #iteration from top-left corner of the image; needs to be imroved
        for (x,y), value in np.ndenumerate(matrix):
            if (x==1 and y==1):
                pass
            if value:
                forward = math.tan(CAMERA_ANGLE) * alt * (1 - x) * 2/3
                right = math.tan(CAMERA_ANGLE) * alt * (1 - y) * 2/3
                dNorth = forward * math.cos(heading) + right * math.cos(math.pi/2 + heading)
                dEast = forward * math.sin(heading) + right * math.sin(math.pi/2 + heading)
                self.move2wp(dNorth, dEast, 0)
                return True
        return False

    def move2wp(self, dNorth, dEast, dAlt):
        earth_radius=6378137.0 #Radius of "spherical" earth

        #current location
        original_location = self.vehicle.location.global_relative_frame

        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        wp = LocationGlobal(newlat, newlon,original_location.alt+dAlt)

        cmds = self.vehicle.commands
        cmds.clear()
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 1, 0, 0, 0, 0, wp.lat, wp.lon, wp.alt)
        cmds.add(cmd)
        cmds.upload()

        #wait for completion
        while self.vehicle.commands.next > 0:
            print "move2wp:", self.vehicle.commands.next
            time.sleep(1)
        print "move2wp: finished"

    def takeoff(self, alt):
        cmds = self.vehicle.commands
        cmds.clear()
        cmds.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))
        cmds.upload()

        #wait for completion
        while self.vehicle.commands.next > 0:
            print "takeoff:", self.vehicle.commands.next
            time.sleep(1)
        print "takeoff: finished"


    def arm_and_takeoff(self, aTargetAltitude = 10):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't let the user try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

            
        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        while not vehicle.armed:      
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt      
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
                print "Reached target altitude"
                break
            time.sleep(1)

        print "Ready to start missions"
        # Reset mission set to first (0) waypoint
        vehicle.commands.next=0

        # Set mode to AUTO to start mission
        vehicle.mode = VehicleMode("AUTO")

    def land(self, alt = 2):
        cmds = self.vehicle.commands
        cmds.clear()
        cmds.add(Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_LAND, 0, 1, 0, 0, 0, 0, 0, 0, alt))
        cmds.upload()

        #wait for completion
        while self.vehicle.commands.next > 0:
            print "land:", self.vehicle.commands.next
            time.sleep(1)
        print "land: finished"

if __name__ == '__main__':
    uav = UAV()
    uav.connect2vehicle()
    uav.arm_and_takeoff()
    uav.move2wp(2,0.5,0)
    uav.land()

