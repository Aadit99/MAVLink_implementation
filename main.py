from dronekit import connect, VehicleMode, LocationGlobal, LocationGlobalRelative
from pymavlink import mavutil
import time
import math
import argparse  
import dronekit_sitl

print("Starting simulator (SITL)")
sitl = dronekit_sitl.start_default()
connection_string = sitl.connection_string()

print(f"Connecting to vehicle on: {(connection_string,)}") 
vehicle = connect(connection_string, wait_ready=True)

print(f"Getting some vehicle attribute values:")
print(f" GPS: {vehicle.gps_0}")
print(f" Battery: {vehicle.battery}")
print(f" Last Heartbeat: {vehicle.last_heartbeat}")
print(f" Is Armable?: {vehicle.is_armable}") 
print(f" System status: {vehicle.system_status.state }")
print(f" Mode: {vehicle.mode.name}")


def arm_and_takeoff(aTargetAltitude):

    """ 
    Arms vehicle and fly to aTargetAltitude. 
    """

    print("Basic pre-arm checks")
    
    while not vehicle.is_armable: 
        print( " Waiting for vehicle to initialise..." )
        time.sleep(1)
    
    print ("Arming motors" )
     
    vehicle.mode = VehicleMode("GUIDED") 
    vehicle.armed = True 
     
    while not vehicle.armed: 
        print( " Waiting for arming..." )
        time.sleep(1)
    
    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) 
     
    while True: 
        print( f" Altitude: {vehicle.location.global_relative_frame.alt}")
         
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: 
            print( "Reached target altitude" )
            break 
        time.sleep(1)


"""
MAV_CMD_CONDITION_YAW - set direction of the front of the Copter
MAV_CMD_DO_SET_ROI - set direction where the camera gimbal is aimed 
MAV_CMD_DO_CHANGE_SPEED - set target speed in metres/second.
"""

def condition_yaw(heading, relative=False):
    """
    This method sets an absolute heading by default, but you can set the `relative` parameter to `True` to set yaw relative to the current yaw heading.

    By default the yaw of the vehicle will follow the direction of travel. After setting the yaw using this function there is no way to return to the default yaw "follow direction of travel" behaviour

    message_factory is used to factory encode a message before using the send_mavlink function to transmit the message to  the UAV. meesage_factory contains the encoded version of each message. Example- to encode a SET_POSITION_TARGET_LOCAL_NED message we call message_factory.set_position_target_local_ned_encode function.

    vehicle.message_factory.command_long_encode(target system, target component, command, confirmation, yaw in degrees, yaw speed deg/s, direction, relative offset 1 absolute angle 0, unused, unused, unused)
    """
    
    if relative:
        is_relative = 1
    else:
        is_relative = 0 
    
    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_CONDITION_YAW, 0, heading, 0, 1, is_relative, 0, 0, 0)
    vehicle.send_mavlink(msg)


def set_roi(location):
    """
    Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a specified region of interest (LocationGlobal).The vehicle may also turn to face the same.

    message_factory is used to factory encode a message before using the send_mavlink function to transmit the message to  the UAV. meesage_factory contains the encoded version of each message. Example- to encode a SET_POSITION_TARGET_LOCAL_NED message we call message_factory.set_position_target_local_ned_encode function.

    Note that latitude, longitude and altitude are also taken as inputs for the camera gimbal to point.
    """

    msg = vehicle.message_factory.command_long_encode(0, 0, mavutil.mavlink.MAV_CMD_DO_SET_ROI, 0, 0, 0, 0, 0, location.lat, location.lon, location.alt)
    vehicle.send_mavlink(msg)



"""
get_location_metres - Get LocationGlobal (decimal degrees) at distance (m) North & East of a given LocationGlobal.
get_distance_metres - Get the distance between two LocationGlobal objects in metres
get_bearing - Get the bearing in degrees to a LocationGlobal
"""

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the specified `original_location`. The returned LocationGlobal has the same `alt` value
    as `original_location`.

    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    """
    earth_radius = 6378137.0 
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    if type(original_location) is LocationGlobal:
        targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
    elif type(original_location) is LocationGlobalRelative:
        targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
    else:
        raise Exception("Invalid Location object passed")
        
    return targetlocation


def get_distance_metres(aLocation1, aLocation2):
    """
    Returns the ground distance in metres between two LocationGlobal objects.
    """
    dlat = aLocation2.lat - aLocation1.lat
    dlong = aLocation2.lon - aLocation1.lon
    return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5


def get_bearing(aLocation1, aLocation2):
    """
    Returns the bearing between the two LocationGlobal objects passed as parameters.
    """	
    off_x = aLocation2.lon - aLocation1.lon
    off_y = aLocation2.lat - aLocation1.lat
    bearing = 90.00 + math.atan2(-off_y, off_x) * 57.2957795
    if bearing < 0:
        bearing += 360.00
    return bearing



"""
Functions to move the vehicle to a specified position (as opposed to controlling movement by setting velocity components).

goto_position_target_global_int - Sets position using SET_POSITION_TARGET_GLOBAL_INT command in MAV_FRAME_GLOBAL_RELATIVE_ALT_INT frame
goto_position_target_local_ned - Sets position using SET_POSITION_TARGET_LOCAL_NED command in MAV_FRAME_BODY_NED frame
goto - A convenience function that can use Vehicle.simple_goto (default) or goto_position_target_global_int to travel to a specific position in metres North and East from the current location. This method reports distance to the destination.
"""

def goto_position_target_global_int(aLocation):
    """
    Send SET_POSITION_TARGET_GLOBAL_INT command to request the vehicle fly to a specified LocationGlobal.

    vehicle.message_factory.set_position_target_global_int_encode(unused, target_system, target_component, frame, type_mask, lat_int - X Position in WGS84 frame in 1e7 * meters , lon_int, alt, x velocity in NED frame, y velocity, z velocity, afx, afy, afz, yaw, yaw_rate)
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111111000, aLocation.lat*1e7, aLocation.lon*1e7, aLocation.alt, 0, 0, 0, 0, 0, 0, 0, 0)     
    vehicle.send_mavlink(msg)


def goto_position_target_local_ned(north, east, down):
    """	
    Send SET_POSITION_TARGET_LOCAL_NED command to request the vehicle fly to a specified location in the North, East, Down frame.

    It is important to remember that in this frame, positive altitudes are entered as negative "Down" values. So if down is "10", this will be 10 metres below the home altitude.

    type_mask (0=enable, 1=ignore). 

    set_position_target_local_ned_encode(time_boot_ms, target system, target component, frame, type_mask, x pos, y, z, x velocity, y, z, x acceleration, y, z, yaw, yaw_rate)
    """

    msg = vehicle.message_factory.set_position_target_local_ned_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111111000, north, east, down, 0, 0, 0, 0, 0, 0, 0, 0)    
   
    vehicle.send_mavlink(msg)



def goto(dNorth, dEast, gotoFunction=vehicle.simple_goto):
    """
    Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

    The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
    the target position. This allows it to be called with different position-setting commands. 
    By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

    The method reports the distance to target every two seconds.
    """
    
    currentLocation = vehicle.location.global_relative_frame
    targetLocation = get_location_metres(currentLocation, dNorth, dEast)
    targetDistance = get_distance_metres(currentLocation, targetLocation)
    gotoFunction(targetLocation)
    
    #print "DEBUG: targetLocation: %s" % targetLocation
    #print "DEBUG: targetLocation: %s" % targetDistance

    while vehicle.mode.name=="GUIDED": 
        remainingDistance=get_distance_metres(vehicle.location.global_relative_frame, targetLocation)
        print("Distance to target: ", remainingDistance)
        if remainingDistance<=targetDistance*0.01: 
            print("Reached target")
            break
        time.sleep(2)


"""
Functions that move the vehicle by specifying the velocity components in each direction.
The two functions use different MAVLink commands. The main difference is
that depending on the frame used, the NED velocity can be relative to the vehicle
orientation.
send_ned_velocity - Sets velocity components using SET_POSITION_TARGET_LOCAL_NED command
send_global_velocity - Sets velocity components using SET_POSITION_TARGET_GLOBAL_INT command
"""

def send_ned_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors and for the specified duration.
    This uses the SET_POSITION_TARGET_LOCAL_NED command with a type mask enabling only velocity components. 
    
    type_mask (0=enable, 1=ignore). 

    set_position_target_local_ned_encode(unused, target system, target component, frame, type_mask, x pos, y pos, z pos, x vel, y vel, z vel, x acc, y acc, z acc, yaw, yaw_rate)
    """
    msg = vehicle.message_factory.set_position_target_local_ned_encode(0, 0, 0,  mavutil.mavlink.MAV_FRAME_LOCAL_NED, 0b0000111111000111, 0, 0, 0, velocity_x, velocity_y, velocity_z,0, 0, 0, 0, 0)    

    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)
    
    


def send_global_velocity(velocity_x, velocity_y, velocity_z, duration):
    """
    Move vehicle in direction based on specified velocity vectors.

    This uses the SET_POSITION_TARGET_GLOBAL_INT command with type mask enabling only velocity components 

    set_position_target_global_int_encode(time_boot_ms (not used), target system, target component, frame, type_mask (only speeds enabled), X Position in WGS84 frame in 1e7 * meters, Y Position in WGS84 frame in 1e7 * meters, Altitude in meters in AMSL altitude(not WGS84 if absolute or relative), X velocity in NED frame in m/s, Y velocity, Z velocity, afx, afy, afz acceleration, yaw, yaw_rate)
    """
    msg = vehicle.message_factory.set_position_target_global_int_encode(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT_INT, 0b0000111111000111, 0, 0,0,velocity_x,velocity_y,velocity_z,0, 0, 0, 0, 0)
    for x in range(0,duration):
        vehicle.send_mavlink(msg)
        time.sleep(1)    


vehicle.mode = VehicleMode("LAND")
print("Close vehicle object")
vehicle.close()

if sitl is not None:
    sitl.stop()

print("Completed")