"""
Dette er en doc string som skal forklare hva koden gjor.

Disclaimer: denne koden har hentet inspirasjon direkte fra dronekit og noen funksjoner kan vaere identiske. Bruken av disse fuksjonene skal vaere med paa aa bygge oppunder den funksjonen som er onskelig at dronen skal kunne gjenommfore
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, Command, LocationGlobal, LocationGlobalRelative 
from pymavlink import mavutil
import time
import math

"************* Selve programmet *************"
def armer_og_lett(drone, hoyde):
    """
    Armerer motorene til dronen, letter og flyr til satt hoyde
    """

    print("Enkel pre-arm sjekk")
    # Ikke prov a armere selv, la programmet gjore det selv
    while not drone.is_armable:
        print(" Venter på initialisering av kjoretoy...")
        time.sleep(1)

        
    print("Arming motors")
    # multirotor bor armere i "GUIDED" modus
    drone.mode = VehicleMode("GUIDED")
    drone.armed = True

    while not drone.armed:      
        print(" Venter pa armering...")
        time.sleep(1)

    print("Letter!")
    drone.simple_takeoff(hoyde) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", drone.location.global_relative_frame.alt)      
        if drone.location.global_relative_frame.alt>=hoyde*0.95: #Trigger just below target alt.
            print("Opnaad satt hoyde")
            break
        time.sleep(1)


def lastned_plan(drone):
    """
    Laster ned planen som er gitt til dronen
    """
    plan = drone.commands
    plan.download()
    plan.wait_ready() # venter til nedlastningen er ferdig.

def fjern_plan(drone):
    """
    Fjerner naaverende plan fra dronen
    """
    plan = drone.commands
    plan.clear()
    plan.flush()

    plan.download()
    plan.wait_ready()

def hent_navaerende_plan(drone):
    lastned_plan(drone)
    planliste = []
    n_veipunkt = 0

    for veipunkt in drone.commands:
        planliste.append(veipunkt)
        n_veipunkt += 1

    return n_veipunkt, planliste

def get_location_metres(original_location, dNorth, dEast):
    """
    Returns a LocationGlobal object containing the latitude/longitude `dNorth` and `dEast` metres from the 
    specified `original_location`. The returned Location has the same `alt` value
    as `original_location`.
    The function is useful when you want to move the vehicle around specifying locations relative to 
    the current vehicle position.
    The algorithm is relatively accurate over small distances (10m within 1km) except close to the poles.
    For more information see:
    http://gis.stackexchange.com/questions/2951/algorithm-for-offsetting-a-latitude-longitude-by-some-amount-of-meters
    """
    earth_radius=6378137.0 #Radius of "spherical" earth
    #Coordinate offsets in radians
    dLat = dNorth/earth_radius
    dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

    #New position in decimal degrees
    newlat = original_location.lat + (dLat * 180/math.pi)
    newlon = original_location.lon + (dLon * 180/math.pi)
    return LocationGlobal(newlat, newlon,original_location.alt)

def bytt_modus(kjoretoy, modus):
    while kjoretoy.modus != Vehicle(modus):
        kjoretoy.modus = VehicleMode(modus)
        time.sleep(0.5)
    return True

def kjorerute(drone, lokasjon, dstr):
    plan = drone.commands
    print("Fjerner eksisterende kommandoer")
    plan.clear()

    #Hvis dronen allerede er i luften vil denne kommandoen bli ignorert
    plan.add(Command(0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_TAKEOFF, 0, 0, 0, 0, 0, 0, 0, 0, 10))

    #definerer punktene den skal fly til
    pkt1 = get_location_metres(lokasjon, dstr, -dstr)
    pkt2 = get_location_metres(lokasjon, dstr, dstr)
    pkt3 = get_location_metres(lokasjon, -dstr, dstr)
    pkt4 = get_location_metres(lokasjon, -dstr, -dstr)

    plan.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pkt1.lat, pkt1.lon, 15))
    
    plan.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pkt2.lat, pkt2.lon, 10))

    plan.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pkt3.lat, pkt3.lon, 15))

    plan.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pkt4.lat, pkt4.lon, 10))
    #dummy punkt
    plan.add(Command( 0, 0, 0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_WAYPOINT, 0, 0, 0, 0, 0, 0, pkt4.lat, pkt4.lon, 10))

    print("Laster opp kommandoer")
    plan.upload()

def distance_to_current_waypoint():
    """
    Gets distance in metres to the current waypoint. 
    It returns None for the first waypoint (Home location).
    """
    nextwaypoint = vehicle.commands.next
    if nextwaypoint==0:
        return None
    missionitem=vehicle.commands[nextwaypoint-1] #commands are zero indexed
    lat = missionitem.x
    lon = missionitem.y
    alt = missionitem.z
    targetWaypointLocation = LocationGlobalRelative(lat,lon,alt)
    distancetopoint = get_distance_metres(vehicle.location.global_frame, targetWaypointLocation)
    return distancetopoint

if __name__ == "__main__":
    "*----- Simulering -----*"
    "************* argparse er for å gi beskjed om hvordan programmet skal brukes *************"
    import argparse
    parser = argparse.ArgumentParser(description='Print out vehicle state information. Connects to SITL on local PC by default.')
    parser.add_argument('--connect', 
                    help="vehicle connection target string. If not specified, SITL automatically started and used.")
    args = parser.parse_args()

    connection_string = args.connect
    sitl = None
    #Starter SITL hvis ikke en koblings string er spsifisert
    if not connection_string:
        import dronekit_sitl
        print("Starter simulering")
        sitl = dronekit_sitl.start_default()
        connection_string = sitl.connection_string()

    "*----- Initialiserer dronen -----*"
    print('Koblet til drone paa: %s' % connection_string)
    drone = connect(connection_string, wait_ready=True)
    hoyde = 20

    "*----- Planlegger kjorerute -----*"
    print("Planlegger kjorerute")
    kjorerute(drone, drone.location.global_frame,20)

    "*----- Starter dronen og utforer oppdrag -----*"
    armer_og_lett(drone, hoyde)
    drone.commands.next=0
    print("Bytter modus til AUTO")
    bytt_modus(drone,"AUTO")

    # Monitor mission. 
    # Demonstrates getting and setting the command number 
    # Uses distance_to_current_waypoint(), a convenience function for finding the 
    #   distance to the next waypoint.

    while True:
        nextwaypoint=drone.commands.next
        print('Distance to waypoint (%s): %s' % (nextwaypoint, distance_to_current_waypoint()))
    
        if nextwaypoint==3: #Skip to next waypoint
            print('Skipping to Waypoint 5 when reach waypoint 3')
            vehicle.commands.next = 5
        if nextwaypoint==5: #Dummy waypoint - as soon as we reach waypoint 4 this is true and we exit.
            print("Exit 'standard' mission when start heading to final waypoint (5)")
            break;
        time.sleep(1)

    print("Returner hjem")
    bytt_modus(drone, "RTL")

    print("Skrur av dronen")
    drone.close()

    if sitl is not None:
        sitl.stop()