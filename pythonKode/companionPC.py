from __future__ import print_function
"""
Doc-string: her skal det stå forklart hva koden gjør
"""


"*----- Imoporterer nødvendige bibliotek ------*"
from dronekit import connect
import time
import oppsett

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