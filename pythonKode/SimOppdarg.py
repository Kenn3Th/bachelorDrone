"""
Dette programmet simulerer oppdraget vi har definert i missionPLanner. Oppdraget er oppdrag.py og blir simulert ved hjelp av dronekit-sitl og ubuntu20.x
"""
import ALKE
import sys
from dronekit import connect

#----- Koble til drone -----

tilkoblingslink = "udp:127.0.0.1:14551" 
print(f"Dronen er forbindelse er koblet til på {tilkoblingslink}")
drone = connect(tilkoblingslink,wait_ready=True)
drone = ALKE.Drone(drone)

#------ Demo --------
import sys
arg = len(sys.argv)
if arg>1:
    runder = int(sys.argv[1])
    if arg>2:
        fart = int(sys.argv[2])
    else:
        fart = 2
elif arg>3:
    print("Godtar kun 2 argumenter, antall runder og fart")
    sys.exit()
else:
    runder = 1
    fart = 2

print("Starter programmet")
print("fjerner gammelt oppdrag")
drone.fjern_plan()
drone.armer_og_lett(5)
print(f"Setter fart til {fart}, og gjør klar til å fly {runder} runder")
drone.fart_rBakke(fart)
print("Flyr til første punkt")
drone.fly_til(10,10,10)
print("Setter opp og utfører oppdraget")
drone.oppdrag_film(runder,fart)
drone.returner_hjem()
print("Slår av drone")
drone.AV()
