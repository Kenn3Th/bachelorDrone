import ALKE
import sys
import time
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
    antallRunder = int(sys.argv[1])
    if arg>2:
        fart = int(sys.argv[2])
    else:
        fart = 2
elif arg>3:
    print("Godtar kun 2 argumenter, antall runder og fart")
    sys.exit()
else:
    antallRunder = 1
    fart = 2

startHoyde = 5
startPunkt = [10,10,10]
#-- Styrer dronen
drone.fjern_plan()
try:
    drone.armer_og_lett(startHoyde)
except Exception as e:
    print("Noe gikk galt, landerog avslutter")
    drone.landing()
    sys.exit()

try:
    drone.fly_til(startPunkt[0],startPunkt[1],startPunkt[2])
    drone.fart_rBakke(fart)
    #Starter oppdrag
    drone.oppdrag_film(antallRunder)
except Exception as e:
    print(f"Noe gikk galt, error: {e}\stopper programmet")
    drone.returner_hjem()
    sys.exit()

#------- Sammenfletting av drone og AI -------#
"""
Her må vi finne ut om vi skal kjøre film. Skal kansje bruke import os til å kjøre skript parallellt.
"""
objektFunnet = False
antallPunkter, punktListe = drone.hent_navaerende_plan()
tidStart = time.time()
try:
    while True:
        """
        OpenCV magic
        """
        tidFunnet = 30
        if (tidStart-time.time())>=tidFunnet:
            objektFunnet = True
        gjennomfortPunkter = drone.drone.commands.next
        if objektFunnet:
            lokasjonObjekt = drone.drone.location.global_relative_frame
            drone.bytt_modus("GUIDED")
            drone.drone.simple_goto(lokasjonObjekt)
            time.sleep(20)
            drone.returner_hjem()
            break
        elif gjennomfortPunkter >= antallPunkter:
            drone.returner_hjem()
            break
except Exception as e:
    print(f"Noe gikk galt, error: {e}\nreturnerer hjem")
    drone.returner_hjem()
    sys.exit()