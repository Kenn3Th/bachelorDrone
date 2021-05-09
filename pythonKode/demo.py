#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet er en sammensatt demo av objektgjennkjennings modellen og autonom flygings script
"""

#------- Initialisere Objektgjenkjenning -------#

#------- Initialiserer og kobler til dronen -------#
import ALKE
from dronekit import connect, LocationGlobal, LocationGlobalRelative 
drone = connect("/dev/serial0", wait_ready=True, baud=921600)
drone = ALKE.Drone(drone)

import sys
arg = len(sys.argv)
if arg>1:
    antallRunder = int(sys.argv[1])
    if arg>2:
        fart = int(sys.argv[2])
    else:
        fart = 2 #m/s
elif arg>3:
    print("Godtar kun 2 argumenter, antall runder og fart")
    sys.exit()
else:
    antallRunder = 1
    fart = 2 #m/s
startHoyde = 5
startPunkt = [10,10,10]
#-- Styrer dronen
drone.oppdrag(antallRunder)
drone.armer_og_lett(startHoyde)
drone.fly_til(startPunkt[0],startPunkt[1],startPunkt[2])
drone.fart_rBakke(fart)
#Starter oppdrag
drone.oppdrag_film(antallRunder)

#------- Sammenfletting av drone og AI -------#
"""
Her må vi finne ut om vi skal kjøre film. Skal kansje bruke import os til å kjøre skript parallellt.
"""
objektFunnet = False
antallPunkter, punktListe = drone.hent_navaerende_plan()

while True:
    """
    OpenCV magic
    """
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