#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet tester om klassen ALKE.py fungerer.
"""

import ALKE
import sys
from dronekit import connect

#----- Koble til drone -----

tilkoblingslink = "/dev/serial0" 
print("Dronen er forbindelse er koblet til på %s"%tilkoblingslink)
drone = connect(tilkoblingslink,wait_ready=True,baud=921600)
drone = ALKE.Drone(drone)

#------ Demo --------
import sys
arg = len(sys.argv)
if arg>1:
    runder = sys.argv[1]
    if arg>2:
        fart = sys.argv[2]
    else:
        fart = 2
if arg>3:
    print("Godtar kun 2 argumenter, antall runder og fart")
    sys.exit()
else:
    runder = 1
    fart = 2

drone.armer_og_lett(5)
drone.fart_rBakke(fart)
drone.fly_til(10,10,10)
drone.oppdrag_film(runder)
print("Slår av drone")
drone.AV()