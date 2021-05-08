#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet er en sammensatt demo av objektgjennkjennings modellen og autonom flygings script
"""

#------- Initialisere Objektgjenkjenning -------#

#------- Initialiserer og kobler til dronen -------#
import ALKE
from dronekit import connect
drone = connect("/dev/serial0",wait_ready=True,baud=921600)
drone = ALKE.Drone(drone)
antallRunder = 6
fart = 3 #m/s
#-- Styrer dronen
drone.oppdrag(antallRunder)
drone.armer_og_lett(5)
drone.fly_til(10,10,10)
drone.fart_rBakke(fart)
kommando = drone.drone.commands
kommando.wait_ready()
kommando.next = 0

#------- Sammenfletting av drone og AI -------#
"""
Her må vi finne ut om vi skal kjøre film. Skal kansje bruke import os til å kjøre skript parallellt.
"""