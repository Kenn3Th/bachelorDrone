#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet tester om motorene kan armeres lette 2m, hovre i 5sek deretter lande.
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, Command, LocationGlobal, LocationGlobalRelative 
from pymavlink import mavutil
import time
import ALKE

#----- Koble til drone -----
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

tilkoblingslink = args.connect
print("Dronen er forbindelse er koblet til på %s"%tilkoblingslink)
drone = connect(tilkoblingslink,wait_ready=True)
drone = ALKE.Drone(drone)

print("Første test!")
drone.armer_og_lett(5)

fart = 6
drone.fart_rBakke(fart)
drone.fly_til(90,50)
drone.set_roi()
drone.fart_rBakke(10)
drone.fly_til(50,-60)
drone.fart_rLuft(15)
drone.returner_hjem()

print("Slår av dronen")
drone.AV()