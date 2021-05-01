#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet tester om klassen ALKE.py fungerer.
"""
import ALKE
from dronekit import connect

#----- Koble til drone -----
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

tilkoblingslink = args.connect
print("Dronen er forbindelse er koblet til på %s"%tilkoblingslink)
drone = connect(tilkoblingslink,wait_ready=True)
drone = ALKE.Drone(drone)

print("Starter test!")
drone.armer_og_lett(10)
fart = 10
drone.fart_rBakke(fart)
drone.fly_til(15,15)
drone.set_roi()
drone.fart_rLuft(fart)
drone.fly_til(-15,15)
drone.set_roi()
drone.fart_rLuft(15)
drone.returner_hjem()

print("Slår av dronen")
drone.AV()