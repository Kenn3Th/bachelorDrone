#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet tester om klassen ALKE.py fungerer.
"""
import ALKE
from dronekit import connect
import time


#----- Koble til drone -----
import argparse
parser = argparse.ArgumentParser(description='commands')
parser.add_argument('--connect')
args = parser.parse_args()

tilkoblingslink = args.connect
print("Dronen er forbindelse er koblet til på %s"%tilkoblingslink)
drone = connect(tilkoblingslink,wait_ready=True,baud=921600)
drone = ALKE.Drone(drone)

#----- Test kode ------
print("Starter test")
drone.armer_og_lett(2)
time.sleep(5)
drone.landing()
print("Slår av dronen")
drone.AV()