#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette programmet er en sammensatt demo av objektgjennkjennings modellen og 
autonom flygings script for bachelorprosjektet ALKE.

Produsert av Kenneth R. Eikrehagen, Lars-Erik Ulvund og Erik Johansson
"""

#------- Initialisere Objektgjenkjenning -------#
import tensorflow as tf
import cv2

#------- Initialiserer og kobler til dronen -------#
import ALKE
from dronekit import connect, LocationGlobal, LocationGlobalRelative 
drone = connect("/dev/serial0", wait_ready=True, baud=921600)
drone = ALKE.Drone(drone)

import sys #for å gi argumenter når man kjører koden samt avsluttet programmet om en feil skulle oppstå
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
startPunkt = [10,10,10] #Der dronen skal begynne oppdraget må endres om dronen starter på en ny plass

#------- Styrer dronen -------#
#Dronen letter
try:
	drone.armer_og_lett(startHoyde)
except:
	drone.landing()	
	sys.exit()

#Starter oppdrag
try:
	drone.fly_til(startPunkt[0],startPunkt[1],startPunkt[2])
	drone.fart_rBakke(fart)
	drone.oppdrag_film(antallRunder)
except:
	drone.returner_hjem()
	sys.exit()
	
#------- Sammenfletting av drone og AI -------#
"""
Her skal kameraet sammen med objektgjenkjenningen kjøre. Dette må skje etter at dronen har fått oppdraget sitt dette må eventuelt optimaliseres senere.
"""
objektFunnet = False

try:
	while True:
		# GoPro HERO 7
		gst_arg = 'v4l2src device=/dev/video0 ! video/x-raw, format=YUY2, width=1920, height=1080 ! videoconvert ! videoscale ! appsink'

		cap = cv2.VideoCapture(gst_arg, cv2.CAP_GSTREAMER)

		if cap.isOpened():
			window_handle = cv2.namedWindow('vidWindow', cv2.WINDOW_NORMAL)

			# Window
			while cv2.getWindowProperty('vidWindow', 0) >= 0:
				# Capture frames
				ret, frame = cap.read()
				cv2.resizeWindow(vidWindow, 800,450)
				"""
				Erik do your MAGIC!
				"""

			# When everything done, release the capture
			cap.release()
			 
		if objektFunnet:
		    lokasjonObjekt = drone.drone.location.global_relative_frame
		    drone.bytt_modus("GUIDED")
		    drone.drone.simple_goto(lokasjonObjekt)
		    time.sleep(20)
		    drone.returner_hjem()
		    break
except:
	drone.returner_hjem()
	sys.exit()
