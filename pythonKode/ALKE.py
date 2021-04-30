#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette er en klasse for bachelorprosjektet ALKE
"""
from __future__ import print_function

from dronekit import connect, VehicleMode, Command, LocationGlobal, LocationGlobalRelative 
from pymavlink import mavutil
import math
import time

class Drone:

    def __init__(self, drone):
        self.drone = drone
        self.homeLat = drone.location.global_relative_frame.lat
        self.homeLon = drone.location.global_relative_frame.lon

    def bytt_modus(self, modus):
        print(f"Change vehicle modus from\n{self.drone.mode}")
        while self.drone.mode.name != modus:
            self.drone.mode = VehicleMode(modus)
            time.sleep(0.5)
        print(f"To\n{self.drone.mode}")

    def armer_og_lett(self, hoyde):
        drone = self.drone
        """
        Armerer motorene og letter til 98% av satt høyde
        """
        print("Pre-arm sjekk\nVenter på initialisering")
        while not drone.is_armable:
            print("...")
            time.sleep(1)

        print("Armerer motorene")
        #Bør armere i "GUIDED" modus
        self.bytt_modus("GUIDED")
        drone.armed= True
        while not drone.armed:
            print("Venter på armering...")
            time.sleep(1)
        
        print("Letter!!")
        drone.simple_takeoff(hoyde)

        #Venter på at dronen oppnår 98% av oppgitt høyde
        counter = 0
        while True:
            altitude = drone.location.global_relative_frame.alt
            if counter%3 == 0:
                print(f"Høyde: {altitude}")
            if altitude >= hoyde*0.98:
                print("Oppnådd høyde")
                break
            counter += 1
            time.sleep(1)

    def landing(self):
        #Lander dronen og printer høyde i terminalen
        print("Landing")
        self.bytt_modus("LAND")
        counter = 0
        while True:
            hoyde = self.drone.location.global_relative_frame.alt
            if counter%3 == 0:
                print(f"Høyde: {hoyde}")
            if hoyde <= 5:
                break
            counter += 1
            
    def lastinn_plan(self):
        """
        Laster ned opplastet plan fra mission planner og venter til den er ferdig nedlastet
        """
        plan = self.drone.commands
        plan.download()
        plan.wait_ready() #venter til nedlastningen er ferdig

    def fjer_plan(self):
        #Fjerner nåverende plan
        plan = self.drone.commands
        plan.celar()
        plan.flush()
        plan.download()
        plan.wait_ready()

    def hent_navaerende_plan(self):
        self.lastinn_plan()
        planliste = []
        n_veipunkt = 0
        for veipunkt in drone.commands:
            planliste.append(veipunkt)
            n_veipunkt += 1
        return n_veipunkt, planliste

    def hent_lokasjon_meter(self, original_location, dNorth, dEast):
        """
        Kildekode funnet på: https://dronekit-python.readthedocs.io/en/latest/examples/guided-set-speed-yaw-demo.html
        """
        earth_radius = 6378137.0 #Radius of "spherical" earth
        #Coordinate offsets in radians
        dLat = dNorth/earth_radius
        dLon = dEast/(earth_radius*math.cos(math.pi*original_location.lat/180))

        #New position in decimal degrees
        newlat = original_location.lat + (dLat * 180/math.pi)
        newlon = original_location.lon + (dLon * 180/math.pi)
        if type(original_location) is LocationGlobal:
            targetlocation=LocationGlobal(newlat, newlon,original_location.alt)
        elif type(original_location) is LocationGlobalRelative:
            targetlocation=LocationGlobalRelative(newlat, newlon,original_location.alt)
        else:
            raise Exception("Invalid Location object passed")
            
        return targetlocation;

    def get_distance_metres(self, aLocation1, aLocation2):
        """
        Returns the ground distance in metres between two LocationGlobal objects.

        This method is an approximation, and will not be accurate over large distances and close to the 
        earth's poles. It comes from the ArduPilot test code: 
        https://github.com/diydrones/ardupilot/blob/master/Tools/autotest/common.py
        """
        dlat = aLocation2.lat - aLocation1.lat
        dlong = aLocation2.lon - aLocation1.lon
        return math.sqrt((dlat*dlat) + (dlong*dlong)) * 1.113195e5

    def fly_til(self, dNorth, dEast, speed):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
        the target position. This allows it to be called with different position-setting commands. 
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        drone = self.drone
        currentLocation = drone.location.global_relative_frame
        targetLocation = self.hent_lokasjon_meter(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)
        drone.simple_goto(targetLocation)
        
        #print "DEBUG: targetLocation: %s" % targetLocation
        #print "DEBUG: targetLocation: %s" % targetDistance

        while drone.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.get_distance_metres(drone.location.global_relative_frame, targetLocation)
            print("Distance to target: ", remainingDistance)
            self.drone.groundspeed(fart)
            if remainingDistance<=targetDistance*0.01: #Just below target, in case of undershoot.
                print("Reached target")
                break;
            time.sleep(2)

    def returner_hjem(self):
        simple_goto(self.homeLat, self.homeLon, 2)
        self.fart_rBakke(5)
        alt = self.drone.location.global_relative_frame.alt
        while alt > 2:
            time.sleep(1)
        self.landing()

    def AV(self):
        self.drone.close()

    def fart_rBakke(self, fart):
        self.drone.groundspeed = fart
    
    def fart_rLuft(self, fart):
        self.drone.airspeed = fart