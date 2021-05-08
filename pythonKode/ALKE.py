#!/usr/bin/env python
# -*- coding: utf-8 -*-
""" 
Dette er en klasse for bachelorprosjektet ALKE
"""
from __future__ import print_function

from dronekit import VehicleMode, Command, LocationGlobal, LocationGlobalRelative 
from pymavlink import mavutil
import math
import time

class Drone:

    def __init__(self, drone):
        self.drone = drone
        self.homeLat = drone.location.global_relative_frame.lat
        self.homeLon = drone.location.global_relative_frame.lon
        self.homePkt = LocationGlobalRelative(self.homeLat, self.homeLon, 5)

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
        while not drone.armed:
            print("Venter på armering...")
            drone.armed = True
            time.sleep(5)
        
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
            if hoyde <= 0.5:
                break
            counter += 1
            time.sleep(1)
            
    def lastinn_plan(self):
        """
        Laster ned opplastet plan fra mission planner og venter til den er ferdig nedlastet
        """
        plan = self.drone.commands
        plan.download()
        plan.wait_ready() #venter til nedlastningen er ferdig

    def fjern_plan(self):
        #Fjerner nåverende plan
        plan = self.drone.commands
        plan.clear()
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

    def hent_lokasjon_meter(self, original_location, dNorth, dEast, dHight=None):
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
        if dHight is not None:
            if type(original_location) is LocationGlobal:
                targetlocation=LocationGlobal(newlat, newlon,dHight)
            elif type(original_location) is LocationGlobalRelative:
                targetlocation=LocationGlobalRelative(newlat, newlon,dHight)
            else:
                raise Exception("Invalid Location object passed")
        else:
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

    def fly_til(self, dNorth, dEast, dHight=None):
        """
        Moves the vehicle to a position dNorth metres North and dEast metres East of the current position.

        The method takes a function pointer argument with a single `dronekit.lib.LocationGlobal` parameter for 
        the target position. This allows it to be called with different position-setting commands. 
        By default it uses the standard method: dronekit.lib.Vehicle.simple_goto().

        The method reports the distance to target every two seconds.
        """
        drone = self.drone
        currentLocation = drone.location.global_relative_frame
        if dHight is not None:
            targetLocation = self.hent_lokasjon_meter(currentLocation, dNorth, dEast, dHight)
        else:
            targetLocation = self.hent_lokasjon_meter(currentLocation, dNorth, dEast)
        targetDistance = self.get_distance_metres(currentLocation, targetLocation)

        drone.simple_goto(targetLocation)
        
        #print "DEBUG: targetLocation: %s" % targetLocation
        #print "DEBUG: targetLocation: %s" % targetDistance
        if targetDistance<2:
            adjustment = 0.5
        elif targetDistance <10:
            adjustment = 0.4
        else:
            adjustment = 0.05
        while drone.mode.name=="GUIDED": #Stop action if we are no longer in guided mode.
            #print "DEBUG: mode: %s" % vehicle.mode.name
            remainingDistance = self.get_distance_metres(drone.location.global_relative_frame, targetLocation)
            print("Avstand til mål: ", remainingDistance)
            if remainingDistance<=targetDistance*adjustment: #Just below target, in case of undershoot.
                print("Mål nådd")
                break;
            time.sleep(2)

    def returner_hjem(self):
        print("Returnerer til start")
        self.drone.simple_goto(self.homePkt)
        self.set_roi()
        self.fart_rBakke(5)
        
        posisjon = self.drone.location.global_relative_frame
        avstand = self.get_distance_metres(posisjon, self.homePkt)
        alt = posisjon.alt
        landings_punkt = LocationGlobalRelative(self.homeLat, self.homeLon, 2)
        print(f"Avstand hjem {avstand}")
        timeStart = time.time()
        while True:
            na_posisjon = self.drone.location.global_relative_frame
            alt = na_posisjon.alt
            avstand = self.get_distance_metres(na_posisjon, self.homePkt)
            print(f"Avstand hjem {avstand}")
            if avstand<0.4:
                print("Nærme nok")
                self.drone.simple_goto(landings_punkt)
            if alt<=2.1:
                print(f"Høyden er {alt} og gjør klar til landing")
                break
            if (time.time()-timeStart)>120:
                print(f"Dronen har vært {time.time()-timeStart}s i luften og nødlander")
                break
            time.sleep(1)
        self.landing()

    def AV(self):
        self.drone.close()

    def fart_rBakke(self, fart):
        self.drone.groundspeed = fart
    
    def fart_rLuft(self, fart):
        self.drone.airspeed = fart

    def set_roi(self):
        """
        Send MAV_CMD_DO_SET_ROI message to point camera gimbal at a 
        specified region of interest (LocationGlobal).
        The vehicle may also turn to face the ROI.

        For more information see: 
        http://copter.ardupilot.com/common-mavlink-mission-command-messages-mav_cmd/#mav_cmd_do_set_roi
        """
        # create the MAV_CMD_DO_SET_ROI command
        location = self.drone.location.global_relative_frame
        msg = self.drone.message_factory.command_long_encode(
            0, 0,    # target system, target component
            mavutil.mavlink.MAV_CMD_DO_SET_ROI, #command
            0, #confirmation
            0, 0, 0, 0, #params 1-4
            location.lat,
            location.lon,
            location.alt
            )
        # send command to vehicle
        self.drone.send_mavlink(msg)

    def goto_spline(self, dNorth, dEast, dAlt):
        posisjon = self.drone.location.global_relative_frame
        punkt = self.hent_lokasjon_meter(posisjon, dNorth, dEast)
        cmd = Command(0,0,0, mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT, mavutil.mavlink.MAV_CMD_NAV_SPLINE_WAYPOINT , 0, 0, 0, 0, 0, 0, punkt.lat, punkt.lon, dAlt)
        return cmd

"""
    def oppdrag_spline(self):
        print("Definerer oppdrag")
        punkter = [[35,10,10],[35,-10,10],[0,-5,5],[0,-20,5]]
        #Lager oppdraget
        cmds = self.drone.commands
        cmds.wait_ready()
        cmds.clear()
        print("Setter punkter")
        for pkt in punkter:
            nord = pkt[0]
            ost = pkt[1]
            hoyde = pkt[2]
            cmds.add(self.goto_spline(nord,ost,hoyde))
        cmds.upload() #Laster oppdraget til dronen
        self.fly_oppdrag()
    

    def oppdrag_film(self,runder):
        print("Fjerner gammelt oppdrag")
        self.fjern_plan()
        for runde in range(runder):
            self.oppdrag_spline()


    def fly_oppdrag(self):
        self.lastinn_plan()
        plan = self.drone.commands
        plan.next = 0
        self.bytt_modus("AUTO")
        while plan.next<4:
            nestePunkt = plan.next

"""
    def oppdrag_spline(self,runder):
        """
        Lager søke oppdraget til dronen der runder gir hvor mange runder dronen skal ha i søket sitt
        """
        print("Definerer oppdrag")
        punkter = [[35,10,10],[35,-10,10],[0,-5,5],[0,-20,5]]
        #Lager oppdraget
        cmds = self.drone.commands
        cmds.wait_ready()
        cmds.clear()
        print("Setter punkter")
        for runde in range(runder):
            for pkt in punkter:
                nord = pkt[0]
                ost = pkt[1]
                hoyde = pkt[2]
                cmds.add(self.goto_spline(nord,ost,hoyde))
            punkter = punkter*2
            
        cmds.upload() #Laster oppdraget til dronen

    def oppdrag_film(self, runder, fart):
        print("Fått oppdrag om filming!")
        self.oppdrag_spline(runder)
        cmds = self.drone.commands
        print("Laster opp oppdrag")
        cmds.wait_ready() #Venter til punktene er ferdig opplastet
        cmds.download()
        cmds.wait_ready()

        cmds.next = 0
        runde = 0
        self.bytt_modus("AUTO")
        rundeTid = 40.0/fart
        print("Begynner oppdrag!")
        for runde in range(runder):
            startTid = time.time()
            tid = 0
            while tid<rundeTid:
                cmds.next
                tid = time.time()-startTid
        
        self.returner_hjem()
