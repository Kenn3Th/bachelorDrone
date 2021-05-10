Her er pythonkoden som skal styre pixhawk 4 gjennom en "companion pc" 
ALKE.py er en klasse som bruker dronekit biblioteket til å lage funksjoner som styrer dronen og blir brukt gjennomgående i alle andre skripts.
SimOppdrag.py er programmer som blir brukt med dronekit-sitl simulator og missionPlanner for å feilsøke å verifisere at flygingen skjer slik den er tiltenkt.
Dette er gjort for bachelorprosjektet ALKE i 2021

Pakker som er nødvendige for å kunne kjøre dette programmet er dronekit (https://dronekit.io) and pymavlink (https://github.com/ArduPilot/pymavlink)
