# Trottinette_Electrique_PFE_L3
This is my bachelor project repository.
The goal of this project is to build a smart scooter and its android app.
We should :
- using a DC motor, control it in order to move the scooter 
- visualize some data on a graphical user interface
- localise the scooter and stop it if it was stolen or it is outside a defined area

### 1. Embedded Program
This repo contains the embeded program. This program does the interfacing between the conceived card, actuators, sensors, and Graphical User Interface.

### 2. App
We can visualize on the android App some data from the state of the scooter such as : 
- speed
- battery state
- ligth state and control
- real time trajectory
With this app, we first unlock the app and the scooter by scanning a QR_Code.

### 3. PCB design - Altium Designer
To control the DC motor, we designed a card. This card has as components :
- some resistors, 
- optocoupler for power and control part isolation 
- amplifier
- transistors
- diodes
- infrared led
- photodiode
- LDR (for turning on the light according to the external ligthing conditions)
- Potentiometer
To conceived this card, we used Altium Designer and for the realization, we used a chemical process.

#### keywords : 
C/C++, FreeRToS, ESP32 Dev Module, Xbee, GPS, MIT App Inventor, Altium Designer
