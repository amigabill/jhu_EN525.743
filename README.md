# jhu_EN525.743

Bill Toner
Repository for my Fall 2015 semester project at JHU Embedded System Development Lab

This project is my "capstone" semester project in Embedded Systems at Johns Hopkins University
as part of my Master's Degree program in Electrical and Computer Engineering, at the
Dorsey Center campus in Elkridge, MD.

This project will create the lighting and ceiling fan control portion of a smarthome home-automation 
system, and will initially consist of 4 segments. The house based controls and targets will communicate
via Zigbee wireless protocol, but will be powered by home 120V AC mains, of the USA standard.

1. LCD Touchscreen wall "switch" controls. These will be either Arduino Uno or Arduino Due based controls.
2. Target items to be controlled, such as light fixtures or ceiling fans. Also Arduino Uno or Due based. 
3. Home server and Zigbee coordinator. This will log control events, control "away/vacation mode" events, 
observe if any residents are home or not, and communicate with remote devices such as Android smartphones. 
(I do not currently have a working Apple iDevice to support the iOS platform)
4. Android remote units. These would typically be smartphones carried with home residents. These will
provide GPS geofence automatic controls, manual NFC tag controls to indicate someone leaves or arrives,
as well as manual control and status observation via an App GUI.

The design should allow additional target types and controls to be added in the future.
