# ArdupilotMicro
Run Ardupilot API on atmega328p.

#About it
It's Ardupilot API port to atmega328p from last version master-AVR branch.

There are 2 example:

    - Compass TEST.
	- Scheduler TEST.

#Important Changes
    STORAGE: Fited to 328p Eeprom size. It's secure work with AP_Param.
	GPIO: Remaped for 328p, all work good.
	SPI: Working.
	I2C: Working.
	Scheduler: Working very good, now we have fully multitaskingon 328p.

#Changes without testing

	RCOutput.
    RCInput.

#Not Working
    
	Analog input.
	ADC.
	
#HOW TO BUILD

    - Download and install Arduino 1.6.6 (Tested with this version).
    - Open Arduino, click File, Preference and select ArdupilotMicro path root.
	- Open included example, compile and enjoy!