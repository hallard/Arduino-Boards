# Arduino Boards

This repository contains support for the following Arduino compatible development boards:
* [ULPNode](http://hallard.me/category/ulpnode/)

Each board will be added as an entry to the Arduino **Tools** > **Board** menu.

### Installation Instructions

You need to use Arduino IDE > 1.6.4

To add this support to your Arduino IDE, simply launch the IDE then 
* go to menu File / Preferences, add the link https://github.com/hallard/Arduino-Boards/raw/master/package_hallard_index.json to the Additional Boards Manager URLs section then ckick OK
* go to Menu Tools / Boards / Board Manager, in the list select ULPNode Arduino Boards and click Instamm
* Once installed you should see ULPNode board into boards menu


### Notes
	
* **Please note: This will only work under Arduino IDE versions 1.6.4 and up.**
* Information on compiling and programming the bootloaders can be found in the bootloaders directory.