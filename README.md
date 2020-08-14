# Remote 3-2-3

Author: Neil Hutchison (TheJugg1er)
Thanks:  Based on the original work from Kevin Holmes
Version: 0.1
Release: 14 August 2020

This sketch is based on the 3-2-3-Simple Sketch Shared by Kevin Holme for his 3-2-3 system.

I have taken the sketch and modified it to work with a Rolling code Remote trigger, as an addition/alternate 
to an RC remote.
This allows an independent system to trigger the 3-2-3 transition.  A 4 button rolling code remote is used.
The trasnmitter receiver I used is a CHJ-8802
https://www.ebay.com/itm/4-Channel-Rolling-Code-Remote-Receiver-and-Transmitter-CHJ-8802/163019935605?ssPageName=STRK%3AMEBIDX%3AIT&_trksid=p2057872.m2749.l2649


One button the Remote is used to acrivate the 3-2-3 system remotely.  Without that being pressed, no other
buttons will trigger a transition.  This works the same as Kevin's original master switch setting on the RC. (Aux1)

Also added is the ability to receive comands on the USB Serial to manage and trigger the 3-2-3 transitions.
Note that I use this for testing only, and if you connect the system to say MarcDuino or Stealth, you should
use the remote commands with Caution.  The same safety command is required before the main transitions will
be enabled, so there is some protection.  Additionally, the killswitch command will reset after 30 seconds
so if you've not triggered the transition, we go back to safe mode.

Sending P0 will enable/disable the transitions.  It's like a momentary switch with a 30 second timer.
Sending P2 will try to go to a two leg stance.
Sending P3 will try to go to a three leg stance.

The original RC triggers are still available.  Different trigger modes are selected with the #defines below

************************************* WARNING ********************************************
If you're not hooking up an RC Transmitter, you MUST comment out ENABLE_RC_TRIGGER
If you do not the loop will become a 2 second loop due to the code that reads the pulses
from the RC.  Each read has a 1 second timeout and there's two reads!
This will most likely cause an issue on the 3->2 transitions.
******************************************************************************************

The Default Pins are for a Pro Micro

The Sabertooth Libraries can be found here:
https://www.dimensionengineering.com/info/arduino

We include the i2c stuff so that we can both receive commands on i2c, and also so that we can talk to
the LED display via i2c and the two gyro/accelerometer units.  This gives us even more positioning data on
the 2-3-2 transitions, so that we can know more about what is going on.  It may allow us to "auto restore"
good state if things are not where we expect them to be. (That's advanced ... and TBD)
                                                          * 
Note that there is no need for these additional sensors.  Everything will work with just the 4 limit switches.

The Sketch Assumes that the Limit Switches are used in NO mode (Normal Open). This means that when the Switch is
depressed it reads LOW, and will read HIGH when open (or not pressed).

Things that need to happen:

When starting a transition, if the expected Limit switches don't release stop! - TBD
Add a STOP command, so that if the safety is toggled, the sequence stops immediately - DONE
Convert ShowTime to be a timer, instead of a counter.  Just use the counter directly. - TBD
Check for over amperage?? - TBD
