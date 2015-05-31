TinyG Introduction
========

![TinyG v7 Board](http://farm9.staticflickr.com/8186/8436183471_6b3708cd0d_c.jpg)

TinyG is a 6 axis motion control system designed for high-performance on small to mid-sized machines. Some features:

* 6 axis motion (XYXABC axes)
* jerk controlled motion for acceleration planning (3rd order motion planning)
* status displays ('?' character)
* XON/XOFF protocol over serial
* config is necessarily different to take into account the larger number of settings
* RESTful interface using JSON

See these links for more details.

* [Synthetos](https://www.synthetos.com/)
* [TinyG Wiki](https://www.synthetos.com/wiki/index.php?title=Projects:TinyG)
* [TinyG Support Forum](https://www.synthetos.com/forum/tinyg/)
* [TinyG Github](https://github.com/synthetos/TinyG)
* [Synthetos Web Store](https://www.synthetos.com/webstore/)


CURRENT MASTER VERSION
========
<<<<<<< HEAD
The current master version is 0.95, BUILD 370.01
Please refer to the [TinyG wiki](https://github.com/synthetos/TinyG/wiki) for documentation and user manuals

The previous master version is 0.94, BUILD 339.09 (GoGo Boots)
=======
The current master version is 0.94, BUILD 339.09 (GoGo Boots)
>>>>>>> refs/heads/edge
Changelog:

339.09: 
BUG FIXES IN 0.94
* Issue #14 - Full circle G02 results in error - fixed
* Issue #15 - Ignore CR and Ignore LF comnined into a single command to prevent both from being active at once and bricking TinyG
* Checksum added to JSON output lines

338.12

MAJOR FEATURES IN 0.93
* [Homing cycles added - G28.1](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing)
* [Return to home added - G28](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Homing)
* Multiple work coordinate systems added - G54, G55, G56, G57, G58, G59 as per [NIST rs274NGCv3 specification](http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.141.2441)
* Coordinate system offset support added - G92 as per [NIST rs274NGCv3 specification](http://citeseerx.ist.psu.edu/viewdoc/summary?doi=10.1.1.141.2441)
* [Feedhold and cycle(re)start added (! and ~ respectively)](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Gcode-Support#Starting.2C_Stopping.2C_Feedhold_and_Rate_Overrides_-_Design_Notes)
* [Software reset command added (control-X command)](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Gcode-Support#Starting.2C_Stopping.2C_Feedhold_and_Rate_Overrides_-_Design_Notes)
* [TinyG JSON Support added](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-JSON)
* [Configuration has changed](http://www.synthetos.com/wiki/index.php?title=TinyG:Configuring) 
* [Support for spindle control added - M3,M4,M5](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Gcode-Support#Gcode_Language_Support)
* [Support for coolant control added - M7,M8,M9](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG-Gcode-Support#Gcode_Language_Support)

MINOR FEATURES AND INTERNALS IN 0.93
* Status reports are now configurable via JSON command
* Arcs are now planned through the line planner
* Help display updated (type $h)
* Self tests added (type $test)
* Restore settings to defaults command added (type $defaults)
* Configuration and display sub-system compeletly re-written for flexibilioty and to handle JSON

BUG FIXES IN 0.93
* Issues #5, #7 and #12 - Issues were found and fixed that affected positional accuracy with some combinations of settings (fixed since 0.93)
* Issue #10 - Help display error fixed
* Issue #8 - $xTR did not update until reset
* Fixed some typos in command strings (fixed since 0.93)

* Removed diagnostic logging to fix bug where rapid status reports and feed rates < 800 mm/min would occasionally cause input to lock up. (338.12)
* Removed config warning messages when in JSON mode. Warnings still echoed in text mode (command line mode) (338.12)

If you have feature requests or find any bugs please log them in the Issues tab on the github


========
BRANCHES
========

MASTER is the current production code. This is not updated very frequently.
EDGE branch is thought to be relatively stable, but has not received enough testing to be promoted to maser
DEV is work in process. THere are no gurantees that it will even run. This code from dev only if specificlly sent there
