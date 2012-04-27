<<<<<<< HEAD
TinyG Introduction
========

TinyG is a port of grbl to the Atmel xmega that runs on the TinyG hardware. Some differences are:

* 6 axis motion (XYXABC axes)
* jerk controlled motion for acceleration planning (3rd order motion planning)
* status displays ('?' character)
* XON/XOFF protocol over serila
* config is necessarily different to take into account the larger number of settings
=======
TinyG Edge Branch
========
The edge branch is the work-in-process for the next master release. 
Depending on the velocity of changes it's somewhere between alpha and beta. 
An attempt is made to keep everything working and somewhat usable, but there are no guarantees. 
Generally speaking, the edge branch should not be used for production uses - use the master branch instead.

What's currently in edge?
========
Edge is the staging area for version 0.93. So far the following has been implemented:

* Feedhold and restart (! and ~)
* System reset (^x)
* Homing cycle (G28.1)
* Return to home (G28)
* Spindle and coolant support
* Added 6 work coordinate systems (G54, G55, G56, G57, G58, G59) in addition to machine coordinates (G53)
* Enhanced G92 offsets
* Acceleration management added to arc motion (G2, G3)
* Enhanced status reporting supporting configuration of status report contents and reporting intervals
* Embedded self tests ($test command)
* Enhanced config system (type $h for details or see the [TinyG Wiki](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG))
* JSON support
* Extensive internal refactoring
* Bug fixes

![TinyG v6 Board](http://farm7.staticflickr.com/6080/6138119387_c6301797dd.jpg)
>>>>>>> de0c7e512fc471067e2ad077c08bbacebc3af5d1

See the Synthetos website for more details.

* [Synthetos](https://www.synthetos.com/)
* [TinyG Wiki](http://www.synthetos.com/wiki/index.php?title=Projects:TinyG)

<<<<<<< HEAD



CURRENT VERSION
========
The current stable version is 0.92 (Crocs)
This version has been used "in the field" now for a couple of months and is pretty stable.
A variety of issues from earlier version have been fixed.
If you find any bugs please log them in the Issues tab.


![TinyG v6 Board](http://farm7.staticflickr.com/6080/6138119387_c6301797dd.jpg)

=======
=======
=======
>>>>>>> de0c7e512fc471067e2ad077c08bbacebc3af5d1
