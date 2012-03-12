/* Test for loading a profile
 */
const char PROGMEM gcode_file[] = "\
(MSG#### Zen Toolworks 7x12 settings profile 001 ####)\n\
\n\
(Select Units Mode before loading settings: G20=inches, G21=mm)\n\
G21\n\
\n\
(MSGGeneral settings)\n\
$ja 50000		(Corner acceleration max)\n\
$hm 0			(Global setting to enable power-on-homing)\n\
\n\
(MSGMotor Settings)\n\
(Map motor 1-4 to X=0, Y=1, Z=2, A=3, B=4, C=5)\n\
$1ma 0\n\
$2ma 1\n\
$3ma 2\n\
$4ma 3\n\
\n\
(Step angle in degrees per *whole* step)\n\
$1sa 1.8\n\
$2sa 1.8\n\
$3sa 1.8\n\
$4sa 1.8\n\
\n\
(Microsteps setting: one of: 8, 4, 2, 1\n\
$1mi 4\n\
$2mi 4\n\
$3mi 4\n\
$4mi 4\n\
\n\
(Travel per revolution)\n\
$1tr 1.25		(Linear axes in linear units per revolution)\n\
$2tr 1.25\n\
$3tr 1.25\n\
$4tr 18			(Rotary axes in degrees per revolution)\n\
\n\
(Direction polarity: 0=normal, 1=inverted)\n\
(Conventions for XYZ table:\n\
(	X is left-to-right axis with +X moving toward the right)\n\
(	Y is front-to-back axis with +Y moving away from you - to the back)\n\
(	Z is the vertical and cutting axis with +Z moving upwards, away from the work)\n\
$1po 0\n\
$2po 1			(Y is inverted on my particular Zen)\n\
$3po 0\n\
$4po 0\n\
\n\
(Power mode: 0=steppers powered when idle, 1=Steppers not powered when idle)\n\
$1pw 0\n\
$1pw 0\n\
$1pw 0\n\
$1pw 0\n\
\n\
(MSGAxis Settings)\n\
(Axis mode)\n\
(	0 = disabled: axis will not be computed or run)\n\
(	1 = enabled: axis will be computed into coordinated motion and run)\n\
(	2 = inhibited: axis will be computed but not run - e.g. Z kill)\n\
(	3 = radius: rotary axis values will be scaled to radius value - rotary axes only)\n\
$xmo 1\n\
$ymo 1\n\
$zmo 2\n\
$amo 3\n\
$bmo 0			(not used, so don't factor into coordinated motion computations)\n\
$cmo 0			(not used...)\n\
\n\
(Axis radius setting - rotary axes only)\n\
(	Sets effective radius for onversion of linear uints to degrees)\n\
(	e.g. setting a 10mm radius yields 62.831 mm for one complete revolution - 360 degrees)\n\
$ara 10\n\
$bra 10\n\
$cra 10\n\
\n\
(Seek rate - Speeds for G0 traverses)\n\
$xsr 700		(in linear units per minute)\n\
$ysr 700\n\
$zsr 550\n\
$asr 12000		(in degrees per minute)\n\
$bsr 12000\n\
$csr 12000\n\
\n\
(Feed rate - Maximum allowable feed rates for G1, G2, G3 feeds)\n\
$xfr 700		(in linear units per minute)\n\
$yfr 700\n\
$zfr 550\n\
$afr 12000		(in degrees per minute)\n\
$bfr 12000\n\
$cfr 12000\n\
\n\
(Maximum jerk)\n\
$xjm 100,000,000	(in linear units per minute)\n\
$yjm 100,000,000\n\
$zjm 100,000,000\n\
$ajm 100,000,000\n\
$bjm 100,000,000\n\
$cjm 100,000,000\n\
\n\
(Corner delta)\n\
(	Works in conjunction with $ja to set maximum cornering velocity)\n\
(	Set smaller for slower cornering, larger for faster cornering)\n\
$xcd 0.06		(in mm)\n\
$ycd 0.06\n\
$zcd 0.06\n\
$acd 0.06\n\
$bcd 0.06\n\
$ccd 0.06\n\
\n\
(Travel hard limit)\n\
(	Travel between limit switches or crashes)\n\
(	Used to calibrate homing cycles)\n\
$xth 400\n\
$yth 175\n\
$zth 75\n\
$ath -1			(set to -1 to disable)\n\
$bth -1\n\
$cth -1\n\
\n\
(Travel soft limit)\n\
(	Travel range managed in software)\n\
(	*** NOT IMPLEMENTED YET ***)\n\
$xts 400\n\
$yts 175\n\
$zts 75\n\
$ats -1			(set to -1 to disable)\n\
$bts -1\n\
$cts -1\n\
\n\
(Switch modes)\n\
( 	1=limit switches present and enabled)\n\
$xli 1\n\
$yli 1\n\
$zli 1\n\
$ali 1\n\
$bli 1\n\
$cli 1\n\
\n\
(Homing settings)\n\
$xhe 0			(0=disabled, 1=enabled)\n\
$yhe 0\n\
$zhe 0\n\
$ahe 0\n\
$bhe 0\n\
$che 0\n\
\n\
$xho -200		(offset from X switch to X zero)\n\
$yho -85\n\
$zho 0\n\
$aho 0\n\
$bho 0\n\
$cho 0\n\
\n\
$xhr 700		(traverse rate for fast portion of homing)\n\
$yhr 700\n\
$zhr 550\n\
$ahr 12000\n\
$bhr 12000\n\
$chr 12000\n\
\n\
$xhc 10			(closing rate for slow portion of homing)\n\
$yhc 10\n\
$zhc 10\n\
$ahc 360\n\
$bhc 360\n\
$chc 360\n\
\n\
$xhb 5			(backoff distance)\n\
$yhb 5\n\
$zhb 5\n\
$ahb 5\n\
$bhb 5\n\
$chb 5\n";
