TINYG MASTER BRANCH
![TinyG v7 Board](http://farm9.staticflickr.com/8186/8436183471_6b3708cd0d_c.jpg)

TinyG is a 6 axis motion control system designed for high-performance on small to mid-sized machines. Some features:

* 6 axis motion (XYZABC axes)
* jerk controlled motion for acceleration planning
* status displays ('?' character)
* XON/XOFF and RTS/CTS protocol over USB serial
* RESTful interface using JSON

See these links for more details.

* [Synthetos](https://www.synthetos.com/)
* [TinyG Wiki](https://github.com/synthetos/TinyG/wiki)
* [TinyG Support Forum](https://www.synthetos.com/forum/tinyg/)
* [TinyG Github](https://github.com/synthetos/TinyG)
* [Synthetos Web Store](https://www.synthetos.com/webstore/)

**See also TinyG G2 [Arduino Due ARM Port](https://github.com/synthetos/g2) and the [G2 wiki](https://github.com/synthetos/g2/wiki)**

### BRANCHES

* MASTER is the current production code. This is not updated very frequently.

* EDGE branches are work-in-process for the next Master releases. Depending on the velocity of changes they will be somewhere between alpha and beta. An attempt is made to keep everything working and usable, but there are no guarantees for edge branches. Generally speaking, edge branches should not be used for production uses - use the master branch instead.

* Edge-0.97 is designed to be (mostly) backwards compatible with Master 440.xx. Changes are new features and additions that do not break current processing. However, there are a few changes that affect communication and other behaviors. These are listed below as Changes Affecting Migration from Master 440.xx to Edge-0.97.

* Edge-0.98 ports-in functionality available in the g2 code base. There are many changes that affect migration from Master 440.xx or edge-0.98. Please refer the the edge-0.98 readme or wiki for details.

#### SUMMARY OF CHANGES
Features added:
* `G10 L20` support
* `G43/G48` Tool Length Offset
* `ENQ` and `{enq:n}` behaviors for clear board connect sequence
* Linemode serial tranmission `{rxm:1}`
* Switch state readers `{inN:null}`
* JSON transaction IDs `{tid:NNNNN}`
* JSON text containers `{txt:"...."}`
* Incremental status report configuration `{srs:...}`
* Minimum length moves and minimum time moves are accepted and not reported as errors

#### CHANGES AFFECTING MIGRATION FROM MASTER TO EDGE-0.97
The following are changes between Master build 440.xx and Edge-0.97 that directly affect communications or operation:

* Revised the footer array and removed checksum from the response footer. See Line mode.
* Status code changes (error.h)
* Default flow control is now RTS/CTS, not XON/OFF


#### EDGE VERSION CHANGES
CHanges relative to master 440.20<br>
Many of these features are still in test, and some are still in development. See STATUS bullets for details.

* [Line mode ](https://github.com/synthetos/TinyG/wiki/Sending-Data-to-TinyG-using-RX-Packet-Mode)
  * Line mode permits buffer management on a per-line basis, significantly simplifying synchronization of tinyg with the host
  * {rxm:0} selects character mode, {rxm:1} selects line mode operation
  * STATUS: relatively stable


* Footer changes `{f[3,0,24]}`
  * The checksum has been removed from the footer (array element 4)
  * In character mode the footer revision (first array element) is set to `2`
  * In line mode the footer revision (first array element) is set to `3`
  * The second array element is still the status code
  * STATUS: stable


* Status Codes
  * There will be some changes here - new codes and some re-arrangement. Still need to update


* [Transaction ID {tid}](https://github.com/synthetos/TinyG/wiki/Text-Wrappers-and-Transaction-IDs)
  * The `tid` tag is a transaction ID and will be returned in the {r} response
  * STATUS: in test


* [Text Containers {txt}](https://github.com/synthetos/TinyG/wiki/Text-Wrappers-and-Transaction-IDs)
  * Text commands can be delivered wrapped in JSON, with their responses returned as text wrapped in JSON. This supports intermixing text mode commands in JSON communication. It is intended to simplify end-user command-line access with programmatic JSON.
  * STATUS: in test


* [Status Report settings](https://github.com/synthetos/TinyG/wiki/TinyG-Status-Reports#set-status-report-fields---firmware-build-44022-and-later)
  * Status reports can now be set and cleared incrementally. Individual elements can be added and removed without resetting the entire report string. This has become useful as more and more items are being requested in a status report. It allows a report specification to be built from multiple JSON lines, and avoids the problem of one line that may be too long for the system to handle.
  * STATUS: in test, relatively stable as of 446.xx


* JA Changes
  * A new cornering algorithm based entirely on jerk has been implemented. This is more accurate and higher performing that the approximation using centripetal acceleration.
  * `{ja:}` has changed meaning from `Junction Acceleration` to `Junction Aggression`. This is a unitless constant that is used in the integration of the jerk over the cornering distance. Reasonable values are from 0.25 to 2.00, with 0.75 being a good starting point. Higher numbers for faster cornering.
  * `{_jd}` (Junction Deviation) is no longer used and has been deprecated. It will be accepted for a phase out period, then removed.
  * STATUS: stable


* Arc handling


* Tool Length Offset


* G10 L20 (actually in earlier code)


* Default Flow Control
  * Has been changed from XON/XOFF to RTS/CTS
  * We have found RTS/CTS to be more universally and correctly implemented than XON/XOFF
