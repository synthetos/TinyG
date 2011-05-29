pycam : Python CAM
------------------

Toolpath Generation for 3-Axis CNC machining


Features:

    * Cutters:
      ** Spherical Cutter
      ** Cylindrical Cutter
      ** Toroidal Cutter

    * Path Generators:
      ** DropCutter
      ** PushCutter

    * Path Processors:
      ** Linear
      ** ZigZag
      ** Polygon
      ** Contour        (aka. waterline)

    * Importers:
      ** STL
      ** DXF (only lines)
      ** SVG (paths for engravings)

    * Exporters:
      ** STL
      ** GCODE
      ** EMC2 tool defintion (tbl)

    * Visualization:
      ** OpenGL


RUNNING:

   extract the archive and run "python pycam"



USAGE:

   as a practical approach, you would probably: 
 
   1) for "rough" cutting,
      use the Cylindrical cutter 
      with the PushCutter Pathgenerator 
      and the Polygon PostProcessor in "x" or "y" mode 
      
 
   2) for "semifinish" cutting,
      use the Cylindrical/Toroidal cutter 
      with the PushCutter Pathgenerator 
      and the Contour PostProcessor in "xy" mode 

   3) "finish" cutting
      use the Spherical cutter 
      with the DropCutter Pathgenerator 
      and the ZigZag PostProcessor in "x" or "y" mode


TODO / OPEN ISSUES:
    see http://sourceforge.net/apps/mediawiki/pycam/index.php?title=TODO


FEATURE REQUESTS:

    * climbing / conventional cutting paths

    * diagonal paths

    * start at arbitrary corner


KNOWN BUGS:

    * The ODE collision detection sometimes fails at the corners of a
      model.
      http://sourceforge.net/tracker/?func=detail&aid=2973876&group_id=24884&atid=382799

    * Bugs in the sourceforge bug tracker:
      http://sourceforge.net/tracker/?group_id=237831&atid=1104176


CONTRIBUTORS:

    * Lode Leroy: initiated the project; developed the toolpath generation,
      collision detection, geometry, Tk interface, ...

    * Lars Kruse: developed the GTK interface; added ODE support

    * Arthur Magill: distutils packaging

    * Sebastian Kuzminsky: debian packaging

