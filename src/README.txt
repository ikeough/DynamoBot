CatBot Release Notes:
--------------------

CatBot is a software for connecting Catia and Robot. It relies on specific parameters and specific labelling of objects in your Catia model.
Failing to provide the required parameters and analytical set structure may results in a crash, or an incomplete model.

It is highly recommended that the user establish a template file with all the requisite parameters and analytical set structures. 


-Parameters
	|
	|-Wind Load = -500		*All load values are in Newtons.
	|
	|-Live Load = -800
	|
	|-Slab Thickness = 200		*All thicknesses, diameters, and depths are in millimeters.
	|
	|-End Release = ffffffffffff	*Currently not implemented.
	|
	|-Tube Diameter 00 = 600	*Three possible tube sections (mm)
	|
	|-Tube Thickness 00 = 20
	|
	|-Tube Diameter 01 = 600
	|
	|-Tube Thickness 01 = 20
	|
	|-Tube Diameter 02 = 600
	|
	|-Tube Thickness 02 = 20
	|
	|-Plate Thickness 00 = 100	*Three possible plate sections (mm)
	|
	|-Plate Depth 00= 1200
	|
	|-Plate Thickness 01 = 100
	|
	|-Plate Depth 01= 1200
	|
	|-Plate Thickness 02 = 100
	|
	|-Plate Depth 02 = 1200
	|
	|-Displacement Tolerance = 12.7	*Adjustable displacement tolerance (mm)
	|
	|-Cable Radius = 101.6		


-MyGeometric Set			*This can be called anything, but "Analytical Set"(s) must be contained inside a top-level geom. set.
	|				*There can be as many analytical sets as you want, but they must each live in their own top-level set.
	|-Analytical Set		*An example of this would be powercopies, each with its own embedded analytical set.
		|
		|-Points
		|	|
		|	|-Pt1_FIXED
		|	|
		|	|-Pt2_FREE
		|
		|
		|-Lines
		|	|
		|	|-line1_TUBE00		*To create tube members.
		|	|
		|	|-line1_TUBE01		
		|	|
		|	|-line1_TUBE02		
		|	|
		|	|-line2_PLATE00		*To create plate members.
		|	|
		|	|-line2_PLATE01		
		|	|
		|	|-line2_PLATE02		
		|	|
		|	|-line3_CABLE		*To create cable members.
		|
		|-Plates
			|
			|-plate1_PANEL_WL	*PANEL - Indicates a single finite element wind panel
			|
			|-plate2_PANEL_COMB	*COMB - Indicates that an element gets wind and live loads
			|
			|-plate3_SLAB_LL	*LL - Indicates that an element gets live loads only.
			|
			|-plate2_SLAB_COMB	*WL - Indicates that an element gets wind loads only.
						
						**All plates must have SLAB/PANEL and WL/LL indications!


Command Line Syntax:
--------------------

In order to run CatBot from the command line, the syntax is as follows:

CatBot outputFile -o(-s) C:\OutputDirectory

CatBot - The name of the executable, which must be visible from the location where you are running it.

outputFile - The name of the output file. This can be anything of the user's choosing.

-o or -s These flags tell CatBot whether to overwrite (-o) existing Robot files of the same name, or save (-s) a sequentially numbered version  
of the Robot file.

C:\OutputDirectory - This is the path to the folder where you want your Robot and text results file written.


A Description of the Analytical Components:
-------------------------------------------

Nodes:

Nodes (points in Catia) must be point objects. They can be created in any way


Analytical Checks Performed at Run-time:
----------------------------------------
1. Nodal Displacement check - The number of nodal displacements in the global Z direction greater than 1/2" are recorded in the "Nodal Displacement Failures" section of the results file.
2. Bending stress failures - The number of bars that fail bending stress checks per ASD are recorded in the "Bending Stress Failures," section of the results file.
3. Static, Non-Linear analysis of structure.


TODO:
-----
1. Add code check for buckling before analysis.
 



