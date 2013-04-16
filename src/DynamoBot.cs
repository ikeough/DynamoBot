/*DynamoBot - Linking Dynamo and Robot
Copyright (C) 2013  Ian Keough (ian@iankeough.com)
This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.*/

using System;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Windows.Forms;
using Dynamo.Connectors;
using Dynamo.Nodes;
using Microsoft.FSharp.Collections;
using RobotOM;
using Value = Dynamo.FScheme.Value;

namespace Dynamo.Analysis
{
	public enum SectionType : byte
	{
		tube00 	= 1,
		tube01 	= 2,
		tube02 	= 3,
		plate00 = 4,
		plate01 = 5,
		plate02 = 6,
		cable 	= 7
	}
	
	public enum PlateType : byte
	{
		slab = 1,
		panel = 2,
		shear = 3
	}
	
	class DynamoBot
	{
		static IRobotApplication m_robot;
		static IRobotNodeServer m_nodes;
		static IRobotBarServer m_bars;
		static IRobotObjObjectServer m_objects;
		static IRobotProject m_project;
		static IRobotStructure m_structure;
		static IRobotResultServer m_results;
		static IRobotNodeDisplacementServer m_displacements;
		static IRobotCaseServer m_cases;
		static IRobotBarStressServer m_stresses;

		static List<Bar> 	m_barInfo;
		static List<Node> 	m_nodeInfo;
		static List<Plate> 	m_plateInfo;
		static double[] m_sectionInfo = new double[13];
		
		static string m_xmlPath;
		static string m_robotPath;
		static string m_outputPath;
		static string m_saveMode;
		
		static List<int> m_failedNodes = new List<int>();
		static List<int> m_failedBars = new List<int>();
		static double m_displacementTolerance = 0.0;	//13mm mm->M
		static double m_stressTolerance = .6 * 248211262.8;	//36ksi(steel) -> Pascal * .6 (per ASD)
		static double m_windx, m_windy, m_windz;	//the wind vector
		
        //static SPATypeLib.SPAWorkbench m_spaWorkBench;
        //static SPATypeLib.Measurable m_meas;
        //static SPATypeLib.Measurable m_meas_pt;
		
		static string m_errorMessage = "";
		
		[STAThreadAttribute]
		
		public static void Main(string[] args)
		{
			
			#region write license
			Console.WriteLine("*******************************************************");
			Console.WriteLine("CatBot  Copyright (C) 2009  Ian Keough (ian@iankeough.com)");
			Console.WriteLine("This program comes with ABSOLUTELY NO WARRANTY; see ");
			Console.WriteLine("http://www.gnu.org/licenses/gpl-3.0.txt for details.");
			Console.WriteLine("This is free software, and you are welcome to redistribute it");
			Console.WriteLine("under certain conditions; see");
			Console.WriteLine("http://www.gnu.org/licenses/gpl-3.0.txt for details.");
			Console.WriteLine("*******************************************************");
	
			#endregion
			
			#region parse command line args
			//parse the command line arguments
			//get the file name and the runmode

			if (args.Length < 3 || args.Length > 3)	//there are too many or too few args
			{
				MessageBox.Show("You must input file name, run mode, directory, and save mode arguments!!");
				System.Threading.Thread.Sleep(3000);
				return;
			}
			
			string fileName = 			args[0];
			m_saveMode =				args[1];
			string currDirectory =  	args[2];

			#endregion parse command line args
			
			#region set file paths
			
			//create the primary file paths
			
			switch(m_saveMode)
			{
				case "-s":	//save a new file
					
					int saveCount = 0;
					bool saved = false;
					
					while (!saved)
					{
						if (System.IO.File.Exists(currDirectory + "\\" + fileName + "_" + saveCount.ToString() + ".rtd"))
						{
						    saveCount++;
						}
						else
						{
							m_robotPath = currDirectory + "\\" + fileName + "_" + saveCount.ToString() + ".rtd";
							saved = true;
						}
					}
					
					break;
					
				case "-o":  //overwrite the existing robot file
					m_robotPath = 	currDirectory + "\\" + fileName + ".rtd";
					break;
					
			}
			
			m_xmlPath = 	currDirectory + "\\" + fileName + ".xml";
			
			m_outputPath=	currDirectory + "\\" + fileName + ".dat";

			#endregion set file paths
			
			try
			{
			System.Diagnostics.Stopwatch sw = new Stopwatch();
			sw.Start();

			m_barInfo = new List<Bar>();
			m_nodeInfo = new List<Node>();
			m_plateInfo = new List<Plate>();

			if (!ReadFromCatia(ref m_nodeInfo, ref m_barInfo, ref m_plateInfo))
			{
				Console.WriteLine(m_errorMessage);
				System.Threading.Thread.Sleep(3000);
				return;
			}

			if (m_nodeInfo.Count == 0)
			{
				Console.WriteLine("No node information could be returned.");
				//System.Threading.Thread.Sleep(5000);
				return;
			}
			
			
			//Initialize Robot
			if (!InitializeRobot())
			{
				Console.WriteLine("Robot could not be initialized. Please make sure the application is available.");
				//System.Threading.Thread.Sleep(5000);
				return;
			}
			
			if(!CreateRobotModel(ref m_nodeInfo, ref m_barInfo, ref m_plateInfo, m_robotPath))
			{
				Console.WriteLine("The Robot model could not be created. Check std out for errors.");
				return;
			}
			
			if(!AnalyzeAndWriteResults(currDirectory, fileName))
			{
				Console.WriteLine("Robot model could not be analyzed. Check std out for errors.");
				return;
			}
			
			GC.Collect();
			
			sw.Stop();
			//System.Diagnostics.Debug.WriteLine("Loop finished in " + sw.Elapsed.ToString());

			return;
			}
			catch(Exception e)
			{
//				MessageBox.Show(e.InnerException + ":" + e.StackTrace);
				Console.WriteLine(e.InnerException + ":" + e.StackTrace);
				Debug.WriteLine(e.Message + ":" + e.InnerException + ":" + e.StackTrace);
			}
			finally
			{
				System.Diagnostics.Process[] processes = System.Diagnostics.Process.GetProcessesByName("robot");
				foreach(Process p in processes)
				{
					p.Kill();
				}
			}
		}

		public static bool ReadFromCatia(ref List<Node> _nodeInfo, ref List<Bar> _barInfo, ref List<Plate> _plateInfo)
		{
			INFITF.Application catApp = System.Runtime.InteropServices.Marshal.GetActiveObject("CATIA.Application") as INFITF.Application;
			MECMOD.PartDocument catDoc = catApp.ActiveDocument as PartDocument;
			MECMOD.Part part 				= catDoc.Part;
			Parameters	catParams			= part.Parameters; 
			
			string workbenchName = "SPAWorkbench";
			m_spaWorkBench = catDoc.GetWorkbench(ref workbenchName) as SPATypeLib.SPAWorkbench;
			
			string analyticalSetName		= "Analytical Set";
			string pointSetName				= "Points";
			string lineSetName				= "Lines";
			string plateSetName				= "Plates";
			
			int fixity = 0;
			string windLoadName 	= "Wind Load";
			string liveLoadName		= "Live Load";
			string buildLoadName	= "Building Load";
			string slabThickName 	= "Slab Thickness";
			string endRelName		= "End Release";
			string displaceTolName	= "Displacement Tolerance";
			string cableRadiusName	= "Cable Radius";
			
			//multiple section types 
			//future releases will have a more robust version of this
			//type 1
			string tubeDiamName_00		= "Tube Diameter 00";
			string tubeThickName_00		= "Tube Thickness 00";
			string tubeDiamName_01		= "Tube Diameter 01";
			string tubeThickName_01		= "Tube Thickness 01";
			string tubeDiamName_02		= "Tube Diameter 02";
			string tubeThickName_02		= "Tube Thickness 02";
			
			string plateThickName_00	= "Plate Thickness 00";
			string plateDepthName_00	= "Plate Depth 00";
			string plateThickName_01	= "Plate Thickness 01";
			string plateDepthName_01	= "Plate Depth 01";
			string plateThickName_02	= "Plate Thickness 02";
			string plateDepthName_02	= "Plate Depth 02";
			
			string windXName = "Wind X";
			string windYName = "Wind Y";
			string windZName = "Wind Z";
			
			string windValue, liveValue, slabThickValue, endRelValue, displaceTolerance;
			string tubeDiamValue_00, tubeDiamValue_01, tubeDiamValue_02;
			string tubeThickValue_00, tubeThickValue_01, tubeThickValue_02;
			string plateThickValue_00, plateThickValue_01,plateThickValue_02;
			string plateDepthValue_00, plateDepthValue_01, plateDepthValue_02;
			string cableRadiusValue;
			string windxValue, windyValue, windzValue;
			string buildLoadValue;
			
			double windLoad, liveLoad, slabThick, displaceTolVal;
			double tubeDiam_00, tubeDiam_01, tubeDiam_02;
			double tubeThick_00, tubeThick_01, tubeThick_02;
			double plateThick_00, plateThick_01, plateThick_02;
			double plateDepth_00, plateDepth_01, plateDepth_02;
			double cableRadius;
			double buildLoad;
			
			try
			{
				windValue 			= (catParams.GetItem(ref windLoadName) as Parameter).ValueAsString();
				liveValue 			= (catParams.GetItem(ref liveLoadName) as Parameter).ValueAsString();
				slabThickValue		= (catParams.GetItem(ref slabThickName) as Parameter).ValueAsString();
				endRelValue 		= (catParams.GetItem(ref endRelName) as Parameter).ValueAsString();
				buildLoadValue		= (catParams.GetItem(ref buildLoadName) as Parameter).ValueAsString();
				
				tubeDiamValue_00 	= (catParams.GetItem(ref tubeDiamName_00) as Parameter).ValueAsString();
				tubeThickValue_00 	= (catParams.GetItem(ref tubeThickName_00) as Parameter).ValueAsString();
				tubeDiamValue_01 	= (catParams.GetItem(ref tubeDiamName_01) as Parameter).ValueAsString();
				tubeThickValue_01 	= (catParams.GetItem(ref tubeThickName_01) as Parameter).ValueAsString();
				tubeDiamValue_02 	= (catParams.GetItem(ref tubeDiamName_02) as Parameter).ValueAsString();
				tubeThickValue_02 	= (catParams.GetItem(ref tubeThickName_02) as Parameter).ValueAsString();
				
				plateThickValue_00 	= (catParams.GetItem(ref plateThickName_00) as Parameter).ValueAsString();
				plateDepthValue_00 	= (catParams.GetItem(ref plateDepthName_00) as Parameter).ValueAsString();
				plateThickValue_01 	= (catParams.GetItem(ref plateThickName_01) as Parameter).ValueAsString();
				plateDepthValue_01 	= (catParams.GetItem(ref plateDepthName_01) as Parameter).ValueAsString();
				plateThickValue_02 	= (catParams.GetItem(ref plateThickName_02) as Parameter).ValueAsString();
				plateDepthValue_02 	= (catParams.GetItem(ref plateDepthName_02) as Parameter).ValueAsString();
				
				displaceTolerance 	= (catParams.GetItem(ref displaceTolName) as Parameter).ValueAsString();
				cableRadiusValue 	= (catParams.GetItem(ref cableRadiusName) as Parameter).ValueAsString();
				
				windxValue = (catParams.GetItem(ref windXName) as Parameter).ValueAsString();
				windyValue = (catParams.GetItem(ref windYName) as Parameter).ValueAsString();
				windzValue = (catParams.GetItem(ref windZName) as Parameter).ValueAsString();
					
				windLoad 			= Convert.ToDouble(windValue);
				liveLoad 			= Convert.ToDouble(liveValue);
				slabThick			= Convert.ToDouble(slabThickValue);
				
				tubeDiam_00			= Convert.ToDouble(tubeDiamValue_00);
				tubeThick_00		= Convert.ToDouble(tubeThickValue_00);
				tubeDiam_01			= Convert.ToDouble(tubeDiamValue_01);
				tubeThick_01		= Convert.ToDouble(tubeThickValue_01);
				tubeDiam_02			= Convert.ToDouble(tubeDiamValue_02);
				tubeThick_02		= Convert.ToDouble(tubeThickValue_02);
				
				plateThick_00		= Convert.ToDouble(plateThickValue_00);
				plateDepth_00		= Convert.ToDouble(plateDepthValue_00);
				plateThick_01		= Convert.ToDouble(plateThickValue_01);
				plateDepth_01		= Convert.ToDouble(plateDepthValue_01);
				plateThick_02		= Convert.ToDouble(plateThickValue_02);
				plateDepth_02		= Convert.ToDouble(plateDepthValue_02);

				displaceTolVal 		= Convert.ToDouble(displaceTolerance);
				cableRadius			= Convert.ToDouble(cableRadiusValue);
				
				m_sectionInfo[0] = tubeDiam_00;
				m_sectionInfo[1] = tubeThick_00;
				m_sectionInfo[2] = tubeDiam_01;
				m_sectionInfo[3] = tubeThick_01;
				m_sectionInfo[4] = tubeDiam_02;
				m_sectionInfo[5] = tubeThick_02;
				
				m_sectionInfo[6] = plateDepth_00;
				m_sectionInfo[7] = plateThick_00;
				m_sectionInfo[8] = plateDepth_01;
				m_sectionInfo[9] = plateThick_01;
				m_sectionInfo[10] = plateDepth_02;
				m_sectionInfo[11] = plateThick_02;
				
				m_sectionInfo[12] = cableRadius;

				m_displacementTolerance = displaceTolVal;
				
				m_windx = Convert.ToDouble(windxValue);
				m_windy = Convert.ToDouble(windyValue);
				m_windz = Convert.ToDouble(windzValue);
				
				buildLoad = Convert.ToDouble(buildLoadValue);
				
			}
			catch
			{
				m_errorMessage = ("One of your CATIA variables is missing or has an invalid value." + "\n" + 
				                 "Please check the ReadMe.txt file for a description of parameters.");
				return false;
			}
			
			
			SectionType secType;
			
			//variables for the plate support
			Reference plElement;
			Length length;
			
			HybridBody bodySet				= null;	//the top level set that will contain everything else.
			HybridBody analyticalSet 		= null;	//the inner analytical set
			
			for (int k=1; k<=part.HybridBodies.Count; k++)
			{
				object outerIndex = k as object;
				
				bodySet 						= part.HybridBodies.Item(ref outerIndex);
				
				try{
					analyticalSet 		= bodySet.HybridBodies.GetItem(ref analyticalSetName) as HybridBody;	//add for geomSet support 081031
					if (analyticalSet.HybridBodies.Count == 0)
					{
						continue;	//go to next analytical set
					}
				}
				catch{
					//catia API throws error right away if the hybrid body isn't found
					continue;
				}

				HybridBody pointSet				= analyticalSet.HybridBodies.GetItem(ref pointSetName) as HybridBody;
				HybridBody lineSet				= analyticalSet.HybridBodies.GetItem(ref lineSetName)as HybridBody;
				HybridBody plateSet				= analyticalSet.HybridBodies.GetItem(ref plateSetName)as HybridBody;
				
				Point point = null;
				HybridShapeLinePtPt l = null;
				HybridShapePolyline pl = null;

				object[] coords = new object[3];
				
				Console.WriteLine("Getting the active instance of Catia...");
				
				string catiaStartId;
				string catiaEndId;
				string catiaId;
				int robotId = 0;
				double posX, posY, posZ; 
				double displaceX = 0.0;
				double displaceY = 0.0;
				double displaceZ = 0.0;
				int robotStartId 	= 0;
				int robotEndId		= 0;
				double axial		= 0.0;
			
			
			#region nodes
			
			Console.WriteLine("Listing nodes...");
			
			for (int i=1; i<=pointSet.HybridShapes.Count; i++)
			{
				//create all the fixed nodes and add them to the list
				object index = i as object;
				
				try
				{
					point		= pointSet.HybridShapes.Item(ref index) as Point;
					catiaId 	= pointSet.HybridShapes.Item(ref index).get_Name();
					point.GetCoordinates(coords);
					posX 		= Convert.ToDouble(coords[0]) / 1000;
					posY 		= Convert.ToDouble(coords[1]) / 1000;
					posZ		= Convert.ToDouble(coords[2]) / 1000;
					
					if (catiaId.Contains("FIXED"))
					{
						fixity = 1;
					}
					else if (catiaId.Contains("FREE"))
					{
						fixity = 0;
					}
					
				}
				catch
				{
					Console.WriteLine("Some elements in the set were not points. Please reorganize and try again...");
					return false;
				}
				
				//create the node object
				Node n = new Node(catiaId, robotId, posX, posY, posZ, displaceX, displaceY, displaceZ, fixity);
				_nodeInfo.Add(n);
				
				Console.WriteLine("Node " + catiaId + " written to list...");
				
			}
			
			#endregion

			#region bars
			
			Console.WriteLine("Listing bars...");
			
			catiaId = "";
			catiaEndId = "";
			catiaStartId = "";
			
			object[] lineCoords = new object[9];
			double startX, startY, startZ;
			double endX, endY, endZ;
			
			int datumCount = 0;
			for (int i=1; i<=lineSet.HybridShapes.Count; i++)
			{
				try
				{
					object index = i as object;
					
					l = lineSet.HybridShapes.Item(ref index) as HybridShapeTypeLib.HybridShapeLinePtPt;
					
					//THIS CODE WAS ADDED TO DEAL WITH UDFS CREATING EXPLICIT GEOMETRY
					if(l == null)
					{
						datumCount ++;
						Debug.WriteLine(datumCount.ToString());
						Line ldat = lineSet.HybridShapes.Item(ref index) as Line;
						Reference lineRef = part.CreateReferenceFromObject(lineSet.HybridShapes.Item(ref index));
						m_meas = m_spaWorkBench.GetMeasurable(lineRef);
						m_meas.GetPointsOnCurve(lineCoords);	//get the start middle and end points
						
						catiaId = ldat.get_Name();
						
						//find the corresponding node in the _nodeInfo
						startX 		= Convert.ToDouble(lineCoords[0]) / 1000;
						startY 		= Convert.ToDouble(lineCoords[1]) / 1000;
						startZ		= Convert.ToDouble(lineCoords[2]) / 1000;
						endX 		= Convert.ToDouble(lineCoords[6]) / 1000;
						endY 		= Convert.ToDouble(lineCoords[7]) / 1000;
						endZ		= Convert.ToDouble(lineCoords[8]) / 1000;
						
						Debug.WriteLine("Start:" + startX.ToString() + "," + startY.ToString() + "," + startZ.ToString());
						Debug.WriteLine("End:" + endX.ToString() + "," + endY.ToString() + "," + endZ.ToString());
						
						int foundCount = 0;
						foreach(Node n in _nodeInfo)
						{
		
							
//							if(n.X.Equals(startX) && n.Y.Equals(startY) && n.Z.Equals(startZ))
//							{
//								catiaStartId = n.catiaID;
//								foundCount++;
//							}
//							else if(n.X.Equals(endX) && n.Y.Equals(endY) && n.Z.Equals(endZ))
//							{
//								catiaEndId = n.catiaID;
//								foundCount++;
//							}
							
							if(IsAlmostEqualTo(n.X, startX) && IsAlmostEqualTo(n.Y,startY) && IsAlmostEqualTo(n.Z,startZ)){
							   	catiaStartId = n.catiaID;
								foundCount++;
							}
							if(IsAlmostEqualTo(n.X, endX) && IsAlmostEqualTo(n.Y,endY) && IsAlmostEqualTo(n.Z,endZ)){
							   	catiaEndId = n.catiaID;
								foundCount++;
							}
							
							if(foundCount == 2)
							{
								//you've found the start and the end
								break;
							}
						}

					}
					else{
						catiaId 		= l.get_Name();
						catiaStartId 	= l.PtOrigine.DisplayName;
						catiaEndId		= l.PtExtremity.DisplayName;
					}

				}
				catch
				{
					Console.WriteLine("Some elements in the set were not lines. Please reorganize and try again...");
					return false;
				}
				try{
					if (catiaId.Contains("TUBE00"))
					{
						secType = SectionType.tube00;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[0], secType, m_sectionInfo[1], endRelValue);
						_barInfo.Add(b);
					}
					else if (catiaId.Contains("TUBE01"))
					{
						secType = SectionType.tube01;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[2], secType, m_sectionInfo[3], endRelValue);
						_barInfo.Add(b);
					}
					else if (catiaId.Contains("TUBE02"))
					{
						secType = SectionType.tube02;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[4], secType, m_sectionInfo[5], endRelValue);
						_barInfo.Add(b);
					}
					else if(catiaId.Contains("PLATE00"))
					{
						secType = SectionType.plate00;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[6], secType, m_sectionInfo[7], endRelValue);
						_barInfo.Add(b);
					}
					else if(catiaId.Contains("PLATE01"))
					{
						secType = SectionType.plate01;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[8], secType, m_sectionInfo[9], endRelValue);
						_barInfo.Add(b);
					}
					else if(catiaId.Contains("PLATE02"))
					{
						secType = SectionType.plate02;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, m_sectionInfo[10], secType, m_sectionInfo[11], endRelValue);
						_barInfo.Add(b);
					}
					else if(catiaId.Contains("CABLE"))
					{
						secType = SectionType.cable;
						Bar b = new Bar(catiaId, robotId, catiaStartId, catiaEndId, robotStartId, robotEndId, axial, 0.0, secType, 0.0, endRelValue);
						_barInfo.Add(b);
					}
					else
					{
						MessageBox.Show("Please define your section type as 'bar' or 'plate' before running CatBot.");
						return false;
					}
				}
				catch{
					Console.WriteLine("Some datum line geometry does not have corresponding points.");
					return false;
				}

				Console.WriteLine("Bar " + catiaId + " written to list...");
			}
			#endregion
			
			#region plates
			Console.WriteLine("Listing plates...");
			
			if(plateSet.HybridShapes.Count > 0)
			{
				for (int i=1; i<=plateSet.HybridShapes.Count; i++)
				{
					try
					{
						List<string> catiaPoints = new List<string>();
						List<int> robotPoints = new List<int>();

						object index = i as object;
						pl = plateSet.HybridShapes.Item(ref index) as HybridShapeTypeLib.HybridShapePolyline;
						
						catiaId = pl.get_Name();
						
						for (int j=1; j<=pl.NumberOfElements; j++)
						{
							pl.GetElement(j,out plElement, out length );
							catiaPoints.Add(plElement.DisplayName);
							robotPoints.Add(0);
						}
						
						double[] loadList = new double[3]{0.0,0.0,0.0};
						if(catiaId.Contains("WL"))
						{
							loadList[0] = windLoad;
						}
						else if (catiaId.Contains("LL"))
						{
							loadList[1] = liveLoad;
						}
						else if (catiaId.Contains("COMB"))
						{
							loadList[0] = windLoad;
							loadList[1] = liveLoad;
						}
						else if (catiaId.Contains("BL")){
							loadList[2] = buildLoad;
						}
						
						Plate p = new Plate(catiaId, robotId, catiaPoints, robotPoints, loadList, slabThick);
						
						if (catiaId.Contains("SLAB"))
						{
							p.PlateType = PlateType.slab;
						}
						else if (catiaId.Contains("PANEL"))
						{
							p.PlateType = PlateType.panel;
						}
						else if(catiaId.Contains("SHEAR"))
						{
							p.PlateType = PlateType.shear;	
						}
						
						_plateInfo.Add(p);
						
						Console.WriteLine("Plate " + catiaId + " written to list...");
					}
					catch
					{
						Console.WriteLine("Some elements in the set were not polylines. Please reorganize and try again...");
						return false;
					}
				}
			}
			
			#endregion
			}
			return true;
		}
		
		private static bool IsAlmostEqualTo(double a, double b)
		{
			if (b < a + .001 && b > a - .001){
				return true;
			}
			else return false;
		}
		/// <summary>
		/// Initialize Robot
		/// </summary>
		/// <returns>True or false</returns>
		public static bool InitializeRobot()
		{
			Console.WriteLine("Initializing Robot...");
			m_robot = new RobotApplication();
			
			System.Threading.Thread.Sleep(20000);
			Console.WriteLine("Waiting for Robot...");
			
			if (m_robot != null)
			{
//				m_robot.Visible = 0; //robot visible
//				m_robot.Interactive = 1;
				
				
				return true;
			}
			else
			{
				MessageBox.Show("Robot can not be initialized.");
				return false;
			}

		}
		
		/// <summary>
		/// Create the Robot model from scratch.
		/// </summary>
		/// <param name="_nodeInfo">A List of node objects.</param>
		/// <param name="_barInfo">A List of bar objects.</param>
		/// <param name="_filePath">The Robot model file path</param>
		public static bool CreateRobotModel(ref List<Node> _nodeInfo, ref List<Bar> _barInfo,ref List<Plate> _plateInfo, string _filePath)
		{

			m_robot.Project.New(RobotOM.IRobotProjectType.I_PT_SHELL);
			
			m_project = m_robot.Project;
			if (m_project == null)
			{
				Console.WriteLine("The robot project could not be initialized.");
				System.Threading.Thread.Sleep(5000);
				return false;
			}
			
			//create the structure and the node and bar servers
			m_structure = m_project.Structure;
			m_nodes = m_structure.Nodes;
			m_bars = m_structure.Bars;	
			m_objects = m_structure.Objects;
			
			List<int> forWindLoads = new List<int>();
			List<int> forLiveLoads = new List<int>();
			List<int> forBuildingLoads = new List<int>();
			List<int> forDeadLoadsBars = new List<int>();
			List<int> forDeadLoadsSlabs = new List<int>();
			
			double windLoad	= 0.0;
			double liveLoad = 0.0;
			double buildingLoad = 0.0;
			double fatManLoad = 300 * 4.45;	//300lbs. -> Newton
			
			//create load cases
			IRobotSimpleCase dl = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_DL", IRobotCaseNature.I_CN_PERMANENT, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
			IRobotSimpleCase ll = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_LL", IRobotCaseNature.I_CN_EXPLOATATION, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
			IRobotSimpleCase wl = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_WL", IRobotCaseNature.I_CN_WIND, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
			IRobotSimpleCase bl = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_BL", IRobotCaseNature.I_CN_EXPLOATATION, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
			
//			IRobotSimpleCase sl = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_SL", IRobotCaseNature.I_CN_SNOW, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
//			IRobotSimpleCase wl_uplift = m_structure.Cases.CreateSimple(m_structure.Cases.FreeNumber, "Catia_WL_Uplift", IRobotCaseNature.I_CN_WIND, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
			IRobotCaseCombination comb1 = m_structure.Cases.CreateCombination(m_structure.Cases.FreeNumber, "Catia_DL+LL+WL+BL", IRobotCombinationType.I_CBT_ACC, IRobotCaseNature.I_CN_PERMANENT, IRobotCaseAnalizeType.I_CAT_COMB);
			IRobotCaseCombination comb2 = m_structure.Cases.CreateCombination(m_structure.Cases.FreeNumber, "Catia_DL_60+WL", IRobotCombinationType.I_CBT_ACC, IRobotCaseNature.I_CN_PERMANENT, IRobotCaseAnalizeType.I_CAT_COMB);
			
			RobotCaseFactorMngr caseManage1 = comb1.CaseFactors;
			caseManage1.New(1, 1.0);
			caseManage1.New(2, 1.0);
			caseManage1.New(3, 1.0);
			caseManage1.New(4, 1.0);	//add the building load
			
			RobotCaseFactorMngr caseManage2 = comb2.CaseFactors;
			caseManage2.New(1, 0.6);
			caseManage2.New(2, 1.0);
			caseManage2.New(3, 1.0);
			
			//define planar loads to be applied to the panels and slabs
			ll.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//record 1 - slab loads
			ll.Records.New(IRobotLoadRecordType.I_LRT_NODE_FORCE);	//record 2 - Fat man load
			wl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//record 3 - plate loads
//			wl_uplift.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//record 1 
			dl.Records.New(IRobotLoadRecordType.I_LRT_DEAD);	//record 4 see pg. 150, 160 Robot API docs
			//sl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//snow load for panels and slabs
			bl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//record 5 - building load
			
			//parse the end release
			if(_barInfo[0].EndRelease.Length != 12)
			{
				MessageBox.Show("End Release value must be 12 values formatted as 'x'(free) and 'f'(fixed)");	
				return false;
			}
				
			//create a bar end release
			IRobotLabel rel1 = m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_RELEASE, "CatiaBarRelease");
			IRobotBarReleaseData relData1 = rel1.Data as IRobotBarReleaseData;
			IRobotBarEndReleaseData relStartData1 	= relData1.StartNode;
			IRobotBarEndReleaseData relEndData1 	= relData1.EndNode;
			
			IRobotBarEndReleaseValue[] relVals = new IRobotBarEndReleaseValue[12];
			char[] letters = _barInfo[0].EndRelease.ToCharArray();

			for (int i=0; i<_barInfo[0].EndRelease.Length; i++)
			{
				if (letters[i].ToString() == "f")
				{
					relVals[i] = IRobotBarEndReleaseValue.I_BERV_FIXED; 
				}
				else if (letters[i].ToString() == "x")
				{
					relVals[i] = IRobotBarEndReleaseValue.I_BERV_NONE;
				}
			}
			
			relStartData1.UX 	= relVals[0];
			relStartData1.UY 	= relVals[1];
			relStartData1.UZ 	= relVals[2];
			relStartData1.RX	= relVals[3];
			relStartData1.RY	= relVals[4];
			relStartData1.RZ	= relVals[5];
			relEndData1.UX		= relVals[6];
			relEndData1.UY		= relVals[7];
			relEndData1.UZ		= relVals[8];
			relEndData1.RX		= relVals[9];
			relEndData1.RY		= relVals[10];
			relEndData1.RZ		= relVals[11];
			
			m_structure.Labels.Store(rel1);
			
			#region nodes
			
			IRobotLabel sup = m_structure.Labels.Create(IRobotLabelType.I_LT_SUPPORT, "CatiaSupport");
			IRobotNodeSupportData sup_data = sup.Data as IRobotNodeSupportData;
			
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UX, 1);
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UY, 1);
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UZ, 1);
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RX, 0);
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RY, 0);
			sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RZ, 0);
			
			m_structure.Labels.Store(sup);
				
			int nodeId = 100;
			for (int i=0; i<_nodeInfo.Count; i++)
			{
				Console.WriteLine("Now creating " + nodeId.ToString() + "...");
				Node n = (Node)_nodeInfo[i];

				//create the robot node
				m_nodes.Create(nodeId, n.X, n.Y, n.Z);
				
				if (n.Fixed == 1)
				{
					
					Console.WriteLine("Fixing node " + nodeId.ToString() + "...");
					IRobotNode node = m_nodes.Get(nodeId) as IRobotNode;
					node.SetLabel(IRobotLabelType.I_LT_NODE_SUPPORT, "CatiaSupport");
					
				}
				
				//set the robot node information in the List
				//to be pushed back into the XML
				n.robotID = nodeId;		//the robot node Id
				n.displaceX = 0.0;
				n.displaceY = 0.0;
				n.displaceZ = 0.0;
				
				nodeId++;
			}
			#endregion
			
			#region bars
			
			//create the tube section
			//type 1
			IRobotLabel sec1 			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_00");
			IRobotBarSectionData data1 	= sec1.Data as IRobotBarSectionData;
			data1.Type 					= IRobotBarSectionType.I_BST_NS_TUBE;
			data1.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_TUBE;
			IRobotBarSectionNonstdData nonst_data1 = data1.CreateNonstd(0);
			
			//type 2
			IRobotLabel sec2 			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_01");				
			IRobotBarSectionData data2 	= sec2.Data as IRobotBarSectionData;
			data2.Type 					= IRobotBarSectionType.I_BST_NS_TUBE;
			data2.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_TUBE;
			IRobotBarSectionNonstdData nonst_data2 = data2.CreateNonstd(0);
			
			//type 3
			IRobotLabel sec3 			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_02");				
			IRobotBarSectionData data3 	= sec3.Data as IRobotBarSectionData;
			data3.Type 					= IRobotBarSectionType.I_BST_NS_TUBE;
			data3.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_TUBE;
			IRobotBarSectionNonstdData nonst_data3 = data3.CreateNonstd(0);
			
			//create the plate section
			IRobotLabel sec4 			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_00");				
			IRobotBarSectionData data4 	= sec4.Data as IRobotBarSectionData;
			data4.Type 					= IRobotBarSectionType.I_BST_NS_RECT;
			data4.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_RECT;
			IRobotBarSectionNonstdData nonst_data4 = data4.CreateNonstd(0);
			
			IRobotLabel sec5			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_01");				
			IRobotBarSectionData data5 	= sec5.Data as IRobotBarSectionData;
			data5.Type 					= IRobotBarSectionType.I_BST_NS_RECT;
			data5.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_RECT;
			IRobotBarSectionNonstdData nonst_data5 = data5.CreateNonstd(0);
			
			IRobotLabel sec6 			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_02");				
			IRobotBarSectionData data6 	= sec6.Data as IRobotBarSectionData;
			data6.Type 					= IRobotBarSectionType.I_BST_NS_RECT;
			data6.ShapeType 			= IRobotBarSectionShapeType.I_BSST_USER_RECT;
			IRobotBarSectionNonstdData nonst_data6 = data6.CreateNonstd(0);
			
			//create a cable section
			IRobotLabel sec7			= m_structure.Labels.Create(IRobotLabelType.I_LT_BAR_CABLE, "CatiaCable");
			IRobotBarCableData data7 	= sec7.Data as IRobotBarCableData;
			data7.SectionAX				= Math.PI * (Math.Pow((m_sectionInfo[4]/1000),2));		//the area formula using the cable radius parameter
			data7.MaterialName			= "STEEL";
			
			if (_barInfo.Count != 0)
			{
//				double sectionDiameter 	= _barInfo[0].Diameter/1000;			//the diameter in meters
//				double sectionthickness	= _barInfo[0].SectionThickness/1000;	//the thickness in meters
				
				//set the values of the tubes	
				nonst_data1.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, m_sectionInfo[0]/1000);	//the section diameter
				nonst_data1.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, m_sectionInfo[1]/1000);	//the section thickness
				data1.CalcNonstdGeometry();
				m_structure.Labels.Store(sec1);
				
				nonst_data2.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, m_sectionInfo[2]/1000);	//the section diameter
				nonst_data2.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, m_sectionInfo[3]/1000);	//the section thickness
				data2.CalcNonstdGeometry();
				m_structure.Labels.Store(sec2);
				
				nonst_data3.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, m_sectionInfo[4]/1000);	//the section diameter
				nonst_data3.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, m_sectionInfo[5]/1000);	//the section thickness
				data3.CalcNonstdGeometry();
				m_structure.Labels.Store(sec3);
				
				//set the values of the plate
				nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, m_sectionInfo[6]/1000);
				nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*m_sectionInfo[7]/1000);	//generate correct section properties without overlapping thicknesses
				nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, m_sectionInfo[7]/1000);
				data4.CalcNonstdGeometry();
				m_structure.Labels.Store(sec4);
				
				nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, m_sectionInfo[8]/1000);
				nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*m_sectionInfo[9]/1000);	//generate correct section properties without overlapping thicknesses
				nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, m_sectionInfo[9]/1000);
				data5.CalcNonstdGeometry();
				m_structure.Labels.Store(sec5);
				
				nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, m_sectionInfo[10]/1000);
				nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*m_sectionInfo[11]/1000);	//generate correct section properties without overlapping thicknesses
				nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, m_sectionInfo[11]/1000);
				data6.CalcNonstdGeometry();
				m_structure.Labels.Store(sec6);
				
				//the values of the cable are already set!
				m_structure.Labels.Store(sec7);
			}
				
			int barId = 5000;
			
			for (int i=0; i<_barInfo.Count; i++)
			{
				Console.WriteLine("Now creating bar " + barId.ToString() + "...");
				Bar b = (Bar)_barInfo[i];
				
				//find the corresponding points in the nodeList to create the bar
				int robStart = 0;
				int robEnd = 0;
				
				foreach (Node n in _nodeInfo)
				{
					if (n.catiaID.Equals(b.CatiaStart))
					{
						robStart = n.robotID;
					}
					else if (n.catiaID.Equals(b.CatiaEnd))
					{
						robEnd = n.robotID;
					}
					else
					{
						continue;
					}
				}

				//create the bar
				m_bars.Create(barId, robStart, robEnd);
				//DON'T CREATE A RELEASE - DIFFICULT TO IMPLEMENT
//				m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_RELEASE, "CatiaBarRelease");
				
				if(b.SectionType == SectionType.tube00)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_00");
				}
				else if(b.SectionType == SectionType.tube01)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_01");
				}
				else if(b.SectionType == SectionType.tube02)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_02");
				}
				else if(b.SectionType == SectionType.plate00)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_00");
				}
				else if(b.SectionType == SectionType.plate01)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_01");
				}
				else if(b.SectionType == SectionType.plate02)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_02");
				}
				else if(b.SectionType == SectionType.cable)
				{
					m_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_CABLE, "CatiaCable");
				}
				
				forDeadLoadsBars.Add(barId);
				
				b.RobotID = barId;
				b.RobotStart = robStart;
				b.RobotEnd =   robEnd;
				b.StressAxial = 0.0;
				
				barId++;
				
			}
			#endregion
			
			#region plates
			
			#region material info
			
			//create the slab material
//			string materialName = "Catia_material_slab";
//			IRobotLabel Label = m_structure.Labels.Create(IRobotLabelType.I_LT_MATERIAL, materialName);
//			RobotMaterialData Material = Label.Data as RobotMaterialData;
//			Material.Type = IRobotMaterialType.I_MT_CONCRETE;
//			Material.E = 30000000000;
//			Material.NU = 0.16;
//			Material.RO = 25000;
//			Material.Kirchoff = Material.E / (2*(1 + Material.NU));
//			m_project.Structure.Labels.Store(Label);
			
			//create the panel type
			string catSlabSectionName = "Catia_Slab";
			IRobotLabel Label = m_structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catSlabSectionName);
			RobotThicknessData thickness = Label.Data as RobotThicknessData;
//			thickness.MaterialName = materialName;
			thickness.MaterialName = "CONCR";
			thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
			RobotThicknessHomoData thicknessData = thickness.Data as RobotThicknessHomoData;
			if (_plateInfo.Count != 0)
			{
				thicknessData.ThickConst = _plateInfo[0].Thickness/1000;	//test one panel for thickness
			}
			else
			{
				thicknessData.ThickConst = .02;
			}

			m_project.Structure.Labels.Store(Label);

			//create the panel material
//			string materialName = "Catia_material_panel";
//			Label = m_structure.Labels.Create(IRobotLabelType.I_LT_MATERIAL, materialName);
//			Material = Label.Data as RobotMaterialData;
//			Material.Type = IRobotMaterialType.I_MT_CONCRETE;
//			Material.Type = IRobotMaterialType.I_MT_ALUMINIUM;
//			Material.E = 69000000000;
//			Material.NU = 0.16;		//the poisson ratio
//			Material.RO = 25000;	//the mass
//			Material.Kirchoff = Material.E / (2*(1 + Material.NU));
//			m_project.Structure.Labels.Store(Label);
			
			string catPanelSectionName = "Catia_Panel";
			Label = m_structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catPanelSectionName);
			thickness = Label.Data as RobotThicknessData;
//			thickness.MaterialName = materialName;
			thickness.MaterialName = "ALUM";
			thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
			thicknessData = thickness.Data as RobotThicknessHomoData;
			thicknessData.ThickConst = 0.003175;	//a 1/8" thick aluminum panel, we'll need to use a single finite element
			m_project.Structure.Labels.Store(Label);
			
			string catShearPanelName = "Catia_Shear";
			Label = m_structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catShearPanelName);
			thickness = Label.Data as RobotThicknessData;
			thickness.MaterialName = "CONCR";
			thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
			thicknessData = thickness.Data as RobotThicknessHomoData;
			thicknessData.ThickConst = 0.3;	//a 1/8" thick aluminum panel, we'll need to use a single finite element
			m_project.Structure.Labels.Store(Label);
			#endregion
			
			int plateId = 10000;
			
			for (int i=0; i<_plateInfo.Count; i++)
			{
				Console.WriteLine("Now creating plate " + plateId.ToString() + "...");
				List<int> robotNodes= new List<int>();
				
				Plate p = (Plate)_plateInfo[i];	//the plate to create
				
				//at this point we know the CATIA nodes and the robot nodes
				//but the robot node list for each plate is empty
				//find the robot node that corresponds to each catia node in the list
				//fill up the corresponding robot id list 
				
				foreach (string catIndex in p.CatiaPoints)
				{
					foreach (Node n in _nodeInfo)
					{
						//if the node id equals that in the catia points list
						if (n.catiaID.Equals(catIndex))
						{
							robotNodes.Add(n.robotID);	//add the int robot id to the other list
						}
						else
						{
							continue;	//continue;
						}
					}
				}
				
				p.RobotNodes = robotNodes;	//change out the robotNodes list on the object
				
				//the robot point array to hold all the pts
				RobotPointsArray pts = new RobotPointsArrayClass();
				pts.SetSize(p.RobotNodes.Count);

				int ptIndex = 1;
				
				//fill up the points array
				foreach (int rId in p.RobotNodes)
				{	
					IRobotNode robNode = m_structure.Nodes.Get(rId) as IRobotNode;
					pts.Set(ptIndex, robNode.X, robNode.Y, robNode.Z);
					//MessageBox.Show(robNode.X.ToString());
					ptIndex++;
				
				}
				
				//int plateId = m_structure.Objects.FreeNumber;

				m_project.Structure.Objects.CreateContour(plateId, pts);
				IRobotObjObject obj = m_structure.Objects.Get(plateId) as IRobotObjObject;
				obj.Main.Attribs.Meshed = 1;
				
				if (p.PlateType == PlateType.slab)
				{
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catSlabSectionName);
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_REINFORCEMENT, "Panel");
				}
				else if (p.PlateType == PlateType.panel)
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catPanelSectionName);
				else if (p.PlateType == PlateType.shear)
				{
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catShearPanelName);
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_REINFORCEMENT, "Wall");
				}
				else
					obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catPanelSectionName);
				
			
				obj.Initialize();
				obj.Update();

				
				//create some loads
				//if the live load has a value
//				if(p.PlateType == PlateType.slab)
				if(p.LiveLoad > 0.0)
				{
					Console.WriteLine("Now creating live load on slab " + plateId.ToString() + "...");
				
					liveLoad = p.LiveLoad;
					forLiveLoads.Add(plateId);
					
				}
				//if the wind load has a value
//				else if (p.PlateType == PlateType.panel)
				if(p.WindLoad > 0.0)
				{
					Console.WriteLine("Now creating wind load on panel " + plateId.ToString() + "...");

					windLoad = p.WindLoad;
					forWindLoads.Add(plateId);

				}
				
				if(p.BuildingLoad > 0.0)
				{
					Console.WriteLine("Now creating building load on panel " + plateId.ToString() + "...");	
					buildingLoad = p.BuildingLoad;
					forBuildingLoads.Add(plateId);
				}
				forDeadLoadsSlabs.Add(plateId);	//all 

				p.RobotId = plateId; //set the plateId on the object
				plateId++;

			}
			#endregion
			
			#region add loads 
		
			IRobotLoadRecord rec = null;

			if (forLiveLoads.Count > 0)
			{	
				//add the live loads on all panels
				rec = ll.Records.Get(1);
				rec.Objects.FromText(forLiveLoads[0].ToString() +"to"+forLiveLoads[forLiveLoads.Count-1].ToString());
				rec.SetValue(2, -liveLoad);
				rec.SetValue(11, 0);
			}
			
			if (forWindLoads.Count > 0)
			{
				//add the forces on the wind panels
				rec = wl.Records.Get(1);
				rec.Objects.FromText(forWindLoads[0].ToString() +"to"+forWindLoads[forWindLoads.Count-1].ToString());
				
				rec.SetValue(0, m_windx*windLoad);		//the X value
				rec.SetValue(1, m_windy*windLoad);		//the Y value
				rec.SetValue(2, m_windz*windLoad);		// the Z value
//				rec.SetValue(11, 1);	//this sets it use the "local" normal for the wind direction
				rec.SetValue(11, 0);	//this is the default - loads act in the global direction
				
				//add the uplift on the wind panels
//				rec = wl_uplift.Records.Get(1);
//				rec.Objects.FromText(forWindLoads[0].ToString() +"to"+forWindLoads[forWindLoads.Count-1].ToString());
//				rec.SetValue(2, -windLoad);
//				rec.SetValue(11,1);
				
			}
			
			if(forBuildingLoads.Count > 0)
			{
				rec = bl.Records.Get(1);
				rec.Objects.FromText(forBuildingLoads[0].ToString() +"to"+forBuildingLoads[forBuildingLoads.Count-1].ToString());
				rec.SetValue(2, -buildingLoad);
				rec.SetValue(11, 0);
			}
			
			//create a randomly placed live load on the structure
			//the fat man load
			IRobotCollection nodes = m_nodes.GetAll();
			Random r = new Random();
			int randPoint = r.Next((nodes.Get(1) as IRobotNode).Number , (nodes.Get(nodes.Count) as IRobotNode).Number);
//			Debug.WriteLine(randPoint);
			rec = ll.Records.Get(2);
			rec.SetValue(2, -fatManLoad);
			rec.Objects.FromText(randPoint.ToString());
			
			//set the dead loads for the structure
			rec = dl.Records.Get(1);
			
			//add dead loads to bars
			if(forDeadLoadsBars.Count > 0 && forDeadLoadsSlabs.Count == 0)
			{
				rec.Objects.FromText(forDeadLoadsBars[0].ToString() + "to" + forDeadLoadsBars[forDeadLoadsBars.Count-1].ToString());
			}
			//add dead loads to bars and slabs
			else if (forDeadLoadsBars.Count > 0 && forDeadLoadsSlabs.Count > 0)
			{
				rec.Objects.FromText(forDeadLoadsBars[0].ToString() + "to" + forDeadLoadsBars[forDeadLoadsBars.Count-1].ToString() + " " +
					forDeadLoadsSlabs[0].ToString() +"to"+forDeadLoadsSlabs[forDeadLoadsSlabs.Count-1].ToString());
			}
			//add dead loads to slabs only
			else if (forDeadLoadsBars.Count == 0 && forDeadLoadsSlabs.Count > 0)
			{
				rec.Objects.FromText(forDeadLoadsSlabs[0].ToString() +"to"+forDeadLoadsSlabs[forDeadLoadsSlabs.Count-1].ToString());
			}
			rec.SetValue(2, -1);	//set the z value

//			Console.WriteLine("Waiting for load application...");
//			System.Threading.Thread.Sleep(10000);
			
			#endregion
			
			return true;
		}

		public static bool AnalyzeAndWriteResults(string currDirectory, string fileName)
		{
			Console.WriteLine("Calculating new single element meshes...");
			
			//select all the finite element stuff
			RobotSelection allFE = m_structure.Selections.CreateFull(IRobotObjectType.I_OT_FINITE_ELEMENT);
			
			//get the finite element server
			IRobotFiniteElementServer fe_server = m_structure.FiniteElems;
			
			//delete the finite element meshes
			fe_server.DeleteMany(allFE);
			
			//consolidate finite elements with a coefficient of one for panels to convex boundaries
			//so two triangle become a square...
			//fe_server.MeshConsolidate(1.0, allFE, false);

			IRobotMeshParams meshParams = m_project.Preferences.MeshParams;
			meshParams.MeshType = IRobotMeshType.I_MT_NORMAL;
			meshParams.SurfaceParams.Generation.Division1 = 0;
			meshParams.SurfaceParams.Generation.Division2 = 0;
	
			
			fe_server.Update();
			fe_server.MeshConcentrate(IRobotMeshRefinementType.I_MRT_SIMPLE, allFE, false);
				
			Console.WriteLine("Now analyzing...");
			
			m_project.CalcEngine.AnalysisParams.IgnoreWarnings = true;
			m_project.CalcEngine.AutoGenerateModel = true;
//			m_project.CalcEngine.GenerateModel();
		
			int calcResult = m_project.CalcEngine.Calculate();

			if (calcResult != 0)
			{
				Console.WriteLine("Model analyzed successfully...");
				//System.Threading.Thread.Sleep(2000);
			}
			else if (calcResult == 0)
			{
				Console.WriteLine("Model could not be analyzed...");
				//System.Threading.Thread.Sleep(2000);
			}
			
			Console.WriteLine("Saving model with analysis results...");
			m_project.SaveAs(m_robotPath);
			Console.WriteLine("Robot project saved to " + m_robotPath + "...");
			
			IRobotPrintEngine printEngine = m_project.PrintEngine;
			printEngine.AddTemplateToReport("Results");
			printEngine.SaveReportToFile(currDirectory + "\\" + fileName + ".txt", IRobotOutputFileFormat.I_OFF_TEXT);
			
			InitializeResultsServers();
			CheckDeflections();
			CheckStresses();
			
			WriteCheckDataToOutputFile(currDirectory, fileName);

			return true;	
		}
		
		public static bool InitializeResultsServers()
		{
			Console.WriteLine("Initializing the results servers...");

			m_results = m_structure.Results;
			m_displacements = m_structure.Results.Nodes.Displacements;
			m_cases = m_structure.Cases;
			m_stresses = m_structure.Results.Bars.Stresses;

			return true;
			
		}
		
		/// <summary>
		/// Check nodal deflections for all cases returning number of failed nodes 
		/// </summary>
		/// <returns></returns>
		public static bool CheckDeflections()
		{
			Console.WriteLine("Checking nodal deflections...");
			
			//for each load case
			IRobotCaseCollection cases = m_cases.GetAll();
			IRobotCollection nodes = m_nodes.GetAll();

			for (int i=1; i<=cases.Count; i++)
			{
				for(int j=1; j<=nodes.Count; j++)
				{
					IRobotNode n = nodes.Get(j) as IRobotNode;
					
					IRobotDisplacementData displace = m_displacements.Value(n.Number, i);
					
//					Debug.WriteLine(n.Number.ToString() + " : Node displacement = " + displace.UZ.ToString());
					
					//calculate the vector of total displacement
					double dispDist = Math.Sqrt(Math.Pow(Math.Abs(displace.UX),2) + 
					                             Math.Pow(Math.Abs(displace.UY),2) +
					                             Math.Pow(Math.Abs(displace.UZ),2));
					if (dispDist > m_displacementTolerance/1000)
					{
						//don't put the node in the list if it has alread failed
						//in another case
						if(!m_failedNodes.Contains(n.Number))
						{
							m_failedNodes.Add(n.Number);
//							Debug.WriteLine("Nodal displacement failure " + m_failedNodes.Count.ToString() + ": " + displace.UZ.ToString() + ">" + m_displacementTolerance.ToString());
						}
					}
				}
				
			}

			return true;
		}
		
		/// <summary>
		/// Check mmeber stresses for all cases returning number of failed members
		/// </summary>
		/// <returns></returns>
		public static bool CheckStresses()
		{
			Console.WriteLine("Checking member bending stresses...");
			
			//for each load case
			IRobotCaseCollection cases = m_cases.GetAll();
			IRobotCollection bars = m_bars.GetAll();

			for (int i=1; i<=cases.Count; i++)
			{
				for(int j=1; j<=bars.Count; j++)
				{
					IRobotBar b = bars.Get(j) as IRobotBar;
					
					IRobotBarStressData stress1 = m_stresses.Value(b.Number, i,0);	//check stresses at both ends
					IRobotBarStressData stress2 = m_stresses.Value(b.Number, i,1);
					
//					Debug.WriteLine(b.Number.ToString() + " : Member stress = " + stress1.SmaxMY.ToString() + "/" + stress2.SmaxMY.ToString());
					
					if (Math.Abs(stress1.SmaxMY) > m_stressTolerance || Math.Abs(stress2.SmaxMY) > m_stressTolerance)
					{
						//don't put the bar in the list if it has already failed
						//in another case
						if(!m_failedBars.Contains(b.Number))
						{
							m_failedBars.Add(b.Number);
//							Debug.WriteLine("Bar stress failure " + m_failedBars.Count.ToString() + ": " + stress1.SmaxMY.ToString() + ">" + m_stressTolerance.ToString());
						}
					}
				}
				
			}
			return true;
			
		}
		
		public static bool WriteCheckDataToOutputFile(string _currDirectory, string _fileName)
		{
			string filePath = _currDirectory + "\\" + _fileName + ".txt";
			
			Console.WriteLine("Writing check information to output file...");			
			using (StreamWriter sw = new StreamWriter(filePath,true))
       		{
				sw.WriteLine("Nodal displacement failures : " + m_failedNodes.Count.ToString());
				sw.WriteLine("Member bending stress failures : " + m_failedBars.Count.ToString());
			}
			
			return true;
		}

	}

    [NodeName("DynamoBotSectionType")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Choose a section type to be used with analytical members.")]
    public class SectionTypeNode : dynEnum
    {
        public SectionTypeNode()
        {
            InPortData.Add(new PortData("n", "A description.", typeof(object)));

            WireToEnum(Enum.GetValues(typeof(SectionType)));
        }
    }

    [NodeName("DynamoBotPlateType")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Choose a plate type to be used with analytical members.")]
    public class PlateTypeNode : dynEnum
    {
        public PlateTypeNode()
        {
            InPortData.Add(new PortData("n", "A description.", typeof(object)));

            WireToEnum(Enum.GetValues(typeof(PlateType)));
        }
    }

    [NodeName("DynamoBot")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical link with Robot.")]
    public class DynamoRobot : dynNodeWithOneOutput
    {
        public DynamoRobot()
        {
            InPortData.Add(new PortData("nodes", "The point(s) from which to define analytical nodes.", typeof(Point3D)));
            InPortData.Add(new PortData("bars", "The curves from which to define analytical bars.", typeof(Curve3D)));
            OutPortData.Add(new PortData("results", "The analytical results.", typeof(object)));

            NodeUI.RegisterAllPorts();
        }

        public override FScheme.Value Evaluate(FSharpList<FScheme.Value> args)
        {
            throw new NotImplementedException();
        }
    }

	//!The Node Class
    [NodeName("DynamoBotNode")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical node in Robot.")]
	public class Node : dynNodeWithOneOutput
	{
		#region private members
		private double m_x;
		private double m_y;
		private double m_z;
		
        //private double m_dispX;
        //private double m_dispY;
        //private double m_dispZ;
		
        //private string m_catID;
        //private int m_robotID;
		private bool m_fixed = false;
		
		#endregion private members
		
		#region properties
		//*!The X coordinate of the node.
        //public double X
        //{
        //    get
        //    {
        //        return m_x;
        //    }
        //    set
        //    {
        //        m_x = (double)value;
        //    }
        //}
		
        ////!The Y coordinate of the node.
        //public double Y
        //{
        //    get
        //    {
        //        return m_y;
        //    }
        //    set
        //    {
        //        m_y = (double)value;
        //    }
        //}
		
        ////!The Z coordinate of the node.
        //public double Z
        //{
        //    get
        //    {
        //        return m_z;
        //    }
        //    set
        //    {
        //        m_z = (double)value;
        //    }
        //}
		
        ////!The corresponding catiaID
        //public string catiaID
        //{
        //    get
        //    {
        //        return m_catID;
        //    }
        //    set
        //    {
        //        m_catID = (string)value;
        //    }
        //}
		
        ////!The corresponding robotID
        //public int robotID
        //{
        //    get
        //    {
        //        return m_robotID;
        //    }
        //    set
        //    {
        //        m_robotID = (int)value;
        //    }
        //}
		
        ////!The analyzed displacement in the X direction.
        //public double displaceX
        //{
        //    get
        //    {
        //        return m_dispX;
        //    }
        //    set
        //    {
        //        m_dispX = (double)value;
        //    }
        //}
		
        ////!The analyzed displacement in the Y direction.
        //public double displaceY
        //{
        //    get
        //    {
        //        return m_dispY;
        //    }
        //    set
        //    {
        //        m_dispY = (double)value;
        //    }
        //}
		
        ////!The analyzed displacement in the Z direction.
        //public double displaceZ
        //{
        //    get
        //    {
        //        return m_dispZ;
        //    }
        //    set
        //    {
        //        m_dispZ = (double)value;
        //    }
        //}
		
        ////!The fixity of the node
        //public int Fixed
        //{
        //    get
        //    {
        //        return m_fixed;
        //    }
        //}
		#endregion properties
		
		//!The node constructor
		public Node()
		{
            //m_x = x;
            //m_y = y;
            //m_z = z;
            //m_dispX = dispx;
            //m_dispY = dispy;
            //m_dispZ = dispz;
            //m_fixed = fixity;
			
            //m_catID = catNode;
            //m_robotID = robNode;

            InPortData.Add(new PortData("x", "The X location.", typeof(double)));
            InPortData.Add(new PortData("y", "The Y location.", typeof(double)));
            InPortData.Add(new PortData("z", "The Z location.", typeof(double)));
            InPortData.Add(new PortData("fix", "Should the node be fixed?", typeof(bool)));

            OutPortData.Add(new PortData("node", "The analytical node", typeof(bool)));
		}

        public override FScheme.Value Evaluate(FSharpList<FScheme.Value> args)
        {
            m_x = (double)((Value.Number)args[0]).Item;
            m_y = (double)((Value.Number)args[1]).Item;
            m_z = (double)((Value.Number)args[2]).Item;
            m_fixed = Convert.ToBoolean((double)((Value.Number)args[0]).Item);

            throw new NotImplementedException("Construct a node.");
        }
	}
	
	//!The Bar Class
    [NodeName("DynamoBotBar")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical bar in Robot.")]
	public class Bar : dynNodeWithOneOutput
	{
		#region private members
		
        //private string m_catStart;
        //private string m_catEnd;
        //private int m_robStart;
        //private int m_robEnd;
        //private int m_robId;
        //private string m_catId;
        //private double m_stressAxial;
		private double m_diameter;
        private SectionType m_sectionType;
        private double m_sectionThickness;
        private string m_endRelease;
		
		#endregion private members
		
		#region properties
		//!The corresponding Catia start node.
        //public string CatiaStart
        //{
        //    get
        //    {
        //        return m_catStart;
				
        //    }
        //    set
        //    {
        //        m_catStart = (string)value;
        //    }
        //}
		
        ////!The corresponding Catia end node.
        //public string CatiaEnd
        //{
        //    get
        //    {
        //        return m_catEnd;
        //    }
        //    set
        //    {
        //        m_catEnd = (string)value;
        //    }
        //}
		
        ////!The corrseponding Robot start node.
        //public int RobotStart
        //{
        //    get
        //    {
        //        return m_robStart;
        //    }
        //    set
        //    {
        //        m_robStart = (int)value;
        //    }
        //}
		
        ////!The corresponding Robot end node.
        //public int RobotEnd
        //{
        //    get
        //    {
        //        return m_robEnd;
        //    }
        //    set
        //    {
        //        m_robEnd = (int)value;
        //    }
        //}
		
        ////!The analyzed axial stress in the bar.
        //public double StressAxial
        //{
        //    get
        //    {
        //        return m_stressAxial;
        //    }
        //    set
        //    {
        //        m_stressAxial = (double)value;
        //    }
        //}
		
        ////!The corresponding Robot Id of the bar.
        //public int RobotID
        //{
        //    get
        //    {
        //        return m_robId;	
        //    }
        //    set
        //    {
        //        m_robId = (int)value;
        //    }
        //}
		
        ////!The corresponding CatiaID of the bar.
        //public string CatiaID
        //{
        //    get
        //    {
        //        return m_catId;
        //    }
        //    set
        //    {
        //        m_catId = (string)value;
        //    }
        //}
		
		//!The diameter of the bar
        //public double Diameter
        //{
        //    get
        //    {
        //        return m_diameter;
        //    }
        //}
		
        ////!The section thickness.
        //public double SectionThickness
        //{
        //    get
        //    {
        //        return m_sectionThickness;
        //    }
        //}
		
        ////!The section type - bar or plate.
        //public SectionType SectionType
        //{
        //    get
        //    {
        //        return m_sectionType;
        //    }
        //}
		
        //public string EndRelease
        //{
        //    get
        //    {
        //        return m_endRelease;
        //    }
        //}
		#endregion
		
		//!The Bar constructor.
		public Bar()
		{
            //m_catId 	= _catId;
            //m_catStart 	= _catStart;
            //m_catEnd	= _catEnd;
            //m_robId		= _robId;
            //m_robStart  = _robStart;
            //m_robEnd	= _robEnd;
            //m_stressAxial = _axial;
            //m_diameter	= _diameter;
            //m_sectionType = _sectionType;
            //m_sectionThickness = _thickness;
            //m_endRelease = _endRelease;

            InPortData.Add(new PortData("n", "The diameter of the bar.", typeof(double)));
            InPortData.Add(new PortData("n", "The section type of the bar.", typeof(PlateType)));
            InPortData.Add(new PortData("n", "The section thickness of the bar.", typeof(double)));
            InPortData.Add(new PortData("n", "The the end release of the bar.", typeof(string)));
            OutPortData.Add(new PortData("bar", "The analytical bar.", typeof(Bar)));

            NodeUI.RegisterAllPorts();
		}

        public override Value Evaluate(FSharpList<Value> args)
        {
            m_diameter = ((Value.Number)args[0]).Item;
            m_sectionType = (SectionType)((Value.Container)args[0]).Item;
            m_sectionThickness = ((Value.Number)args[0]).Item;
            m_endRelease = ((Value.String)args[0]).Item; 

            throw new NotImplementedException("Construct a bar");
        }

	}
	
	//!The Plate Class
    [NodeName("DynamoBotPlate")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical plate in Robot.")]
	public class Plate
	{
		#region private members
		private string 			m_catiaId;
		private int 			m_robId;
		private List<string>	m_catNodes;
		private List<int>		m_robNodes;
		private double			m_liveLoad;
		private double			m_windLoad;
		private PlateType		m_plateType;
		private double			m_thick;
		private double			m_buildingLoad;
		#endregion
		
		#region properties
		//!The corresponding Catia Id
		public string CatiaId
		{
			get
			{
				return m_catiaId;
			}
			set
			{
				m_catiaId = value;
			}
		}
		
		//!The corresponding Robot Id
		public int RobotId
		{
			get
			{
				return m_robId;
			}
			set
			{
				m_robId = value;
			}
		}
		
		//!The List of Catia Points which make up the plate element.
		public List<string> CatiaPoints
		{
			get
			{
				return m_catNodes;
			}
			set
			{
				m_catNodes = value;
			}
		}
		
		//!The list of Robot Nodes which make up the plate element.
		public List<int> RobotNodes
		{
			get
			{
				return m_robNodes;
			}
			set
			{
				m_robNodes = value;
			}
		}
		#endregion
		
		//!The loaded or unloaded state of the plate
		public double WindLoad
		{
			get
			{
				return m_windLoad;
			}
		}
		
		public double LiveLoad
		{
			get
			{
				return m_liveLoad;
			}
		}
		
		public double BuildingLoad
		{
			get{
				return m_buildingLoad;
			}
		}
		public PlateType PlateType
		{
			get
			{
				return m_plateType;
			}
			set
			{
				m_plateType = value;
			}
		}
		
		public double Thickness
		{
			get
			{
				return m_thick;
			}
		}
		
		//!The Plate constructor.
		public Plate (string _catId, int _robId, List<string> _catPts, List<int> _robNodes, double[] _loads, double thick)
		{
			m_catiaId = _catId;
			m_robId = _robId;
			m_catNodes = _catPts;
			m_robNodes = _robNodes;
			m_windLoad = _loads[0];
			m_liveLoad = _loads[1];
			m_buildingLoad = _loads[2];
			m_thick	= thick;
		}
		
	}
	
	
}
