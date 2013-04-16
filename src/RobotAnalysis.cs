using System;
using System.Linq;
using System.Collections;
using System.Collections.Generic;
using RobotOM;

namespace Dynamo.Analysis
{
    public class RobotEngine
    {
        public IRobotApplication Application { get; private set; }

        public RobotEngine()
        {
            InitializeRobot();

            if(Application == null)
                throw new Exception("Robot could not be initialized. Please make sure the Robot application is available.");
        }

        /// <summary>
        /// Initialize Robot
        /// </summary>
        /// <returns>True or false</returns>
        public void InitializeRobot()
        {
            Console.WriteLine("Initializing Robot...");
            Application = new RobotApplication();

            System.Threading.Thread.Sleep(20000);
            Console.WriteLine("Waiting for Robot...");
        }
    }

    public class RobotModel
    {
        private IRobotProject _project;   
		private IRobotNodeServer _nodes;
		private IRobotBarServer _bars;
		private IRobotObjObjectServer _objects;
		
		private IRobotStructure _structure;
		private IRobotResultServer _results;
		private IRobotNodeDisplacementServer _displacementServer;
		private IRobotCaseServer _caseServer;
		private IRobotBarStressServer _stressServer;

		private List<AnalyticalBar> _barInfo = new List<AnalyticalBar>();
		private List<AnalyticalNode> _nodeInfo = new List<AnalyticalNode>();
		private List<AnalyticalPlate> _plateInfo = new List<AnalyticalPlate>();
		private double[] _sectionInfo = new double[13];

        private double _windx = 0.0;
        private double _windy = 0.0;
        private double _windz = 0.0;	//the wind vector

        /// <summary>
		/// Create the Robot model from scratch.
		/// </summary>
		/// <param name="_nodeInfo">A List of node objects.</param>
		/// <param name="_barInfo">A List of bar objects.</param>
        public RobotModel(IRobotApplication robot, IEnumerable<AnalyticalNode> nodeInfo, IEnumerable<AnalyticalBar> barInfo, IEnumerable<AnalyticalPlate> plateInfo)
		{
			robot.Project.New(RobotOM.IRobotProjectType.I_PT_SHELL);
            _project = robot.Project;

			if (_project == null)
				throw new Exception("The robot project could not be initialized.");
			
			//create the structure and the node and bar servers
			_structure = _project.Structure;
			_nodes = _structure.Nodes;
			_bars = _structure.Bars;	
			_objects = _structure.Objects;
			
			List<int> forWindLoads = new List<int>();
			List<int> forLiveLoads = new List<int>();
			List<int> forBuildingLoads = new List<int>();
			List<int> forDeadLoadsBars = new List<int>();
			List<int> forDeadLoadsSlabs = new List<int>();
			
			double windLoad	= 0.0;
			double liveLoad = 0.0;
			double buildingLoad = 0.0;
			double fatManLoad = 300 * 4.45;	//300lbs. -> Newton

            IRobotSimpleCase ll;
            IRobotSimpleCase dl;
            IRobotSimpleCase bl;
            IRobotSimpleCase wl;
            CreateLoadCases(out ll, out dl, out wl, out bl);

            CreateLoads(ll,dl, wl, bl);

            CreateBarEndRelease(_barInfo);

            CreateNodes();

			CreateBars(forDeadLoadsBars);

			#region plates
			
			CreatePlates(liveLoad, forLiveLoads, windLoad, forWindLoads, buildingLoad, forBuildingLoads, forDeadLoadsSlabs, ll, wl, bl, fatManLoad, dl, forDeadLoadsBars);

            #endregion

		}

        private static void CreateLoads(IRobotSimpleCase ll, IRobotSimpleCase wl, IRobotSimpleCase dl, IRobotSimpleCase bl)
        {
            //define planar loads to be applied to the panels and slabs 
            ll.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM); //record 1 - slab loads
            ll.Records.New(IRobotLoadRecordType.I_LRT_NODE_FORCE); //record 2 - Fat man load
            wl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM); //record 3 - plate loads
//			wl_uplift.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//record 1 
            dl.Records.New(IRobotLoadRecordType.I_LRT_DEAD); //record 4 see pg. 150, 160 Robot API docs
            //sl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM);	//snow load for panels and slabs
            bl.Records.New(IRobotLoadRecordType.I_LRT_UNIFORM); //record 5 - building load
        }

        private IRobotSimpleCase CreateLoadCases(out IRobotSimpleCase dl, out IRobotSimpleCase ll, out IRobotSimpleCase wl, out IRobotSimpleCase bl)
        {
            //create load cases
            dl = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_DL",
                                                                IRobotCaseNature.I_CN_PERMANENT,
                                                                IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
            ll = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_LL", IRobotCaseNature.I_CN_EXPLOATATION,
                                               IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
            wl = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_WL", IRobotCaseNature.I_CN_WIND,
                                               IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
            bl = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_BL", IRobotCaseNature.I_CN_EXPLOATATION,
                                               IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);

//			IRobotSimpleCase sl = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_SL", IRobotCaseNature.I_CN_SNOW, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
//			IRobotSimpleCase wl_uplift = _structure.Cases.CreateSimple(_structure.Cases.FreeNumber, "Catia_WL_Uplift", IRobotCaseNature.I_CN_WIND, IRobotCaseAnalizeType.I_CAT_STATIC_LINEAR);
            IRobotCaseCombination comb1 = _structure.Cases.CreateCombination(_structure.Cases.FreeNumber, "Catia_DL+LL+WL+BL",
                                                                             IRobotCombinationType.I_CBT_ACC,
                                                                             IRobotCaseNature.I_CN_PERMANENT,
                                                                             IRobotCaseAnalizeType.I_CAT_COMB);
            IRobotCaseCombination comb2 = _structure.Cases.CreateCombination(_structure.Cases.FreeNumber, "Catia_DL_60+WL",
                                                                             IRobotCombinationType.I_CBT_ACC,
                                                                             IRobotCaseNature.I_CN_PERMANENT,
                                                                             IRobotCaseAnalizeType.I_CAT_COMB);

            RobotCaseFactorMngr caseManage1 = comb1.CaseFactors;
            caseManage1.New(1, 1.0);
            caseManage1.New(2, 1.0);
            caseManage1.New(3, 1.0);
            caseManage1.New(4, 1.0); //add the building load

            RobotCaseFactorMngr caseManage2 = comb2.CaseFactors;
            caseManage2.New(1, 0.6);
            caseManage2.New(2, 1.0);
            caseManage2.New(3, 1.0);
            return dl;
        }

        private void CreateBarEndRelease(List<AnalyticalBar> barInfo)
        {
            if (barInfo.Any(x => x.EndRelease.Length != 12))
                throw new Exception("End Release value must be 12 values formatted as 'x'(free) and 'f'(fixed)");

            //create a bar end release
            IRobotLabel rel1 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_RELEASE, "CatiaBarRelease");
            IRobotBarReleaseData relData1 = rel1.Data as IRobotBarReleaseData;
            IRobotBarEndReleaseData relStartData1 = relData1.StartNode;
            IRobotBarEndReleaseData relEndData1 = relData1.EndNode;

            IRobotBarEndReleaseValue[] relVals = new IRobotBarEndReleaseValue[12];
            char[] letters = _barInfo[0].EndRelease.ToCharArray();

            for (int i = 0; i < _barInfo[0].EndRelease.Length; i++)
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

            relStartData1.UX = relVals[0];
            relStartData1.UY = relVals[1];
            relStartData1.UZ = relVals[2];
            relStartData1.RX = relVals[3];
            relStartData1.RY = relVals[4];
            relStartData1.RZ = relVals[5];
            relEndData1.UX = relVals[6];
            relEndData1.UY = relVals[7];
            relEndData1.UZ = relVals[8];
            relEndData1.RX = relVals[9];
            relEndData1.RY = relVals[10];
            relEndData1.RZ = relVals[11];

            _structure.Labels.Store(rel1);
        }

        private void CreatePlates(double liveLoad, List<int> forLiveLoads, double windLoad, List<int> forWindLoads, double buildingLoad,
                                  List<int> forBuildingLoads, List<int> forDeadLoadsSlabs, IRobotSimpleCase ll, IRobotSimpleCase wl,
                                  IRobotSimpleCase bl, double fatManLoad, IRobotSimpleCase dl, List<int> forDeadLoadsBars)
        {
            #region material info

            //create the slab material
//			string materialName = "Catia_material_slab";
//			IRobotLabel Label = _structure.Labels.Create(IRobotLabelType.I_LT_MATERIAL, materialName);
//			RobotMaterialData Material = Label.Data as RobotMaterialData;
//			Material.Type = IRobotMaterialType.I_MT_CONCRETE;
//			Material.E = 30000000000;
//			Material.NU = 0.16;
//			Material.RO = 25000;
//			Material.Kirchoff = Material.E / (2*(1 + Material.NU));
//			_project.Structure.Labels.Store(Label);

            //create the panel type
            string catSlabSectionName = "Catia_Slab";
            IRobotLabel Label = _structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catSlabSectionName);
            RobotThicknessData thickness = Label.Data as RobotThicknessData;
//			thickness.MaterialName = materialName;
            thickness.MaterialName = "CONCR";
            thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
            RobotThicknessHomoData thicknessData = thickness.Data as RobotThicknessHomoData;
            if (_plateInfo.Count != 0)
            {
                thicknessData.ThickConst = _plateInfo[0].Thickness/1000; //test one panel for thickness
            }
            else
            {
                thicknessData.ThickConst = .02;
            }

            _project.Structure.Labels.Store(Label);

            //create the panel material
//			string materialName = "Catia_material_panel";
//			Label = _structure.Labels.Create(IRobotLabelType.I_LT_MATERIAL, materialName);
//			Material = Label.Data as RobotMaterialData;
//			Material.Type = IRobotMaterialType.I_MT_CONCRETE;
//			Material.Type = IRobotMaterialType.I_MT_ALUMINIUM;
//			Material.E = 69000000000;
//			Material.NU = 0.16;		//the poisson ratio
//			Material.RO = 25000;	//the mass
//			Material.Kirchoff = Material.E / (2*(1 + Material.NU));
//			_project.Structure.Labels.Store(Label);

            string catPanelSectionName = "Catia_Panel";
            Label = _structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catPanelSectionName);
            thickness = Label.Data as RobotThicknessData;
//			thickness.MaterialName = materialName;
            thickness.MaterialName = "ALUM";
            thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
            thicknessData = thickness.Data as RobotThicknessHomoData;
            thicknessData.ThickConst = 0.003175; //a 1/8" thick aluminum panel, we'll need to use a single finite element
            _project.Structure.Labels.Store(Label);

            string catShearPanelName = "Catia_Shear";
            Label = _structure.Labels.Create(IRobotLabelType.I_LT_PANEL_THICKNESS, catShearPanelName);
            thickness = Label.Data as RobotThicknessData;
            thickness.MaterialName = "CONCR";
            thickness.ThicknessType = IRobotThicknessType.I_TT_HOMOGENEOUS;
            thicknessData = thickness.Data as RobotThicknessHomoData;
            thicknessData.ThickConst = 0.3; //a 1/8" thick aluminum panel, we'll need to use a single finite element
            _project.Structure.Labels.Store(Label);

            #endregion

            int plateId = 10000;

            for (int i = 0; i < _plateInfo.Count; i++)
            {
                Console.WriteLine("Now creating plate " + plateId.ToString() + "...");
                List<int> robotNodes = new List<int>();

                var p = (AnalyticalPlate) _plateInfo[i]; //the plate to create

                //at this point we know the CATIA nodes and the robot nodes
                //but the robot node list for each plate is empty
                //find the robot node that corresponds to each catia node in the list
                //fill up the corresponding robot id list 

                //foreach (string catIndex in p.CatiaPoints)
                //{
                //    foreach (Node n in _nodeInfo)
                //    {
                //        //if the node id equals that in the catia points list
                //        if (n.catiaID.Equals(catIndex))
                //        {
                //            robotNodes.Add(n.robotID);	//add the int robot id to the other list
                //        }
                //        else
                //        {
                //            continue;	//continue;
                //        }
                //    }
                //}

                p.Nodes = robotNodes; //change out the robotNodes list on the object

                //the robot point array to hold all the pts
                RobotPointsArray pts = new RobotPointsArrayClass();
                pts.SetSize(p.Nodes.Count);

                int ptIndex = 1;

                //fill up the points array
                foreach (int rId in p.Nodes)
                {
                    IRobotNode robNode = _structure.Nodes.Get(rId) as IRobotNode;
                    pts.Set(ptIndex, robNode.X, robNode.Y, robNode.Z);
                    //MessageBox.Show(robNode.X.ToString());
                    ptIndex++;
                }

                //int plateId = _structure.Objects.FreeNumber;

                _project.Structure.Objects.CreateContour(plateId, pts);
                IRobotObjObject obj = _structure.Objects.Get(plateId) as IRobotObjObject;
                obj.Main.Attribs.Meshed = 1;

                if (p.PlateType == PlateType.Slab)
                {
                    obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catSlabSectionName);
                    obj.SetLabel(IRobotLabelType.I_LT_PANEL_REINFORCEMENT, "Panel");
                }
                else if (p.PlateType == PlateType.Panel)
                    obj.SetLabel(IRobotLabelType.I_LT_PANEL_THICKNESS, catPanelSectionName);
                else if (p.PlateType == PlateType.Shear)
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
                if (p.LiveLoad > 0.0)
                {
                    Console.WriteLine("Now creating live load on slab " + plateId.ToString() + "...");

                    liveLoad = p.LiveLoad;
                    forLiveLoads.Add(plateId);
                }
                //if the wind load has a value
//				else if (p.PlateType == PlateType.panel)
                if (p.WindLoad > 0.0)
                {
                    Console.WriteLine("Now creating wind load on panel " + plateId.ToString() + "...");

                    windLoad = p.WindLoad;
                    forWindLoads.Add(plateId);
                }

                if (p.BuildingLoad > 0.0)
                {
                    Console.WriteLine("Now creating building load on panel " + plateId.ToString() + "...");
                    buildingLoad = p.BuildingLoad;
                    forBuildingLoads.Add(plateId);
                }
                forDeadLoadsSlabs.Add(plateId); //all 

                p.Id = plateId; //set the plateId on the object
                plateId++;
            }

            IRobotLoadRecord rec = null;

            if (forLiveLoads.Count > 0)
            {
                //add the live loads on all panels
                rec = ll.Records.Get(1);
                rec.Objects.FromText(forLiveLoads[0].ToString() + "to" + forLiveLoads[forLiveLoads.Count - 1].ToString());
                rec.SetValue(2, -liveLoad);
                rec.SetValue(11, 0);
            }

            if (forWindLoads.Count > 0)
            {
                //add the forces on the wind panels
                rec = wl.Records.Get(1);
                rec.Objects.FromText(forWindLoads[0].ToString() + "to" + forWindLoads[forWindLoads.Count - 1].ToString());

                rec.SetValue(0, _windx*windLoad); //the X value
                rec.SetValue(1, _windy*windLoad); //the Y value
                rec.SetValue(2, _windz*windLoad); // the Z value
//				rec.SetValue(11, 1);	//this sets it use the "local" normal for the wind direction
                rec.SetValue(11, 0); //this is the default - loads act in the global direction

                //add the uplift on the wind panels
//				rec = wl_uplift.Records.Get(1);
//				rec.Objects.FromText(forWindLoads[0].ToString() +"to"+forWindLoads[forWindLoads.Count-1].ToString());
//				rec.SetValue(2, -windLoad);
//				rec.SetValue(11,1);
            }

            if (forBuildingLoads.Count > 0)
            {
                rec = bl.Records.Get(1);
                rec.Objects.FromText(forBuildingLoads[0].ToString() + "to" +
                                     forBuildingLoads[forBuildingLoads.Count - 1].ToString());
                rec.SetValue(2, -buildingLoad);
                rec.SetValue(11, 0);
            }

            //create a randomly placed live load on the structure
            //the fat man load
            IRobotCollection nodes = _nodes.GetAll();
            Random r = new Random();
            int randPoint = r.Next((nodes.Get(1) as IRobotNode).Number, (nodes.Get(nodes.Count) as IRobotNode).Number);
//			Debug.WriteLine(randPoint);
            rec = ll.Records.Get(2);
            rec.SetValue(2, -fatManLoad);
            rec.Objects.FromText(randPoint.ToString());

            //set the dead loads for the structure
            rec = dl.Records.Get(1);

            //add dead loads to bars
            if (forDeadLoadsBars.Count > 0 && forDeadLoadsSlabs.Count == 0)
            {
                rec.Objects.FromText(forDeadLoadsBars[0].ToString() + "to" +
                                     forDeadLoadsBars[forDeadLoadsBars.Count - 1].ToString());
            }
                //add dead loads to bars and slabs
            else if (forDeadLoadsBars.Count > 0 && forDeadLoadsSlabs.Count > 0)
            {
                rec.Objects.FromText(forDeadLoadsBars[0].ToString() + "to" +
                                     forDeadLoadsBars[forDeadLoadsBars.Count - 1].ToString() + " " +
                                     forDeadLoadsSlabs[0].ToString() + "to" +
                                     forDeadLoadsSlabs[forDeadLoadsSlabs.Count - 1].ToString());
            }
                //add dead loads to slabs only
            else if (forDeadLoadsBars.Count == 0 && forDeadLoadsSlabs.Count > 0)
            {
                rec.Objects.FromText(forDeadLoadsSlabs[0].ToString() + "to" +
                                     forDeadLoadsSlabs[forDeadLoadsSlabs.Count - 1].ToString());
            }
            rec.SetValue(2, -1); //set the z value

//			Console.WriteLine("Waiting for load application...");
//			System.Threading.Thread.Sleep(10000);

        }

        private void CreateBars(List<int> forDeadLoadsBars)
        {
//create the tube section
            //type 1
            IRobotLabel sec1 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_00");
            IRobotBarSectionData data1 = sec1.Data as IRobotBarSectionData;
            data1.Type = IRobotBarSectionType.I_BST_NS_TUBE;
            data1.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_TUBE;
            IRobotBarSectionNonstdData nonst_data1 = data1.CreateNonstd(0);

            //type 2
            IRobotLabel sec2 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_01");
            IRobotBarSectionData data2 = sec2.Data as IRobotBarSectionData;
            data2.Type = IRobotBarSectionType.I_BST_NS_TUBE;
            data2.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_TUBE;
            IRobotBarSectionNonstdData nonst_data2 = data2.CreateNonstd(0);

            //type 3
            IRobotLabel sec3 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_02");
            IRobotBarSectionData data3 = sec3.Data as IRobotBarSectionData;
            data3.Type = IRobotBarSectionType.I_BST_NS_TUBE;
            data3.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_TUBE;
            IRobotBarSectionNonstdData nonst_data3 = data3.CreateNonstd(0);

            //create the plate section
            IRobotLabel sec4 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_00");
            IRobotBarSectionData data4 = sec4.Data as IRobotBarSectionData;
            data4.Type = IRobotBarSectionType.I_BST_NS_RECT;
            data4.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_RECT;
            IRobotBarSectionNonstdData nonst_data4 = data4.CreateNonstd(0);

            IRobotLabel sec5 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_01");
            IRobotBarSectionData data5 = sec5.Data as IRobotBarSectionData;
            data5.Type = IRobotBarSectionType.I_BST_NS_RECT;
            data5.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_RECT;
            IRobotBarSectionNonstdData nonst_data5 = data5.CreateNonstd(0);

            IRobotLabel sec6 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_02");
            IRobotBarSectionData data6 = sec6.Data as IRobotBarSectionData;
            data6.Type = IRobotBarSectionType.I_BST_NS_RECT;
            data6.ShapeType = IRobotBarSectionShapeType.I_BSST_USER_RECT;
            IRobotBarSectionNonstdData nonst_data6 = data6.CreateNonstd(0);

            //create a cable section
            IRobotLabel sec7 = _structure.Labels.Create(IRobotLabelType.I_LT_BAR_CABLE, "CatiaCable");
            IRobotBarCableData data7 = sec7.Data as IRobotBarCableData;
            data7.SectionAX = Math.PI*(Math.Pow((_sectionInfo[4]/1000), 2)); //the area formula using the cable radius parameter
            data7.MaterialName = "STEEL";

            if (_barInfo.Count != 0)
            {
//				double sectionDiameter 	= _barInfo[0].Diameter/1000;			//the diameter in meters
//				double sectionthickness	= _barInfo[0].SectionThickness/1000;	//the thickness in meters

                //set the values of the tubes	
                nonst_data1.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, _sectionInfo[0]/1000);
                    //the section diameter
                nonst_data1.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, _sectionInfo[1]/1000);
                    //the section thickness
                data1.CalcNonstdGeometry();
                _structure.Labels.Store(sec1);

                nonst_data2.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, _sectionInfo[2]/1000);
                    //the section diameter
                nonst_data2.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, _sectionInfo[3]/1000);
                    //the section thickness
                data2.CalcNonstdGeometry();
                _structure.Labels.Store(sec2);

                nonst_data3.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_D, _sectionInfo[4]/1000);
                    //the section diameter
                nonst_data3.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_TUBE_T, _sectionInfo[5]/1000);
                    //the section thickness
                data3.CalcNonstdGeometry();
                _structure.Labels.Store(sec3);

                //set the values of the plate
                nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, _sectionInfo[6]/1000);
                nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*_sectionInfo[7]/1000);
                    //generate correct section properties without overlapping thicknesses
                nonst_data4.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, _sectionInfo[7]/1000);
                data4.CalcNonstdGeometry();
                _structure.Labels.Store(sec4);

                nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, _sectionInfo[8]/1000);
                nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*_sectionInfo[9]/1000);
                    //generate correct section properties without overlapping thicknesses
                nonst_data5.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, _sectionInfo[9]/1000);
                data5.CalcNonstdGeometry();
                _structure.Labels.Store(sec5);

                nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_H, _sectionInfo[10]/1000);
                nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_B, 2*_sectionInfo[11]/1000);
                    //generate correct section properties without overlapping thicknesses
                nonst_data6.SetValue(IRobotBarSectionNonstdDataValue.I_BSNDV_RECT_T, _sectionInfo[11]/1000);
                data6.CalcNonstdGeometry();
                _structure.Labels.Store(sec6);

                //the values of the cable are already set!
                _structure.Labels.Store(sec7);
            }

            int barId = 5000;

            for (int i = 0; i < _barInfo.Count; i++)
            {
                Console.WriteLine("Now creating bar " + barId.ToString() + "...");
                var b = (AnalyticalBar) _barInfo[i];

                //find the corresponding points in the nodeList to create the bar
                //int robStart = 0;
                //int robEnd = 0;

                //foreach (var n in _nodeInfo)
                //{
                //    if (n.catiaID.Equals(b.CatiaStart))
                //    {
                //        robStart = n.Id;
                //    }
                //    else if (n.catiaID.Equals(b.CatiaEnd))
                //    {
                //        robEnd = n.robotID;
                //    }
                //    else
                //    {
                //        continue;
                //    }
                //}

                //create the bar
                _bars.Create(barId, b.Start.Id, b.End.Id);

                //DON'T CREATE A RELEASE - DIFFICULT TO IMPLEMENT
//				_bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_RELEASE, "CatiaBarRelease");

                if (b.SectionType == SectionType.Tube00)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_00");
                }
                else if (b.SectionType == SectionType.Tube01)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_01");
                }
                else if (b.SectionType == SectionType.Tube02)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaTubeSection_02");
                }
                else if (b.SectionType == SectionType.Plate00)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_00");
                }
                else if (b.SectionType == SectionType.Plate01)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_01");
                }
                else if (b.SectionType == SectionType.Plate02)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_SECTION, "CatiaPlateSection_02");
                }
                else if (b.SectionType == SectionType.Cable)
                {
                    _bars.Get(barId).SetLabel(IRobotLabelType.I_LT_BAR_CABLE, "CatiaCable");
                }

                forDeadLoadsBars.Add(barId);

                b.Id = barId;
                //b.RobotStart = robStart;
                //b.RobotEnd =   robEnd;
                //b.StressAxial = 0.0;

                barId++;
            }
        }

        private void CreateNodes()
        {
            IRobotLabel sup = _structure.Labels.Create(IRobotLabelType.I_LT_SUPPORT, "CatiaSupport");
            IRobotNodeSupportData sup_data = sup.Data as IRobotNodeSupportData;

            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UX, 1);
            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UY, 1);
            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_UZ, 1);
            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RX, 0);
            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RY, 0);
            sup_data.SetFixed(IRobotNodeSupportFixingDirection.I_NSFD_RZ, 0);

            _structure.Labels.Store(sup);

            int nodeId = 100;
            for (int i = 0; i < _nodeInfo.Count; i++)
            {
                Console.WriteLine("Now creating " + nodeId.ToString() + "...");
                var n = (AnalyticalNode) _nodeInfo[i];

                //create the robot node
                _nodes.Create(nodeId, n.X, n.Y, n.Z);

                if (n.IsFixed == true)
                {
                    Console.WriteLine("Fixing node " + nodeId.ToString() + "...");
                    IRobotNode node = _nodes.Get(nodeId) as IRobotNode;
                    node.SetLabel(IRobotLabelType.I_LT_NODE_SUPPORT, "CatiaSupport");
                }

                //set the robot node information in the List
                //to be pushed back into the XML
                n.Id = nodeId; //the robot node Id
                //n.displaceX = 0.0;
                //n.displaceY = 0.0;
                //n.displaceZ = 0.0;

                nodeId++;
            }
        }

        private static bool IsAlmostEqualTo(double a, double b)
        {
            if (b < a + .001 && b > a - .001)
            {
                return true;
            }
            else return false;
        }

        public void Analyze()
		{
			//select all the finite element stuff
			RobotSelection allFE = _structure.Selections.CreateFull(IRobotObjectType.I_OT_FINITE_ELEMENT);
			
			//get the finite element server
			IRobotFiniteElementServer fe_server = _structure.FiniteElems;
			
			//delete the finite element meshes
			fe_server.DeleteMany(allFE);
			
			//consolidate finite elements with a coefficient of one for panels to convex boundaries
			//so two triangle become a square...
			//fe_server.MeshConsolidate(1.0, allFE, false);

			IRobotMeshParams meshParams = _project.Preferences.MeshParams;
			meshParams.MeshType = IRobotMeshType.I_MT_NORMAL;
			meshParams.SurfaceParams.Generation.Division1 = 0;
			meshParams.SurfaceParams.Generation.Division2 = 0;
	
			fe_server.Update();
			fe_server.MeshConcentrate(IRobotMeshRefinementType.I_MRT_SIMPLE, allFE, false);

			_project.CalcEngine.AnalysisParams.IgnoreWarnings = true;
			_project.CalcEngine.AutoGenerateModel = true;
//			_project.CalcEngine.GenerateModel();
		
			int calcResult = _project.CalcEngine.Calculate();

			if (calcResult != 0)
			{
				Console.WriteLine("Model analyzed successfully...");
				//System.Threading.Thread.Sleep(2000);
			}
			else if (calcResult == 0)
			{
				throw new Exception("Robot model could not be analyzed.");
				//System.Threading.Thread.Sleep(2000);
			}
			
            //Console.WriteLine("Saving model with analysis results...");
            //_project.SaveAs(m_robotPath);
            //Console.WriteLine("Robot project saved to " + m_robotPath + "...");
			
            //IRobotPrintEngine printEngine = _project.PrintEngine;
            //printEngine.AddTemplateToReport("Results");
            //printEngine.SaveReportToFile(currDirectory + "\\" + fileName + ".txt", IRobotOutputFileFormat.I_OFF_TEXT);
			
			//InitializeResultsServers();

			//CheckDeflections();
			//CheckStresses();
			//WriteCheckDataToOutputFile(currDirectory, fileName);
		}

        public void GetResults()
		{
			RobotResultServer results = _structure.Results;
			_displacementServer = results.Nodes.Displacements;
			_caseServer = _structure.Cases;
			_stressServer = _structure.Results.Bars.Stresses;
		}
    }
}
