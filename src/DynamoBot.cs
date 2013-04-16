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
using System.Collections;
using System.Collections.Generic;
using System.Diagnostics;
using System.IO;
using System.Linq;
using System.Windows.Forms;
using Dynamo.Connectors;
using Dynamo.Nodes;
using Dynamo.Utilities;
using Microsoft.FSharp.Collections;
using RobotOM;
using Dynamo.FSchemeInterop;

using Value = Dynamo.FScheme.Value;

namespace Dynamo.Analysis
{
	class RobotLink
	{

		static List<int> m_failedNodes = new List<int>();
		static List<int> m_failedBars = new List<int>();
		static double m_displacementTolerance = 0.0;	//13mm mm->M
		static double m_stressTolerance = .6 * 248211262.8;	//36ksi(steel) -> Pascal * .6 (per ASD)
		static double m_windx, m_windy, m_windz;	//the wind vector

		static string m_errorMessage = "";

		



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

    [NodeName("RobotModel")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical model in Robot.")]
    public class DynamoRobotModel : dynNodeWithOneOutput
    {
        private RobotEngine _eng;

        public DynamoRobotModel()
        {
            InPortData.Add(new PortData("eng", "The Robot analysis engine.", typeof(object)));
            InPortData.Add(new PortData("nodes", "A list of analytical nodes.", typeof(object)));
            InPortData.Add(new PortData("bars", "A list of analytical bars.", typeof(object)));
            InPortData.Add(new PortData("plates", "A list of analytical plates.", typeof(object)));

            OutPortData.Add(new PortData("model", "The analytical model.", typeof(object)));
        }

        public override FScheme.Value Evaluate(FSharpList<FScheme.Value> args)
        {
            _eng = (RobotEngine)((Value.Container)args[0]).Item;

            var nodes;
            var bars;
            var plates;

            RobotModel model = new RobotModel(_eng.Application, nodes, bars, plates);

            model.Analyze();
            model.GetResults();

            return Value.NewContainer(model);
        }
    }

    [NodeName("RobotEngine")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical link with Robot.")]
    public class DynamoRobotEngine : dynNodeWithOneOutput
    {
        public DynamoRobotEngine()
        {
            InPortData.Add(new PortData("nodes", "The point(s) from which to define analytical nodes.", typeof(object)));
            InPortData.Add(new PortData("bars", "The curves from which to define analytical bars.", typeof(object)));
            InPortData.Add(new PortData("plates", "The faces from which to define analytical plates.", typeof(object)));
            InPortData.Add(new PortData("path", "The path to the results file.", typeof(string)));
            OutPortData.Add(new PortData("results", "The analytical results.", typeof(object)));

            NodeUI.RegisterAllPorts();

        }

        public override FScheme.Value Evaluate(FSharpList<FScheme.Value> args)
        {
            var nodes = (args[0] as Value.List).Item;
            var bars = (args[1] as Value.List).Item;
            var plates = (args[2] as Value.List).Item;

            throw new NotImplementedException();
        }
    }

	//!The Node Class
    [NodeName("DynamoBotNode")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical node in Robot.")]
	public class DynAnalyticalNode : dynNodeWithOneOutput
	{
		//!The node constructor
        public DynAnalyticalNode()
		{
            InPortData.Add(new PortData("x", "The X location.", typeof(double)));
            InPortData.Add(new PortData("y", "The Y location.", typeof(double)));
            InPortData.Add(new PortData("z", "The Z location.", typeof(double)));
            InPortData.Add(new PortData("fix", "Should the node be fixed?", typeof(bool)));

            OutPortData.Add(new PortData("node", "The analytical node", typeof(bool)));
		}

        public override FScheme.Value Evaluate(FSharpList<FScheme.Value> args)
        {
            var x = (double)((Value.Number)args[0]).Item;
            var y = (double)((Value.Number)args[1]).Item;
            var z = (double)((Value.Number)args[2]).Item;
            var isFixed = Convert.ToBoolean((double)((Value.Number)args[3]).Item);

            var n = new AnalyticalNode(x, y, z, isFixed);
            return Value.NewContainer(n);
        }
	}
	
	//!The Bar Class
    [NodeName("DynamoBotBar")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical bar in Robot.")]
	public class DynAnalyticalBar : dynNodeWithOneOutput
	{		
		//!The Bar constructor.
		public DynAnalyticalBar()
		{
            InPortData.Add(new PortData("start", "The start node of the bar.", typeof(AnalyticalNode)));
            InPortData.Add(new PortData("end", "The end node of the bar.", typeof(AnalyticalNode)));
            InPortData.Add(new PortData("n", "The diameter of the bar.", typeof(double)));
            InPortData.Add(new PortData("n", "The section type of the bar.", typeof(SectionType)));
            InPortData.Add(new PortData("n", "The section thickness of the bar.", typeof(double)));
            InPortData.Add(new PortData("n", "The the end release of the bar.", typeof(string)));
            OutPortData.Add(new PortData("bar", "The analytical bar.", typeof(Bar)));

            NodeUI.RegisterAllPorts();
		}

        public override Value Evaluate(FSharpList<Value> args)
        {
            var start = ((Value.Container)args[0]).Item;
            var end = ((Value.Container)args[1]).Item;
            var diameter = ((Value.Number)args[2]).Item;
            var sectionType = ((Value.Container)args[3]).Item;
            var thickness = ((Value.Number)args[4]).Item;
            var endRelease = ((Value.String)args[5]).Item; 

            var b = new AnalyticalBar(start, end, diameter, sectionType, thickness, endRelease);
            return Value.NewContainer(b);
        }
	}
	
	//!The Plate Class
    [NodeName("DynamoBotPlate")]
    [NodeCategory(BuiltinNodeCategories.ANALYSIS)]
    [NodeDescription("Creates a structural analytical plate in Robot.")]
	public class DynAnalyticalPlate
	{
		
		//!The Plate constructor.
		public DynAnalyticalPlate ()
		{
			
		}
		
	}

}
