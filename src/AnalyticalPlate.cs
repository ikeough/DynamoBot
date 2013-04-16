using System.Collections.Generic;

namespace Dynamo.Analysis
{
    public enum PlateType : byte
	{
		Slab = 1,
		Panel = 2,
		Shear = 3
	}

    public class AnalyticalPlate : AnalyticalModelBase
    {
        //!The list of Robot Nodes which make up the plate element.
        public List<int> Nodes { get; set; }

        public double WindLoad { get; private set; }

        public double LiveLoad { get; private set; }

        public double BuildingLoad { get; private set; }

        public PlateType PlateType { get; set; }

        public double Thickness { get; private set; }

        //!The Plate constructor.
        public AnalyticalPlate(List<int> nodes, double[] loads, double thickness)
        {
            Nodes = nodes;
            WindLoad = loads[0];
            LiveLoad = loads[1];
            BuildingLoad = loads[2];
            Thickness = thickness;
        }
    }
}
