using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Dynamo.Analysis
{
    //!The Node Class
    public class AnalyticalNode : AnalyticalModelBase
    {
        #region properties
        //*!The X coordinate of the node.
        public double X { get; private set; }

        //!The Y coordinate of the node.
        public double Y { get; private set; }

        //!The Z coordinate of the node.
        public double Z { get; private set; }

        //!The fixity of the node
        public bool IsFixed { get; private set; }

        #endregion properties

        //!The node constructor
        public AnalyticalNode(double x, double y, double z, bool isFixed)
        {
            X = x;
            Y = y;
            Z = z;
            IsFixed = isFixed;
        }
    }
}
