using System;
using System.Collections.Generic;
using System.Linq;
using System.Text;

namespace Dynamo.Analysis
{
    public enum SectionType : byte
    {
        Tube00 = 1,
        Tube01 = 2,
        Tube02 = 3,
        Plate00 = 4,
        Plate01 = 5,
        Plate02 = 6,
        Cable = 7
    }

    //!The Bar Class
    public class AnalyticalBar : AnalyticalModelBase
    {
        #region properties

        //The start node.
        public AnalyticalNode Start { get; private set; }

        //The the end node.
        public AnalyticalNode End { get; private set; }

        //The diameter of the bar.
        public double Diameter { get; private set; }

        //The section thickness.
        public double SectionThickness { get; private set; }

        //The section type - bar or plate.
        public SectionType SectionType { get; private set; }

        //The end release.
        public string EndRelease { get; private set; }

        #endregion

        /// <summary>
        /// Create an analytical bar.
        /// </summary>
        /// <param name="start">The start node.</param>
        /// <param name="end">The end node.</param>
        /// <param name="diameter">The diameter.</param>
        /// <param name="sectionType">The section type.</param>
        /// <param name="thickness">The thickness.</param>
        /// <param name="endRelease">The end release.</param>
        public AnalyticalBar(AnalyticalNode start, AnalyticalNode end, double diameter, SectionType sectionType, double thickness, string endRelease)
        {
            Start = start;
            End = end;
            Diameter = diameter;
            SectionType = sectionType;
            SectionThickness = thickness;
            EndRelease = endRelease;
        }
    }
}
