namespace MGroup.IGA.Entities.Loads
{
	using System.Collections.Generic;

    /// <summary>
    /// Delegate that calculates the load distribution that can vary depending on the position.
    /// </summary>
    /// <param name="x">Cartesian coordinate x of the load.</param>
    /// <param name="y">Cartesian coordinate y of the load.</param>
    /// <param name="z">Cartesian coordinate y of the load.</param>
    /// <returns></returns>
    public delegate double[] Value(double x, double y, double z);

    /// <summary>
    /// Calculates the load depending on the the type of boundary condition and geometrical entity enforced.
    /// </summary>
    public class LoadProvider
    {
        /// <summary>
        /// Calculates Neumann load on an edge.
        /// </summary>
        /// <param name="element"></param>
        /// <param name="edge"></param>
        /// <param name="neumann"></param>
        /// <returns></returns>
        public Dictionary<int, double> LoadNeumann(Element element, Edge edge, NeumannBoundaryCondition neumann)
        {
            return element.ElementType.CalculateLoadingCondition(element, edge, neumann);
        }

        /// <summary>
        /// Calculates Neumann load on a face.
        /// </summary>
        /// <param name="element"></param>
        /// <param name="face"></param>
        /// <param name="neumann"></param>
        /// <returns></returns>
        public Dictionary<int, double> LoadNeumann(Element element, Face face, NeumannBoundaryCondition neumann)
        {
            return element.ElementType.CalculateLoadingCondition(element, face, neumann);
        }

        /// <summary>
        /// Calculates Pressure load on an edge.
        /// </summary>
        /// <param name="element"></param>
        /// <param name="edge"></param>
        /// <param name="pressure"></param>
        /// <returns></returns>
        public Dictionary<int, double> LoadPressure(Element element, Edge edge, PressureBoundaryCondition pressure)
        {
            return element.ElementType.CalculateLoadingCondition(element, edge, pressure);
        }

        /// <summary>
        /// Calculates Pressure load on a face.
        /// </summary>
        /// <param name="element"></param>
        /// <param name="face"></param>
        /// <param name="pressure"></param>
        /// <returns></returns>
        public Dictionary<int, double> LoadPressure(Element element, Face face, PressureBoundaryCondition pressure)
        {
            return element.ElementType.CalculateLoadingCondition(element, face, pressure);
        }
    }

    /// <summary>
    /// Neumann loading boundary condition. Works as a distributed load.
    /// </summary>
    public class NeumannBoundaryCondition : LoadingCondition
    {
        /// <summary>
        /// Defines a Neumann BoundaryCondition
        /// </summary>
        /// <param name="neumannValue"></param>
        public NeumannBoundaryCondition(Value neumannValue)
        {
            this.Value = neumannValue;
        }

        /// <summary>
        /// Describes the distribution of the load. See <see cref="Loads.Value"/>
        /// </summary>
        public Value Value { get; private set; }
    }
}