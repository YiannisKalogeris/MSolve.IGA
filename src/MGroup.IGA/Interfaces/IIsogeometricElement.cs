namespace MGroup.IGA.Interfaces
{
	using System.Collections.Generic;

	using MGroup.IGA.Entities;
	using MGroup.IGA.Entities.Loads;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.MSolve.Discretization.Interfaces;

	/// <summary>
	/// Isogeometric element interface. Implements <see cref="IElementType"/>
	/// </summary>
	public interface IIsogeometricElement : IElementType
	{
		/// <summary>
		/// Retries the element dimensions.
		/// </summary>
		ElementDimensions ElementDimensions { get; }

		/// <summary>
		/// Retrieves the element ID.
		/// </summary>
		int ID { get; }

		/// <summary>
		/// Calculates the knot displacements for post-processing with Paraview.
		/// </summary>
		/// <param name="element"></param>
		/// <param name="localDisplacements"></param>
		/// <returns></returns>
		double[,] CalculateDisplacementsForPostProcessing(Element element, Matrix localDisplacements);

		/// <summary>
		/// Neumann loading condition imposed on edges.
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		Dictionary<int, double> CalculateLoadingCondition(Element element, Edge edge, NeumannBoundaryCondition neumann);

		/// <summary>
		/// Neumann loading condition imposed on faces.
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		Dictionary<int, double> CalculateLoadingCondition(Element element, Face face, NeumannBoundaryCondition neumann);

		/// <summary>
		/// Pressure loading condition imposed on edges.
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="pressure"></param>
		/// <returns></returns>
		Dictionary<int, double> CalculateLoadingCondition(Element element, Edge edge, PressureBoundaryCondition pressure);

		/// <summary>
		/// Pressure loading condition imposed on faces.
		/// </summary>
		/// <param name="element"></param>
		/// <param name="face"></param>
		/// <param name="pressure"></param>
		/// <returns></returns>
		Dictionary<int, double> CalculateLoadingCondition(Element element, Face face, PressureBoundaryCondition pressure);
	}
}