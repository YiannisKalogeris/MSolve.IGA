namespace MGroup.IGA.Elements
{
	using System;
	using System.Collections.Generic;
	using System.Linq;

	using MGroup.IGA.Entities;
	using MGroup.IGA.Entities.Loads;
	using MGroup.IGA.Interfaces;
	using MGroup.IGA.SupportiveClasses;
	using MGroup.LinearAlgebra.Matrices;
	using MGroup.LinearAlgebra.Vectors;
	using MGroup.Materials.Interfaces;
	using MGroup.MSolve.Discretization;
	using MGroup.MSolve.Discretization.FreedomDegrees;
	using MGroup.MSolve.Discretization.Interfaces;
	using MGroup.MSolve.Discretization.Loads;
	using MGroup.MSolve.Discretization.Mesh;

	/// <summary>
	/// A two-dimensional continuum element that utilizes T-Splines for shape functions.
	/// Its formulation is based on Bezier extraction provided by .iga files exported from Rhino.
	/// For more information please refer to <see href="https://www.oden.utexas.edu/media/reports/2014/1433.pdf"/>
	/// Authors: Dimitris Tsapetis.
	/// </summary>
	public class TSplineElement2D : Element, IStructuralIsogeometricElement
	{
		protected static readonly IDofType[] controlPointDOFTypes = { StructuralDof.TranslationX, StructuralDof.TranslationY };
		protected IDofType[][] dofTypes;
		private readonly IReadOnlyList<IContinuumMaterial2D> _materialsAtGaussPoints;

		/// <summary>
		/// Creates a <see cref="TSplineElement2D"/> by providing a list of materials  equal to the number of GaussPoints.
		/// </summary>
		/// <param name="materialsAtGaussPoints"></param>
		public TSplineElement2D(IReadOnlyList<IContinuumMaterial2D> materialsAtGaussPoints)
		{
			this._materialsAtGaussPoints = materialsAtGaussPoints;
		}

		/// <summary>
		/// Retrieves the type of Finite Element used. Since the element is Isogeometric its type is defined as unknown.
		/// </summary>
		public CellType CellType { get; } = CellType.Unknown;

		/// <summary>
		/// Property that Polynomial degree of the T-Splines shape functions per axis Heta.
		/// </summary>
		public int DegreeHeta { get; set; }

		/// <summary>
		/// Property that Polynomial degree of the T-Splines shape functions per axis Ksi.
		/// </summary>
		public int DegreeKsi { get; set; }

		/// <summary>
		/// Defines the way that elemental degrees of freedom will be enumerated.
		/// For further info see <see cref="IElementDofEnumerator"/>
		/// </summary>
		public IElementDofEnumerator DofEnumerator { get; set; } = new GenericDofEnumerator();

		/// <summary>
		/// Retrieves the number of Dimensions of the element.
		/// </summary>
		public ElementDimensions ElementDimensions => ElementDimensions.TwoD;

		/// <summary>
		/// A <see cref="Matrix"/> that contains the Bezier extraction operator.
		/// </summary>
		public Matrix ExtractionOperator { get; set; }

		/// <summary>
		/// Boolean property that determines whether the material used for this elements has been modified.
		/// </summary>
		public bool MaterialModified => throw new NotImplementedException();

		/// <summary>
		/// Calculates the forces applies to an <see cref="TSplineElement2D"/> due to <see cref="MassAccelerationLoad"/>
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <param name="loads">A list of <see cref="MassAccelerationLoad"/>. For more info see <seealso cref="MassAccelerationLoad"/></param>
		/// <returns></returns>
		public double[] CalculateAccelerationForces(IElement element, IList<MassAccelerationLoad> loads)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Calculates displacements of knots for post-processing with Paraview.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <param name="localDisplacements">A <see cref="Matrix"/> containing the displacements for the degrees of freedom of the element.</param>
		/// <returns></returns>
		public double[,] CalculateDisplacementsForPostProcessing(Element element, Matrix localDisplacements)
		{
			var tsplineElement = (TSplineElement2D)element;
			var elementControlPoints = tsplineElement.ControlPoints.ToArray();
			var knotParametricCoordinatesKsi = Vector.CreateFromArray(new double[] { -1, 1 });
			var knotParametricCoordinatesHeta = Vector.CreateFromArray(new double[] { -1, 1 });

			var tsplines = new ShapeTSplines2DFromBezierExtraction(tsplineElement, elementControlPoints, knotParametricCoordinatesKsi, knotParametricCoordinatesHeta);

			var knotDisplacements = new double[4, 3];
			var paraviewKnotRenumbering = new int[] { 0, 3, 1, 2 };
			for (int j = 0; j < knotDisplacements.GetLength(0); j++)
			{
				for (int i = 0; i < elementControlPoints.Length; i++)
				{
					knotDisplacements[paraviewKnotRenumbering[j], 0] += tsplines.TSplineValues[i, j] * localDisplacements[i, 0];
					knotDisplacements[paraviewKnotRenumbering[j], 1] += tsplines.TSplineValues[i, j] * localDisplacements[i, 1];
					knotDisplacements[paraviewKnotRenumbering[j], 2] += tsplines.TSplineValues[i, j] * localDisplacements[i, 2];
				}
			}

			return knotDisplacements;
		}

		/// <summary>
		/// This method calculates the internal forces of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <param name="localDisplacements">A <see cref="double"/> array containing the displacements for the degrees of freedom of the element.</param>
		/// <param name="localdDisplacements">A <see cref="double"/> array containing the displacements change for the degrees of freedom of the element.</param>
		/// <returns>A <see cref="double"/> array containing the forces all degrees of freedom</returns>
		public double[] CalculateForces(IElement element, double[] localDisplacements, double[] localdDisplacements)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// This method is used for retrieving the internal forces of the element for logging purposes.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <param name="localDisplacements">A <see cref="double"/> array containing the displacements for the degrees of freedom of the element.</param>
		/// <returns>A <see cref="double"/> array containing the forces all degrees of freedom</returns>
		public double[] CalculateForcesForLogging(IElement element, double[] localDisplacements)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Cannot be used with <see cref="TSplineElement2D"/>
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		public Dictionary<int, double> CalculateLoadingCondition(Element element, Edge edge, NeumannBoundaryCondition neumann)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Cannot be used with <see cref="TSplineElement2D"/>
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		public Dictionary<int, double> CalculateLoadingCondition(Element element, Face face, NeumannBoundaryCondition neumann)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Cannot be used with <see cref="TSplineElement2D"/>
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		public Dictionary<int, double> CalculateLoadingCondition(Element element, Edge edge, PressureBoundaryCondition pressure)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Cannot be used with <see cref="TSplineElement2D"/>
		/// </summary>
		/// <param name="element"></param>
		/// <param name="edge"></param>
		/// <param name="neumann"></param>
		/// <returns></returns>
		public Dictionary<int, double> CalculateLoadingCondition(Element element, Face face, PressureBoundaryCondition pressure)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// This method calculates the stresses of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <param name="localDisplacements">A <see cref="double"/> array containing the displacements for the degrees of freedom of the element.</param>
		/// <param name="localdDisplacements">A <see cref="double"/> array containing the displacements change for the degrees of freedom of the element.</param>
		/// <returns></returns>
		public Tuple<double[], double[]> CalculateStresses(IElement element, double[] localDisplacements, double[] localdDisplacements)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Clear the material state of the element
		/// </summary>
		public void ClearMaterialState()
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Clear any saved material states of the element.
		/// </summary>
		public void ClearMaterialStresses()
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Calculates the damping matrix of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <returns>An <see cref="IMatrix"/> containing the damping matrix of an <see cref="TSplineElement2D"/></returns>
		public IMatrix DampingMatrix(IElement element)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Retrieves the dofs of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <returns></returns>
		public IReadOnlyList<IReadOnlyList<IDofType>> GetElementDofTypes(IElement element)
		{
			var nurbsElement = (TSplineElement2D)element;
			dofTypes = new IDofType[nurbsElement.ControlPointsDictionary.Count][];
			for (int i = 0; i < nurbsElement.ControlPointsDictionary.Count; i++)
			{
				dofTypes[i] = controlPointDOFTypes;
			}
			return dofTypes;
		}

		/// <summary>
		/// Calculates the mass matrix of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <returns>An <see cref="IMatrix"/> containing the mass matrix of an <see cref="TSplineElement2D"/></returns>
		public IMatrix MassMatrix(IElement element)
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Resets any saved material states of the element to its initial state.
		/// </summary>
		public void ResetMaterialModified()
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Save the current material state of the element.
		/// </summary>
		public void SaveMaterialState()
		{
			throw new NotImplementedException();
		}

		/// <summary>
		/// Calculates the stiffness matrix of the element.
		/// </summary>
		/// <param name="element">An element of type <see cref="TSplineElement2D"/></param>
		/// <returns>An <see cref="IMatrix"/> containing the stiffness matrix of an <see cref="TSplineElement2D"/></returns>
		public IMatrix StiffnessMatrix(IElement element)
		{
			var tsplineElement = (TSplineElement2D)element;
			var elementControlPoints = tsplineElement.ControlPoints.ToArray();
			var gaussPoints = CreateElementGaussPoints(tsplineElement);
			var stiffnessMatrixElement = Matrix.CreateZero(tsplineElement.ControlPointsDictionary.Count * 2, tsplineElement.ControlPointsDictionary.Count * 2);

			ShapeTSplines2DFromBezierExtraction tsplines = new ShapeTSplines2DFromBezierExtraction(tsplineElement, elementControlPoints);

			for (int j = 0; j < gaussPoints.Count; j++)
			{
				var jacobianMatrix = CalculateJacobianMatrix(elementControlPoints, tsplines, j);

				var jacdet = CalculateJacobianDeterminant(jacobianMatrix);

				var B1 = CalculateDeformationMatrix1(jacobianMatrix, jacdet);

				var B2 = CalculateDeformationMatrix2(tsplineElement, tsplines, j);

				Matrix B = B1 * B2;
				IMatrixView ElasticityMatrix = _materialsAtGaussPoints[j].ConstitutiveMatrix;
				Matrix stiffnessMatrixGaussPoint = B.ThisTransposeTimesOtherTimesThis(ElasticityMatrix);
				stiffnessMatrixGaussPoint = stiffnessMatrixGaussPoint * (jacdet * gaussPoints[j].WeightFactor * tsplineElement.Patch.Thickness);

				for (int m = 0; m < elementControlPoints.Length * 2; m++)
				{
					for (int n = 0; n < elementControlPoints.Length * 2; n++)
					{
						stiffnessMatrixElement[m, n] += stiffnessMatrixGaussPoint[m, n];
					}
				}
			}
			return stiffnessMatrixElement;
		}

		private static Matrix CalculateDeformationMatrix1(Matrix jacobianMatrix, double jacdet)
		{
			var B1 = Matrix.CreateZero(3, 4);

			B1[0, 0] += jacobianMatrix[1, 1] / jacdet;
			B1[0, 1] += -jacobianMatrix[0, 1] / jacdet;
			B1[1, 2] += -jacobianMatrix[1, 0] / jacdet;
			B1[1, 3] += jacobianMatrix[0, 0] / jacdet;
			B1[2, 0] += -jacobianMatrix[1, 0] / jacdet;
			B1[2, 1] += jacobianMatrix[0, 0] / jacdet;
			B1[2, 2] += jacobianMatrix[1, 1] / jacdet;
			B1[2, 3] += -jacobianMatrix[0, 1] / jacdet;
			return B1;
		}

		private static Matrix CalculateDeformationMatrix2(TSplineElement2D tsplineElement,
					ShapeTSplines2DFromBezierExtraction tsplines, int j)
		{
			var B2 = Matrix.CreateZero(4, 2 * tsplineElement.ControlPointsDictionary.Count);
			for (int column = 0; column < 2 * tsplineElement.ControlPointsDictionary.Count; column += 2)
			{
				B2[0, column] += tsplines.TSplineDerivativeValuesKsi[column / 2, j];
				B2[1, column] += tsplines.TSplineDerivativeValuesHeta[column / 2, j];
				B2[2, column + 1] += tsplines.TSplineDerivativeValuesKsi[column / 2, j];
				B2[3, column + 1] += tsplines.TSplineDerivativeValuesHeta[column / 2, j];
			}

			return B2;
		}

		private static double CalculateJacobianDeterminant(Matrix jacobianMatrix)
		{
			double jacdet = jacobianMatrix[0, 0] * jacobianMatrix[1, 1]
							- jacobianMatrix[1, 0] * jacobianMatrix[0, 1];
			return jacdet;
		}

		private static Matrix CalculateJacobianMatrix(ControlPoint[] elementControlPoints,
			ShapeTSplines2DFromBezierExtraction tsplines, int j)
		{
			var jacobianMatrix = Matrix.CreateZero(2, 2);
			for (var k = 0; k < elementControlPoints.Length; k++)
			{
				jacobianMatrix[0, 0] += tsplines.TSplineDerivativeValuesKsi[k, j] * elementControlPoints[k].X;
				jacobianMatrix[0, 1] += tsplines.TSplineDerivativeValuesKsi[k, j] * elementControlPoints[k].Y;
				jacobianMatrix[1, 0] += tsplines.TSplineDerivativeValuesHeta[k, j] * elementControlPoints[k].X;
				jacobianMatrix[1, 1] += tsplines.TSplineDerivativeValuesHeta[k, j] * elementControlPoints[k].Y;
			}

			return jacobianMatrix;
		}

		private IList<GaussLegendrePoint3D> CreateElementGaussPoints(TSplineElement2D element)
		{
			GaussQuadrature gauss = new GaussQuadrature();
			return gauss.CalculateElementGaussPoints(element.DegreeKsi, element.DegreeHeta, new List<Knot>
				{
					new Knot(){ID=0,Ksi=-1,Heta = -1,Zeta = 0},
					new Knot(){ID=1,Ksi=-1,Heta = 1,Zeta = 0},
					new Knot(){ID=2,Ksi=1,Heta = -1,Zeta = 0},
					new Knot(){ID=3,Ksi=1,Heta = 1,Zeta = 0}
				});
		}
	}
}
