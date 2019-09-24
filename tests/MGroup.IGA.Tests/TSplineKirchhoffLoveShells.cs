using System.Linq;
using MGroup.Analyzers;
using MGroup.IGA.Entities;
using MGroup.IGA.Postprocessing;
using MGroup.IGA.Readers;
using MGroup.LinearAlgebra.Vectors;
using MGroup.Materials;
using MGroup.Materials.ShellMaterials;
using MGroup.MSolve.Discretization;
using MGroup.MSolve.Discretization.FreedomDegrees;
using MGroup.MSolve.Discretization.Loads;
using MGroup.Problems;
using MGroup.Solvers;
using MGroup.Solvers.Direct;
using MGroup.Solvers.Ordering;
using MGroup.Solvers.Ordering.Reordering;

namespace MGroup.IGA.Tests
{
	using Xunit;

	public class TSplineKirchhoffLoveShells
	{
		[Fact]
		public void CantileverShellBenchmark()
		{
			Model model = new Model();
			var filename = "CantileverShell";
			string filepath = $"..\\..\\..\\MGroup.IGA.Tests\\InputFiles\\{filename}.iga";
			IgaFileReader modelReader = new IgaFileReader(model, filepath);
			modelReader.CreateTSplineShellsModelFromFile();

			model.PatchesDictionary[0].Material = new ElasticMaterial2D(StressState2D.PlaneStress)
			{
				PoissonRatio = 0.0,
				YoungModulus = 100
			};
			model.PatchesDictionary[0].Thickness = 1;

			foreach (var controlPoint in model.ControlPointsDictionary.Values.Where(cp => cp.X < 3))
			{
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
			}

			foreach (var controlPoint in model.ControlPointsDictionary.Values.Where(cp => cp.X > 49.8))
			{
				model.Loads.Add(new Load()
				{
					Amount = -0.5,
					Node = model.ControlPointsDictionary[controlPoint.ID],
					DOF = StructuralDof.TranslationZ
				});
			}

			var solverBuilder = new DenseMatrixSolver.Builder();
			solverBuilder.DofOrderer = new DofOrderer(
				new NodeMajorDofOrderingStrategy(), new NullReordering());
			ISolver solver = solverBuilder.BuildSolver(model);

			// Structural problem provider
			var provider = new ProblemStructural(model, solver);

			// Linear static analysis
			var childAnalyzer = new LinearAnalyzer(model, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, solver, provider, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			var paraview = new ParaviewTsplineShells(model, solver.LinearSystems[0].Solution, filename);
			paraview.CreateParaviewFile();

			var expectedSolutionVector = Vector.CreateFromArray(new double[]
			{
				0, 0, -306.122431, 0, 0, -1552.478121, 0, 0, -3454.810388, 0, 0, -5881.924153, 0, 0, -8702.62361, 0, 0,
				-11785.71439, 0, 0, -13928.57064, 0, 0, -15000.0008, 0, 0, -306.1224369, 0, 0, -1552.47811, 0, 0,
				-3454.810407, 0, 0, -5881.924117, 0, 0, -8702.623683, 0, 0, -11785.71423, 0, 0, -13928.57093, 0, 0,
				-15000.00025, 0, 0, -306.1224493, 0, 0, -1552.478088, 0, 0, -3454.810449, 0, 0, -5881.924038, 0, 0,
				-8702.623837, 0, 0, -11785.71389, 0, 0, -13928.57157, 0, 0, -14999.99909, 0, 0, -306.1224494, 0, 0,
				-1552.478088, 0, 0, -3454.810449, 0, 0, -5881.924038, 0, 0, -8702.623837, 0, 0, -11785.71389, 0, 0,
				-13928.57157, 0, 0, -14999.99909, 0, 0, -306.1224369, 0, 0, -1552.47811, 0, 0, -3454.810407, 0, 0,
				-5881.924117, 0, 0, -8702.623683, 0, 0, -11785.71423, 0, 0, -13928.57093, 0, 0, -15000.00025, 0, 0,
				-306.122431, 0, 0, -1552.478121, 0, 0, -3454.810388, 0, 0, -5881.924154, 0, 0, -8702.62361, 0, 0,
				-11785.71439, 0, 0, -13928.57064, 0, 0, -15000.0008
			});
			for (int i = 0; i < expectedSolutionVector.Length; i++)
			{
				Assert.True(Utilities.AreValuesEqual(expectedSolutionVector[i], solver.LinearSystems[0].Solution[i],
					1e-8));
			}
		}

		[Fact]
		public void CantileverShellMaterialBenchmark()
		{
			Model model = new Model();
			string filename = "..\\..\\..\\MGroup.IGA.Tests\\InputFiles\\CantileverShell.iga";
			IgaFileReader modelReader = new IgaFileReader(model, filename);

			var thickness = 1.0;

			modelReader.CreateTSplineShellsModelFromFile(TSplineShellType.Thickness, new ShellElasticMaterial2D
			{
				PoissonRatio = 0.0,
				YoungModulus = 100,
			}, thickness);
			foreach (var controlPoint in model.ControlPointsDictionary.Values.Where(cp => cp.X < 3))
			{
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
				model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
			}

			foreach (var controlPoint in model.ControlPointsDictionary.Values.Where(cp => cp.X > 49.8))
			{
				model.Loads.Add(new Load()
				{
					Amount = -0.5,
					Node = model.ControlPointsDictionary[controlPoint.ID],
					DOF = StructuralDof.TranslationZ
				});
			}

			var solverBuilder = new SkylineSolver.Builder();
			ISolver solver = solverBuilder.BuildSolver(model);

			// Structural problem provider
			var provider = new ProblemStructural(model, solver);

			// Linear static analysis
			var childAnalyzer = new LinearAnalyzer(model, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, solver, provider, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			var expectedSolutionVector = Vector.CreateFromArray(new double[]
			{
				0, 0, -306.122431, 0, 0, -1552.478121, 0, 0, -3454.810388, 0, 0, -5881.924153, 0, 0, -8702.62361, 0, 0,
				-11785.71439, 0, 0, -13928.57064, 0, 0, -15000.0008, 0, 0, -306.1224369, 0, 0, -1552.47811, 0, 0,
				-3454.810407, 0, 0, -5881.924117, 0, 0, -8702.623683, 0, 0, -11785.71423, 0, 0, -13928.57093, 0, 0,
				-15000.00025, 0, 0, -306.1224493, 0, 0, -1552.478088, 0, 0, -3454.810449, 0, 0, -5881.924038, 0, 0,
				-8702.623837, 0, 0, -11785.71389, 0, 0, -13928.57157, 0, 0, -14999.99909, 0, 0, -306.1224494, 0, 0,
				-1552.478088, 0, 0, -3454.810449, 0, 0, -5881.924038, 0, 0, -8702.623837, 0, 0, -11785.71389, 0, 0,
				-13928.57157, 0, 0, -14999.99909, 0, 0, -306.1224369, 0, 0, -1552.47811, 0, 0, -3454.810407, 0, 0,
				-5881.924117, 0, 0, -8702.623683, 0, 0, -11785.71423, 0, 0, -13928.57093, 0, 0, -15000.00025, 0, 0,
				-306.122431, 0, 0, -1552.478121, 0, 0, -3454.810388, 0, 0, -5881.924154, 0, 0, -8702.62361, 0, 0,
				-11785.71439, 0, 0, -13928.57064, 0, 0, -15000.0008
			});
			for (int i = 0; i < expectedSolutionVector.Length; i++)
			{
				Assert.True(Utilities.AreValuesEqual(expectedSolutionVector[i], solver.LinearSystems[0].Solution[i],
					1e-6));
			}
		}

		//[Fact]
		public void SimpleHoodBenchmark()
		{
			Model model = new Model();
			var filename = "attempt2";
			string filepath = $"..\\..\\..\\InputFiles\\{filename}.iga";
			IgaFileReader modelReader = new IgaFileReader(model, filepath);

			//var thickness = 1.0;

			//modelReader.CreateTSplineShellsModelFromFile(IgaFileReader.TSplineShellTypes.LinearMaterial,new ShellElasticMaterial2D
			//{
			//  PoissonRatio = 0.3,
			//  YoungModulus = 1e5,
			//}, thickness);
			modelReader.CreateTSplineShellsModelFromFile();

			model.PatchesDictionary[0].Material = new ElasticMaterial2D(StressState2D.PlaneStress)
			{
				PoissonRatio = 0.3,
				YoungModulus = 10000
			};
			model.PatchesDictionary[0].Thickness = 1;

			for (int i = 0; i < 100; i++)
			{
				var id = model.ControlPointsDictionary[i].ID;
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
			}

			for (int i = model.ControlPointsDictionary.Count - 100; i < model.ControlPointsDictionary.Count; i++)
			{
				var id = model.ControlPointsDictionary[i].ID;
				model.Loads.Add(new Load()
				{
					Amount = 100,
					Node = model.ControlPointsDictionary[id],
					DOF = StructuralDof.TranslationZ
				});
			}

			var solverBuilder = new SkylineSolver.Builder();
			ISolver solver = solverBuilder.BuildSolver(model);

			// Structural problem provider
			var provider = new ProblemStructural(model, solver);

			// Linear static analysis
			var childAnalyzer = new LinearAnalyzer(model, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, solver, provider, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			var paraview = new ParaviewTsplineShells(model, solver.LinearSystems[0].Solution, filename);
			paraview.CreateParaviewFile();
		}

		//[Fact]
		public void SimpleHoodBenchmarkMKL()
		{
			Model model = new Model();
			var filename = "attempt2";
			string filepath = $"..\\..\\..\\InputFiles\\{filename}.iga";
			IgaFileReader modelReader = new IgaFileReader(model, filepath);

			var thickness = 1.0;

			modelReader.CreateTSplineShellsModelFromFile(TSplineShellType.Thickness, new ShellElasticMaterial2D()
			{
				PoissonRatio = 0.3,
				YoungModulus = 10000
			}, thickness);

			for (int i = 0; i < 100; i++)
			{
				var id = model.ControlPointsDictionary[i].ID;
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
				model.ControlPointsDictionary[id].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
			}

			for (int i = model.ControlPointsDictionary.Count - 100; i < model.ControlPointsDictionary.Count; i++)
			{
				var id = model.ControlPointsDictionary[i].ID;
				model.Loads.Add(new Load()
				{
					Amount = 100,
					Node = model.ControlPointsDictionary[id],
					DOF = StructuralDof.TranslationZ
				});
			}
			var solverBuilder = new SkylineSolver.Builder();
			//solverBuilder.DofOrderer = new DofOrderer(
			//  new NodeMajorDofOrderingStrategy(), AmdReordering.CreateWithSuiteSparseAmd());
			ISolver solver = solverBuilder.BuildSolver(model);

			// Structural problem provider
			var provider = new ProblemStructural(model, solver);

			// Linear static analysis
			var childAnalyzer = new LinearAnalyzer(model, solver, provider);
			var parentAnalyzer = new StaticAnalyzer(model, solver, provider, childAnalyzer);

			// Run the analysis
			parentAnalyzer.Initialize();
			parentAnalyzer.Solve();

			//var paraview = new ParaviewTsplineShells(model, solver.LinearSystems[0].Solution, filename);
			//paraview.CreateParaviewFile();
		}
	}
}
