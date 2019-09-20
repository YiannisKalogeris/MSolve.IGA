# Examples

## NURBS quadratic cantilever with distributed load.

```csharp
Model model = new Model();
ModelCreator modelCreator = new ModelCreator(model);
var filename = "Cantilever2D";
string filepath = $"..\\..\\..\\MGroup.IGA.Tests\\InputFiles\\{filename}.txt";
IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filepath);
modelReader.CreateModelFromFile();

// Forces and Boundary Conditions
Value verticalDistributedLoad = (x, y, z) => new double[] {0, -100, 0};
model.PatchesDictionary[0].EdgesDictionary[1].LoadingConditions
    .Add(new NeumannBoundaryCondition(verticalDistributedLoad));

// Boundary Conditions
foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[0].ControlPointsDictionary
    .Values)
{
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
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
```

## NURBS Beam3D

```csharp
Model model = new Model();
ModelCreator modelCreator = new ModelCreator(model);
string filename = "Beam3D";
string filepath = $"..\\..\\..\\InputFiles\\{filename}.txt";
IsogeometricReader modelReader = new IsogeometricReader(modelCreator, filepath);
modelReader.CreateModelFromFile();

// Forces and Boundary Conditions
foreach (ControlPoint controlPoint in model.PatchesDictionary[0].FacesDictionary[1].ControlPointsDictionary
    .Values)
{
    model.Loads.Add(new Load()
    {
        Amount = -100,
        Node = model.ControlPointsDictionary[controlPoint.ID],
        DOF = StructuralDof.TranslationZ
    });
}

// Boundary Conditions
foreach (ControlPoint controlPoint in model.PatchesDictionary[0].FacesDictionary[0].ControlPointsDictionary
    .Values)
{
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
}

// Solvers
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

var paraview = new ParaviewNurbs3D(model, solver.LinearSystems[0].Solution, filename);
paraview.CreateParaviewFile();
```

## NURBS Square shell

```csharp
Model model = new Model();
var filename = "SquareShell";
string filepath = $"..\\..\\..\\MGroup.IGA.Tests\\InputFiles\\{filename}.txt";
IsogeometricShellReader modelReader = new IsogeometricShellReader(model, filepath);
modelReader.CreateShellModelFromFile();

Matrix<double> loadVector =
    MatlabReader.Read<double>("..\\..\\..\\MGroup.IGA.Tests\\InputFiles\\SquareShell.mat", "LoadVector");

for (int i = 0; i < loadVector.ColumnCount; i += 3)
{
    var indexCP = i / 3;
    model.Loads.Add(new Load()
    {
        Amount = loadVector.At(0, i),
        Node = model.ControlPointsDictionary[indexCP],
        DOF = StructuralDof.TranslationX
    });
    model.Loads.Add(new Load()
    {
        Amount = loadVector.At(0, i + 1),
        Node = model.ControlPointsDictionary[indexCP],
        DOF = StructuralDof.TranslationY
    });
    model.Loads.Add(new Load()
    {
        Amount = loadVector.At(0, i + 2),
        Node = model.ControlPointsDictionary[indexCP],
        DOF = StructuralDof.TranslationZ
    });
}

foreach (var edge in model.PatchesDictionary[0].EdgesDictionary.Values)
{
    foreach (var controlPoint in edge.ControlPointsDictionary.Values)
    {
        model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
        model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
        model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationZ });
    }
}

// Solvers
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

var paraview = new ParaviewNurbsShells(model, solver.LinearSystems[0].Solution, filename);
paraview.CreateParaview2DFile();
```


## T-Spline Cantilever shell

```csharp
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
```