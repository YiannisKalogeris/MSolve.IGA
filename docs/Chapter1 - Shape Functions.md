# Shape functions
This file will provide the specific for the shape functions implemented in **MSolve.IGA**.

## B-Splines
B-Splines are considered the basis for all advanced shape function used for both Computer Aided Design (CAD) and Isogeometric Analysis. Given a set of non-decreasing values 

![Knot Value Vector ](../docs/Images/KnotValueVector.png)

and a polynomial degree **p**, a multitude of **n** B-Spline shape function can be evaluated using Cox-de-Boor recursive algorithm. For constant B-Splines we have:

![Constant B-Splines](../docs/Images/ConstantBSplines.png)

For degrees **p**>0 we have:

![p>0 B-Splines](../docs/Images/BSplines.png)

with the assumption that 0/0=0. For further information regarding B-Splines please refer to [1].
Below a usage sample of the **MSolve.IGA** code for calculating B-Splines is provide. The parametric coordinates variable denotes the point at which the B-Splines will be evaluated.

```csharp
    var degree = 3;
	var knotValueVector = Vector.CreateFromArray(new double[] { 0, 0, 0, 0, 1/9.0, 2/9.0, 3/9.0, 4/9.0, 5/9.0, 6/9.0, 7/9.0, 8/9.0, 1, 1, 1, 1 });
	var parametricCoordinates = Vector.CreateFromArray(new double[] {0.007714649348171322, 0.0366677197641736, 0.0744433912358264, 0.10339646165182867});
	var bsplines1D = new BSPLines1D(degree, knotValueVector, parametricCoordinates);
```

## Non-Uniform Rational B-Splines (NURBS)
NURBS are a generalization of B-Splines shape functions, as the Control Points that generate the geometries apart from their cartesian cooordinates **X**, **Y**, **Z** also have a weight **W**. For the computation of the NURBS shape functions a weighting function is introduced. 

![Weighting function](../docs/Images/NurbsWeightingFunction.png)

Utilizing the latter one-dimensional NURBS shape functions are evaluated as follows:

![Univariate nurbs](../docs/Images/UnivariateNurbs.png)

Two-dimensional NURBS are calcute in a tensor-product fashion by combining one-dimensional NURBS as follows:

![Bivariate nurbs](../docs/Images/BivariateNurbs.png)

And for three-dimensional NURBS equivalently:

![Trivariate nurbs](../docs/Images/TrivariateNurbs.png)

For more information please refer to [1]. 

```csharp
var degreeKsi=3;
var degreeHeta=3;
var knotValueVectorKsi = Vector.CreateFromArray(new double[] { 0,0,0,0,0.25,0.5,0.75,1,1,1,1 });
var knotValueVectorKsi = Vector.CreateFromArray(new double[] { 0,0,0,0,0.25,0.5,0.75,1,1,1,1 });
var integrationPoint=new NaturalPoint(0.125,0.125,0.125);
var controlPoints = new List<ControlPoint>
{
    new ControlPoint() {ID=..., X=..., Y=..., Z=..., Ksi=..., Heta=..., Zeta=..., WeightFactor=...},
    ...
}
var shapeFunction = Nurbs2D(degreeKsi,degreeHeta,knotValueVectorKsi, knotValueVectorHeta, integrationPoint, controlPoints);
```

## T-Splines
A detailed introduction to T-Splines is provided in [2]. In the context of the current code, the work of [3] is implemented as it offers an immediate connection of Rhino software and an analysis code. Similar to NURBS the rational T-Spline shape functions are calculated as follows:

![Univariate T-Splines](../docs/Images/UnivariateTSplines.png)

where **e** refers to the element. In case Bezier extraction is performed to generate Bezier elements from the T-Spline basis the connection between the two function spaces is the following.

![Bezier extraction](../docs/Images/TSplinesToBezier.png)

where C denotes the Bezier extraction operator. As a result of the last two equations the rational T-SplineShapeFunctions can be written as:

![T-Splines from BEzier](../docs/Images/BezierExtraction.png)

An example of the code used for calculating the shape functions of a Bezier extracted T-Spline is provided below.
```csharp
Element element = new TSplineElement2D(null)
{
	ID = elementIDCounter,
	Patch = _model.PatchesDictionary[0],
	ElementType = new TSplineElement2D(null),
	DegreeKsi = elementDegreeKsi,
	DegreeHeta = elementDegreeHeta,
	ExtractionOperator = extractionOperator
};
foreach (var t in connectivity)
	element.AddControlPoint(_model.ControlPointsDictionary[t]);

var tsplines = new ShapeTSplines2DFromBezierExtraction(tsplineElement, elementControlPoints);

```



### References
[1] [L. Piegl, W. Tiller, The NURBS Book (Monographs in Visual Communication), Second ed., Springer-Verlag, New York, 1997](https://www.springer.com/gp/book/9783642973857)

[2] [Y. Bazilevs , V.M. Calo , J.A. Cottrell , J.A. Evans , T.J.R. Hughes , S. Lipton , M.A. Scott , T.W. Sederberg , Isogeometric analysis using T-splines, Comput. Meth. Appl. Mech. Eng. 199 (2010) 229–263](https://www.sciencedirect.com/science/article/pii/S0045782509000875)

[3] [M. Scott, T. Hughes, T. Sederberg, and M. Sederberg, “An integrated approach to engineering design and analysis using the Autodesk T-spline plugin for Rhino3d,” Adv. Eng. …, 2013.](https://www.oden.utexas.edu/media/reports/2014/1433.pdf)