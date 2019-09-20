![alt text](http://mgroup.ntua.gr/wp-content/uploads/2018/05/MGroup52.png "MGroup")

# MSolve.IGA
MSolve library that support the spatial discretization method of Isogeometric Analysis.

## Features

- **Readers:** Custom file readers are supported for readding an isogeometric model from file
  * .iga File reader. Generates an isogeometric model from Isogeometric Analysis files exported from Autodesk T-Spline plug-in fro Rhino. This reader support the generation of both planar T-Splines 2D geometries as well as T-Splines shells.
  * Custom file readers. Used for reading isogeometric files from custom .txt files. It can be used for 2D and 3D Nurbs geometries and Nurbs shells models.
  
- **Shape functions:** 
  * Non-uniform Rational B-Splines (NURBS). One, two and three dimension shape functions are supported.
  * T-Splines from Bezier extraction. Utiling the exported format from .iga files T-Spline shape functions are calculated with the aid of the extraction operator.
  
- **Elements:** Both continuum and shell elements are supported. Specifically:
  * Continuum Elements
    1. Nurbs Element 1D: One-dimensional continuum element based on NURBS shape functions. It is utilized for calculation of boundary loads on edges of isogeometric patches.
    2. Nurbs Element 2D: Two-dimensional continuum element based on tensor product NURBS. It is used for both the analysis of plane-stress/plane strain models and the calculation of surface boundary loads in case of three-dimensional isogeometric models.
    3. Nurbs Element 3D: Three dimensional continuum element based on tensor product NURBS.
    4. T-Spline 2D: They are based on Bezier extraction feature provided by T-Spline plug-in for Rhino.
    
  * Structural Elements
    1. Nurbs Kirchhoff-Love Shell Element: Structural shell element utilizing NURBS based on the Kirchhoff-Love theory for thin-shells. Geometrically linear formulation.
    2. T-Spline Kirchhoff-Love Shell Element: Structural shell element utilizing T-Splines based on the Kirchhoff-Love theory for thin-shells. Geometrically linear formulation supporting both plane-stress materials, as well as arbitrary 3D materials such as elastoplasticity by providing a throught thickness integration regime.
    
- **Loading conditions:**    
  * Neumann boundary condition: Distributed load that can be applied to edges and surfaces of NURBS models. The user can pick the surface/edge as well as the distribution of the load for each direction that can vary spatially.
  * Pressure boundary conditions: Load that can be applied normal to edges and surfaces of NURBS models. The user can pick the surface/edge as well as the distribution of the normal load magnitude that can vary spatially.
  
- **Paraview export:** Exports the following models in a Paraview file.
  * Nurbs 2D/3D
  * Nurbs shells
  * T-Splines 2D
  * T-Spline shells
