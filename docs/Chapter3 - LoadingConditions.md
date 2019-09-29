# Boundary conditions
This chapter will provide the existing boundary conditions in this code. 
## Loads
### Nodal load
The most common loading condition is applying a load directly the control points.
It is applied by choosing the model node, the degree of freedom and the magnitude of the load as shown below.

```csharp
model.Loads.Add(new Load()
{
	Amount = -1,
	Node = model.ControlPointsDictionary[10],
	DOF = StructuralDof.TranslationZ
});
```

### Neumann boundary condition
It acts a distributed load. A function is provided in the form of a spatatially varying delegate that describes the load condition. In case of 2D nurbs patches the Edges are automatically extracted and this loading condition can be applied by integration directly. The same is true for 3D patches where both edges and faces are automatically extracted.

```csharp
Value verticalDistributedLoad = (x, y, z) => new double[] {0, -100, 0};
model.PatchesDictionary[0].EdgesDictionary[1].LoadingConditions
	.Add(new NeumannBoundaryCondition(verticalDistributedLoad));
```

### Pressure boundary condition
It acts a load normal to a boundary entity. A double provided as input describes the load condition magnitude. In case of 2D nurbs patches the Edges are automatically extracted and this loading condition can be applied by integration directly. The same is true for 3D patches where both edges and faces are automatically extracted.

```csharp
model.PatchesDictionary[0].EdgesDictionary[0].LoadingConditions
	.Add(new PressureBoundaryCondition(30000));
```


## Constraints

### Nodal constraints
The simplest boundary constraint implemented in the code is a degree of freedom constraint. A multitude of Dofs can be selected and a constrained is applied to their equivalent Control Points. In addition to fixed Constraint this boundary conditions can handle fixed displacements by providing an movement etc. magnitude to the dof needed. 


```csharp
foreach (ControlPoint controlPoint in model.PatchesDictionary[0].EdgesDictionary[0].ControlPointsDictionary .Values)
{
	model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationX });
    model.ControlPointsDictionary[controlPoint.ID].Constrains.Add(new Constraint() { DOF = StructuralDof.TranslationY });
}
```
