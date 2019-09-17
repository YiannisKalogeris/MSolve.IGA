namespace MGroup.IGA.Entities.Loads
{
	/// <summary>
	/// Pressure loading boundary condition. Works as a load normal to the boundary entity applied on.
	/// </summary>
	public class PressureBoundaryCondition : LoadingCondition
	{
		/// <summary>
		/// Describes the distribution of the load. See <see cref="Loads.Value"/>
		/// </summary>
		public double Value { get; private set; }

		/// <summary>
		/// Defines a Pressure BoundaryCondition
		/// </summary>
		/// <param name="neumannValue"></param>
		public PressureBoundaryCondition(double pressureValue)
		{
			this.Value = pressureValue;
		}
	}
}