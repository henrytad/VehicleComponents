using Revise
using VehicleComponents
using ADTypes: AutoFiniteDiff
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

@named model = VehicleComponents.FullVehicleTestStatic()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 2.0))
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()))

# ===========================================================================
# Animation
# ===========================================================================
render(model, sol; filename="output/full_vehicle_static.gif", up=[0, 0, 1], x=1.5, y=0.4, z=0.7, lookat=[0, 0.05, 0.3])
