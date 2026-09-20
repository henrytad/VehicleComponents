using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

@named model = VehicleComponents.SuspensionTestRig()
ssys = multibody(model)
prob = ODEProblem(ssys, [], (0.0, 2.0))
sol = solve(prob)

render(model, sol; filename="output/suspension.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
