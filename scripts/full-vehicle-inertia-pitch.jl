using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots
using ADTypes: AutoFiniteDiff
using Statistics: mean

car = VehicleComponents.params
d = 0.5
g = 9.80665
initial_pitch = deg2rad(2)
dt = 0.01
t_end = 12.0

mass, cg = VehicleComponents.Data.vehicle_mass_and_cg(car)

@named model = VehicleComponents.FullVehicleTestInertiaPitch()
ssys = multibody(model)
prob = ODEProblem(ssys, [
    ssys.reference_cg => cg,
    ssys.pivot_height => d,
    ssys.initial_pitch => initial_pitch,
], (0.0, t_end))
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); adaptive=false, dt=dt, save_everystep=true)

# Determine the period and, thus, the Iyy
pitch = sol[model.pitch_joint.phi]
pitch_rate = sol[model.pitch_joint.w]
crossings = findall(i -> pitch[i] < 0 <= pitch[i+1], 1:length(pitch)-1)
period = mean(diff(crossings)) * dt
frequency = 1 / period
Iyy = mass * g * d * (period / (2π))^2 - mass * d^2

println("$(car.name): T = $(round(period; digits=4)) s, f = $(round(frequency; digits=4)) Hz, Iyy = $(round(Iyy; digits=2)) kg·m²")
