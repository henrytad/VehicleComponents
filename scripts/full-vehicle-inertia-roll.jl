using Revise
using VehicleComponents
using ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock
using ADTypes: AutoFiniteDiff
using Statistics: mean

h = 0.25
g = 9.80665
k_roll = 1600.0
initial_roll = deg2rad(2)
dt = 0.01
t_end = 12.0

@named model = FullVehicleTestInertiaRoll()
ssys = multibody(model)
prob = ODEProblem(ssys, [
    ssys.pivot_depth => h,
    ssys.roll_stiffness => k_roll,
    ssys.initial_roll => initial_roll,
], (0.0, t_end))
mass, cg = VehicleComponents.vehicle_mass_and_cg(prob, ssys.vehicle)
prob = remake(prob; p=[ssys.reference_cg => cg])
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); adaptive=false, dt=dt, save_everystep=true)

# Determine the period and, thus, the Ixx
roll = sol[model.roll_joint.phi]
roll_rate = sol[model.roll_joint.w]
crossings = findall(i -> roll[i] < 0 <= roll[i+1], 1:length(roll)-1)
period = mean(diff(crossings)) * dt
frequency = 1 / period
k_effective = k_roll - mass * g * h
k_effective > 0 || error("Increase k_roll above mass * g * h for a stable roll test")
Ixx = k_effective * (period / (2π))^2 - mass * h^2

println("T = $(round(period; digits=4)) s, f = $(round(frequency; digits=4)) Hz, Ixx = $(round(Ixx; digits=2)) kg·m²")
