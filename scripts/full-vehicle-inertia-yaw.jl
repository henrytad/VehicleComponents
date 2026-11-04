using Revise
using VehicleComponents
using GLMakie, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots
using ADTypes: AutoFiniteDiff
using Statistics: mean

car = VehicleComponents.params
pivot = car.body.cg
k_yaw = 200.0
initial_yaw = deg2rad(5)
t_settle = 5.0
dt = 0.01
t_end = 25.0

@named model = VehicleComponents.FullVehicleTestInertiaYaw()
ssys = multibody(model)
D = Differential(ModelingToolkit.get_iv(ssys))
# TODO: Remove guesses once MTK avoids symbolic guess expansion for linear initialization:
# https://github.com/SciML/ModelingToolkit.jl/issues/5272
prob = ODEProblem(ssys, [
    ssys.yaw_stiffness => k_yaw,
    ssys.initial_yaw => initial_yaw,
], (0.0, t_end); guesses=[D(D(ssys.vehicle.body.r_0[i])) => 0.0 for i in 1:3])
sol = solve(prob, Rodas5P(autodiff=AutoFiniteDiff()); adaptive=false, dt=dt, save_everystep=true)

# Determine the period and, thus, the Izz
yaw = sol[model.yaw_joint.phi]
yaw_rate = sol[model.yaw_joint.w]
crossings = findall(i -> sol.t[i] >= t_settle && yaw[i] < 0 <= yaw[i+1], 1:length(yaw)-1)
period = mean(diff(crossings)) * dt
Izz = k_yaw * (period / (2π))^2
frequency = 1 / period

println("$(car.name): T = $(round(period; digits=4)) s, f = $(round(frequency; digits=4)) Hz, Izz = $(round(Izz; digits=2)) kg·m²")

# ===========================================================================
# Animation
# ===========================================================================
render(model, sol; filename="output/full_vehicle_yaw_inertia.gif", lookat=pivot, x=pivot[1]-2.0, y=pivot[2]-2.0, z=pivot[3]+2.8, up=[0, 0, 1], show_axis=true)
