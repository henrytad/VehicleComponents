using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const DEG = 180 / π
const MM = 1000

@named model = VehicleComponents.SuspensionTestRig()
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

front = data.suspension.front
geom, setup = front.geometry, front.setup
ib = geom.inboard
cl, cr = geom.linkages.left, geom.linkages.right
al, ar = setup.alignment.left, setup.alignment.right

parameter_map = Dict([
    # Rig
    ssys.wheel_center_left => cl.wheel_center,
    ssys.wheel_center_right => cr.wheel_center,
    ssys.pushrod_outer_left => cl.pushrod_outer,
    ssys.pushrod_outer_right => cr.pushrod_outer,

    # Left corner
    ssys.suspension.linkage_left.lca_front => cl.lca_front,
    ssys.suspension.linkage_left.lca_rear => cl.lca_rear,
    ssys.suspension.linkage_left.lca_outer => cl.lca_outer,
    ssys.suspension.linkage_left.uca_front => cl.uca_front,
    ssys.suspension.linkage_left.uca_rear => cl.uca_rear,
    ssys.suspension.linkage_left.uca_outer => cl.uca_outer,
    ssys.suspension.linkage_left.tierod_inner => cl.tierod_inner,
    ssys.suspension.linkage_left.tierod_outer => cl.tierod_outer,
    ssys.suspension.linkage_left.static_camber => al.static_camber,
    ssys.suspension.linkage_left.static_toe => al.static_toe,

    # Right corner
    ssys.suspension.linkage_right.lca_front => cr.lca_front,
    ssys.suspension.linkage_right.lca_rear => cr.lca_rear,
    ssys.suspension.linkage_right.lca_outer => cr.lca_outer,
    ssys.suspension.linkage_right.uca_front => cr.uca_front,
    ssys.suspension.linkage_right.uca_rear => cr.uca_rear,
    ssys.suspension.linkage_right.uca_outer => cr.uca_outer,
    ssys.suspension.linkage_right.tierod_inner => cr.tierod_inner,
    ssys.suspension.linkage_right.tierod_outer => cr.tierod_outer,
    ssys.suspension.linkage_right.static_camber => ar.static_camber,
    ssys.suspension.linkage_right.static_toe => ar.static_toe,

    # Inboard
    ssys.suspension.inboard.rocker_pivot_left => ib.rocker_pivot.left,
    ssys.suspension.inboard.rocker_pivot_right => ib.rocker_pivot.right,
    ssys.suspension.inboard.pushrod_inner_left => ib.pushrod_inner.left,
    ssys.suspension.inboard.pushrod_inner_right => ib.pushrod_inner.right,
    ssys.suspension.inboard.heave_pickup_left => ib.heave_pickup.left,
    ssys.suspension.inboard.heave_pickup_right => ib.heave_pickup.right,
    ssys.suspension.inboard.roll_pickup_left => ib.roll_pickup.left,
    ssys.suspension.inboard.roll_pickup_right => ib.roll_pickup.right,
    ssys.suspension.inboard.heave_stiffness => setup.heave.stiffness,
    ssys.suspension.inboard.heave_damping => setup.heave.damping,
    ssys.suspension.inboard.heave_preload => setup.heave.preload,
    ssys.suspension.inboard.roll_stiffness => setup.roll.stiffness,
    ssys.suspension.inboard.roll_damping => setup.roll.damping,
    ssys.suspension.inboard.roll_preload => setup.roll.preload,
    ssys.suspension.inboard.pushrod_adjust_left => setup.pushrod_adjust.left,
    ssys.suspension.inboard.pushrod_adjust_right => setup.pushrod_adjust.right,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 2.0))
sol = solve(prob)

render(model, sol; filename="output/suspension.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
