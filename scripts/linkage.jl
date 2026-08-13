using GLMakie, JSON3, ModelingToolkit, MultibodyComponents, OrdinaryDiffEqRosenbrock, Plots

# Run `using VehicleComponents` if not already in env. If you try importing it again you'll
# get an "importing VehicleComponents into Main conflicts with an existing global" error.
# If you restart the REPL and rerun the lines, you will not get the error.
using VehicleComponents

const DEG = 180 / π
const MM = 1000

@named model = VehicleComponents.LinkageTestRig(is_left=true)
ssys = multibody(model)

data_path = joinpath(pwd(), "assets", "vehicles", "Test.json")
data = JSON3.read(read(data_path, String))

front = data.suspension.front
geom, setup = front.geometry, front.setup
linkage, align = geom.linkages.left, setup.alignment.left

parameter_map = Dict([
    # Test rig
    ssys.wheel_center => linkage.wheel_center,

    # Geometry
    ssys.linkage.lca_front => linkage.lca_front,
    ssys.linkage.lca_rear => linkage.lca_rear,
    ssys.linkage.lca_outer => linkage.lca_outer,
    ssys.linkage.uca_front => linkage.uca_front,
    ssys.linkage.uca_rear => linkage.uca_rear,
    ssys.linkage.uca_outer => linkage.uca_outer,
    ssys.linkage.tierod_inner => linkage.tierod_inner,
    ssys.linkage.tierod_outer => linkage.tierod_outer,
    ssys.linkage.pushrod_outer => linkage.pushrod_outer,

    # Upright
    ssys.linkage.upright_mass => linkage.upright.mass,
    ssys.linkage.upright_I_11 => linkage.upright.i_11,
    ssys.linkage.upright_I_22 => linkage.upright.i_22,
    ssys.linkage.upright_I_33 => linkage.upright.i_33,

    # Alignment
    ssys.linkage.static_camber => align.static_camber,
    ssys.linkage.static_toe => align.static_toe,
])

prob = ODEProblem(ssys, parameter_map, (0.0, 2.0))
sol = solve(prob)

plot_camber = Plots.plot(
    sol,
    idxs=(MM * model.linkage.wc_height, DEG * model.linkage.camber),
    ylabel="camber [deg]"
);
plot_toe = Plots.plot(
    sol,
    idxs=(MM * model.linkage.wc_height, DEG * model.linkage.toe),
    ylabel="toe [deg]",
    xlabel="wheel travel [mm]"
);
Plots.plot(
    plot_camber, plot_toe;
    layout=(2, 1),
    link=:x,
    legend=false,
    lw=2
)

render(model, sol; filename="output/linkage.gif", up=[0, 0, 1], x=1, y=0.1, z=1, lookat=[0, 0, 0.3])
