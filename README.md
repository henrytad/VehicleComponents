# VehicleComponents

## Getting Started

This library was created with the Dyad Studio VS Code extension.  Your Dyad
models should be placed in the `dyad` directory and the files should be
given the `.dyad` extension.  Several such files have already been placed
in there to get you started.  The Dyad compiler will compile the Dyad models
into Julia code and place it in the `generated` folder.  Do not edit the
files in that directory or remove/rename that directory.

A complete tutorial on using Dyad Studio can be found [here](#).  But you
can run the provided example models by doing the following:

1. Run `Julia: Start REPL` from the command palette.

2. Type `]`.  This will take you to the package manager prompt.

3. At the `pkg>` prompt, type `instantiate` (this downloads all the Julia libraries
   you will need, and the very first time you do it it might take a while).

4. From the same `pkg>` prompt, type `test`.  This will test to make sure the models
   are working as expected.  It may also take some time but you should eventually
   see a result that indicates 2 of 2 tests passed.

5. Use the `Backspace`/`Delete` key to return to the normal Julia REPL, it should
   look like this: `julia>`.

6. Type `using VehicleComponents`.  This will load your model library.

7. Type `World()` to run a simulation of the `Hello` model.  The first time you run it,
   this might take a few seconds, but each successive time you run it, it should be very fast.

8. To see simulation results type `using Plots` (and answer `y` if asked if you want
   to add it as a dependency).

9. To plot results of the `World` simulation, simply type `plot(World())`.

10. You can plot variations on that simulation using keyword arguments.  For example,
   try `plot(World(stop=20, h=1.4))`.  The supported model parameters are
   `T_inf` and `h`; run `WorldSpec()` in the REPL to inspect all available
   simulation and model keyword arguments.

## Modeling Conventions

Everything in this library, such as hardpoints, frames, outputs, and the JSON vehicle data in `assets/`, follows the conventions below.

### Coordinate system: ISO 8855

All geometry uses the **ISO 8855:2011** vehicle axis system:

| Axis | Index | Quantity | Positive direction |
| ----- | ----- | ----- | ----- |
| X | `[1]` | Longitudinal | **Forward** |
| Y | `[2]` | Lateral | **Left** |
| Z | `[3]` | Vertical | **Up** |

"Left" and "right" are always as seen by a driver seated in the vehicle facing forward.

#### NOTE: MultibodyComponents defaults to Y-up

`MultibodyComponents` inherits Modelica's convention where **Y** points up: `World` defaults to a gravity direction of `n = [0, -1, 0]`. Every `World` in this library must therefore be instantiated explicitly for ISO:

```
world = MultibodyComponents.World(n = [0, 0, -1])
```

and renders must pass `up = [0, 0, 1]`.

### Vehicle Origin

- `x = 0` at the front axle centreline, so rear-axle hardpoints are therefore **negative** x.
- `y = 0` on the longitudinal plane of symmetry.
- `z = 0` at the ground plane, so wheel-centre z is approximately the tyre's unloaded radius.

### Sign conventions for wheel outputs

ISO distinguishes the **camber angle** (wheel lean measured relative to the vehicle body) from the **inclination angle** (the same lean measured relative to the road).

Camber and toe are reported **side-symmetrically**, so a left and a right corner can be plotted on the same axes and compared directly:

| Output | Positive means |
| ----- | ----- |
| `camber` | Top of the wheel leans **outboard** |
| `toe` | Leading edge of the wheel points **inboard** (toe-in) |

### Naming conventions

#### Corner identifiers

Corners are named **axle first, then side**, lowercase, as a trailing suffix:

| Suffix | Corner      |
| ------ | ----------- |
| `_fl`  | Front left  |
| `_fr`  | Front right |
| `_rl`  | Rear left   |
| `_rr`  | Rear right  |

Rules:

1. **Axle-then-side, always.** Write `_fl`, never `_lf`.
1. **The suffix goes last**, after every other qualifier: `camber_fl`, `wheel_center_rr`, `spring_stiffness_fl`.
1. **Only use it where corners coexist in one scope** such as a full-vehicle or axle-level assembly. Inside a single-corner component, do not bake the corner into names.
1. **Abbreviate only as a fixed-width code, never as a standalone qualifier.** In `_fl` the first character is always the axle and the second always the side, so position tells the reader what each one means. A lone qualifier has no such anchor: `lca_r` gives no way to tell "rear" from "right". Spell those out — `_front` / `_rear` for the axle, `_left` / `_right` for the side.

#### Hardpoints

Hardpoints are named `<link>_<location>`, matching Adams/Car practice so geometry stays portable in both directions:

| Name | Meaning |
| ----- | ----- |
| `lca_front`, `lca_rear` | Lower control arm inboard ball joints |
| `lca_outer` | Lower control arm outboard ball joint |
| `uca_front`, `uca_rear` | Upper control arm inboard ball joints |
| `uca_outer` | Upper control arm outboard ball joint |
| `tierod_inner`, `tierod_outer`| Tie rod ends |
| `pushrod_outer` | Outboard pushrod pickup |
| `wheel_center` | Wheel centre |

#### Units and style parameters

Model interfaces are **SI, unprefixed** throughout: metres, radians, seconds, kilograms, newtons.

Shared visual styling lives in [Styles.dyad](dyad/Styles.dyad) and is prefixed `sty_`, so
appearance parameters are visually distinct from physical ones.

## Testing

### Testing in Julia

1. Run `Julia: Start REPL` from the command palette if it did not automatically start
1. Type `]`.  This will take you to the package manager prompt.
1. At the `pkg>` prompt, type `instantiate` (this downloads all the Julia libraries you will need, and the very first time you do it it might take a while).
1. From the same `pkg>` prompt, type `test`. This will test to make sure the models are working as expected. It may also take some time but you should eventually see a result that indicates X of Y tests passed.
1. Use the `Backspace`/`Delete` key to return to the normal Julia REPL, it should look like this: `julia>`.

### Testing in Docker

Runs the test suite in the same Linux environment CI uses. The image build instantiates the project and runs its tests against the committed Julia files in `generated/`.

1. Install [Docker Desktop](https://www.docker.com/products/docker-desktop/) and start it.

1. Build the image to run the tests:

   ```powershell
   docker buildx build `
      --progress=plain `
      --output type=cacheonly `
      --secret id=juliahub_auth,src="$HOME\.julia\servers\juliahub.com\auth.toml" `
      .
   ```

The build uses your JuliaHub credentials to download Dyad packages; the secret is available only while dependencies are installed and is not retained in the image. The command exits non-zero if dependency installation or a test fails. It does not compile `.dyad` sources, so regenerate `generated/` in Dyad Studio and commit it whenever a model changes.

GitHub Actions passes the base64-encoded `JULIAHUB_TOKEN_ENCODED` repository secret directly to the Docker build.
