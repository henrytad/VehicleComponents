# Random road surface to ISO 8608: Gd(n) = Gd(n0) (n / n0)^-2, n0 = 0.1 cycles/m.

using StableRNGs: StableRNG

const ROAD_N0 = 0.1

function road_waves(N::Integer, seed::Integer, n_min::Real, n_max::Real)
    edges = exp.(range(log(n_min), log(n_max), length=N + 1))
    lo, hi = edges[1:(end-1)], edges[2:end]
    k = sqrt.(lo .* hi)
    amplitude = sqrt.(π * ROAD_N0^2 .* (1 ./ lo .- 1 ./ hi))
    rng = StableRNG(seed)
    direction = 2π .* rand(rng, N)
    phase = 2π .* rand(rng, N)
    return (amplitude=amplitude, kx=k .* cos.(direction), ky=k .* sin.(direction), phase=phase)
end

road_wave_amplitude(N, seed, n_min, n_max) = road_waves(N, seed, n_min, n_max).amplitude
road_wave_kx(N, seed, n_min, n_max) = road_waves(N, seed, n_min, n_max).kx
road_wave_ky(N, seed, n_min, n_max) = road_waves(N, seed, n_min, n_max).ky
road_wave_phase(N, seed, n_min, n_max) = road_waves(N, seed, n_min, n_max).phase

# Road height at `(x, y)`
road_height(amplitude, kx, ky, phase, roughness, x, y) = sqrt(roughness) *
    sum(amplitude[i] * cos(2π * (kx[i] * x + ky[i] * y) + phase[i]) for i in eachindex(amplitude))
road_height(waves, roughness, x, y) = road_height(waves.amplitude, waves.kx, waves.ky, waves.phase, roughness, x, y)
