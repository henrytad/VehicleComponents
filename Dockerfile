FROM ubuntu:24.04 AS setup

ARG DYAD_CHANNEL=dyad-3.5.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl xvfb \
        libgl1 libglx-mesa0 libgl1-mesa-dri \
        libxrandr2 libxinerama1 libxcursor1 libxi6 \
    && rm -rf /var/lib/apt/lists/*

RUN curl -fsSL https://install.julialang.org | sh -s -- --yes

ENV PATH=/root/.juliaup/bin:$PATH \
    JULIAUP_SERVER=https://juliahub.com/juliabin \
    JULIA_PKG_SERVER=juliahub.com \
    LIBGL_ALWAYS_SOFTWARE=1 \
    GKSwstype=100 \
    JULIA_CPU_TARGET=generic \
    JULIA_AUTH=/root/.julia/servers/juliahub.com/auth.toml

RUN juliaup add "$DYAD_CHANNEL" && juliaup default "$DYAD_CHANNEL"

WORKDIR /workspace

COPY Project.toml Manifest.toml ./

RUN --mount=type=secret,id=juliahub_auth,required=true \
    install -Dm600 /run/secrets/juliahub_auth "$JULIA_AUTH" \
    && JULIA_PKG_PRECOMPILE_AUTO=0 julia --project=. -e 'using Pkg; Pkg.instantiate()' \
    && rm "$JULIA_AUTH"

RUN --mount=type=secret,id=juliahub_auth,required=true \
    install -Dm600 /run/secrets/juliahub_auth "$JULIA_AUTH" \
    && xvfb-run -a julia --check-bounds=yes --project=. \
        -e 'using Pkg, TOML; Pkg.precompile(collect(keys(TOML.parsefile("Project.toml")["deps"])))' \
    && rm "$JULIA_AUTH"

COPY . .
RUN --mount=type=secret,id=juliahub_auth,required=true \
    install -Dm600 /run/secrets/juliahub_auth "$JULIA_AUTH" \
    && xvfb-run -a julia --check-bounds=yes --project=. \
        -e 'using Pkg; Pkg.test(; test_args=["--precompile-only"])' \
    && rm "$JULIA_AUTH"

FROM setup AS tests

ARG TEST_FILE=""
RUN --mount=type=secret,id=juliahub_auth,required=true \
    install -Dm600 /run/secrets/juliahub_auth "$JULIA_AUTH" \
    && xvfb-run -a julia --check-bounds=yes --project=. \
        -e 'using Pkg; file = ENV["TEST_FILE"]; Pkg.test(; test_args=isempty(file) ? String[] : [file])' \
    && rm "$JULIA_AUTH"
