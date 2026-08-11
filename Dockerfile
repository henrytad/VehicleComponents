FROM ubuntu:24.04

# Must match [dyad] kernel in Project.toml.
ARG DYAD_CHANNEL=dyad-3.3.0

RUN apt-get update && apt-get install -y --no-install-recommends \
        ca-certificates curl \
    && rm -rf /var/lib/apt/lists/*
RUN curl -fsSL https://install.julialang.org | sh -s -- --yes

ENV PATH=/root/.juliaup/bin:$PATH \
    JULIAUP_SERVER=https://juliahub.com/juliabin \
    JULIA_PKG_SERVER=juliahub.com

RUN juliaup add "$DYAD_CHANNEL" \
    && juliaup default "$DYAD_CHANNEL"

WORKDIR /workspace

COPY Project.toml Manifest.toml ./
RUN julia --project=. -e "using Pkg; Pkg.instantiate()"

COPY . .
RUN julia --project=. -e "using Pkg; Pkg.test()"
