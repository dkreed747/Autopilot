# syntax=docker/dockerfile:1
#
# Autopilot application images (one Dockerfile, three targets):
#   --target autopilot        the autonomy stack (no console, no runner)
#   --target mission-console  the mission-control web console (port 8080)
#   --target mission-runner   the headless one-shot mission driver
#
# A shared build stage compiles everything once; per-target staging stages
# assemble each image's prefix from CMake install components. docker-compose.yml
# stands up the stack; DDS between containers uses unicast peers (no multicast).
#
# Tests are NOT run here -- the CI test job runs the full suite in the dev
# image before these images are built. The build context must include the
# umaa-cpp submodule (git submodule update --init --recursive).
#
# TODO(repo-split): per-tool prefixes and subset configs once the tools move
# to their own repos; /opt/autopilot is kept in all three images this release.

ARG BASE_IMAGE=gitlab.bongo-barley.ts.net/poseidon/utility/development-containers/umaa-cyclone-cpp:latest
ARG RUNTIME_IMAGE=registry.access.redhat.com/ubi10/ubi-minimal:latest

FROM ${BASE_IMAGE} AS build
ARG BUILD_PRESET=amd64-release
USER root
WORKDIR /src/autopilot
COPY . .
RUN test -f umaa-cpp/CMakeLists.txt \
    || { echo "ERROR: umaa-cpp submodule missing from the build context." >&2; \
         echo "Run 'git submodule update --init --recursive' before building." >&2; \
         exit 1; }
RUN cmake --preset ${BUILD_PRESET} -DAUTOPILOT_BUILD_TESTS=OFF \
    && cmake --build --preset ${BUILD_PRESET}

# Shared prefix: core lib + SDK lib + QoS/log4cxx/loopback configs + yaml,
# the harvested third-party .so closure, and the CWD-relative symlinks.
FROM build AS stage-common
RUN cmake --install build --component runtime --prefix /staging \
    && cp -a /opt/umaa/lib64/libumaa_cyclone_cpp*.so* /staging/lib64/ \
    && cp -a /opt/cyclonedds/lib64/libddsc*.so* /staging/lib64/ \
    && cp -a /usr/local/lib64/libyaml-cpp*.so* \
             /usr/local/lib64/liblog4cxx*.so* \
             /usr/local/lib64/libfmt*.so* \
             /staging/lib64/ \
    && cp -a /usr/local/lib/libGeographicLib*.so* /staging/lib64/ \
    && ln -s share/autopilot/autopilot.yaml /staging/autopilot.yaml \
    && ln -s share/umaa-cpp/config/CYCLONE_QOS_PROFILES.xml /staging/CYCLONE_QOS_PROFILES.xml \
    && ln -s share/umaa-cpp/config/log4cxx.xml /staging/log4cxx.xml

FROM stage-common AS stage-autopilot
RUN cmake --install build --component runtime-autopilot --prefix /staging \
    && { LD_LIBRARY_PATH=/staging/lib64 ldd /staging/bin/autopilot | grep "not found" \
         && { echo "ERROR: unresolved shared libraries in autopilot" >&2; exit 1; } || true; }

FROM stage-common AS stage-console
COPY docker/entrypoint-console.sh /staging/entrypoint-console.sh
RUN cmake --install build --component runtime-tools --prefix /staging \
    && cmake --install build --component runtime-console --prefix /staging \
    && chmod 0755 /staging/entrypoint-console.sh \
    && ln -s share/autopilot/web /staging/web \
    && { LD_LIBRARY_PATH=/staging/lib64 ldd /staging/bin/mission_console | grep "not found" \
         && { echo "ERROR: unresolved shared libraries in mission_console" >&2; exit 1; } || true; }

FROM stage-common AS stage-runner
RUN cmake --install build --component runtime-tools --prefix /staging \
    && cmake --install build --component runtime-runner --prefix /staging \
    && { LD_LIBRARY_PATH=/staging/lib64 ldd /staging/bin/mission_runner | grep "not found" \
         && { echo "ERROR: unresolved shared libraries in mission_runner" >&2; exit 1; } || true; }

# Minimal UBI runtime shared by all three images. RPMs are the system halves
# of the shipped binaries' NEEDED closure: apr/apr-util + openldap <- log4cxx,
# openssl-libs <- CycloneDDS, libuuid <- libumaa-cpp, libstdc++ <- everything.
FROM ${RUNTIME_IMAGE} AS runtime-base
RUN microdnf -y install shadow-utils libstdc++ libuuid apr apr-util openldap openssl-libs \
    && useradd --uid 1000 --create-home autopilot \
    && microdnf -y remove shadow-utils \
    && microdnf clean all \
    && rm -rf /var/cache/dnf /var/cache/yum
RUN echo /opt/autopilot/lib64 > /etc/ld.so.conf.d/autopilot.conf
WORKDIR /opt/autopilot

# Loader gate per image: with a nonexistent config every binary exits 1 after
# its libraries load; a missing shared library exits 127 before main() runs.
FROM runtime-base AS autopilot
COPY --from=stage-autopilot --chown=autopilot:autopilot /staging /opt/autopilot
RUN ldconfig \
    && rc=0; timeout 20 bin/autopilot /nonexistent.yaml >/dev/null 2>&1 || rc=$?; \
    [ "${rc}" -eq 1 ] || { echo "ERROR: autopilot failed to load (rc=${rc})" >&2; exit 1; }
USER autopilot
CMD ["/opt/autopilot/bin/autopilot", "autopilot.yaml"]

FROM runtime-base AS mission-console
COPY --from=stage-console --chown=autopilot:autopilot /staging /opt/autopilot
RUN ldconfig \
    && rc=0; timeout 20 bin/mission_console /nonexistent.yaml >/dev/null 2>&1 || rc=$?; \
    [ "${rc}" -eq 1 ] || { echo "ERROR: mission_console failed to load (rc=${rc})" >&2; exit 1; }
USER autopilot
EXPOSE 8080
CMD ["/opt/autopilot/entrypoint-console.sh"]

FROM runtime-base AS mission-runner
COPY --from=stage-runner --chown=autopilot:autopilot /staging /opt/autopilot
RUN ldconfig \
    && mkdir -p mission-out && chown autopilot:autopilot mission-out \
    && rc=0; timeout 20 bin/mission_runner /nonexistent.yaml >/dev/null 2>&1 || rc=$?; \
    [ "${rc}" -eq 1 ] || { echo "ERROR: mission_runner failed to load (rc=${rc})" >&2; exit 1; }
USER autopilot
VOLUME /opt/autopilot/mission-out
CMD ["/opt/autopilot/bin/mission_runner", "autopilot.yaml", "mission-out"]
