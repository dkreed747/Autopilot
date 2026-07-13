# syntax=docker/dockerfile:1
#
# Autopilot application image
# ---------------------------------------------------------------------------
# Build stage:   the umaa-cyclone-cpp development container compiles the
#                autopilot + the pinned umaa-cpp submodule and installs the
#                runtime component to /opt/autopilot, then harvests the
#                shared-library closure from the dev prefixes into
#                /opt/autopilot/lib64.
# Runtime stage: ubi10-minimal + a handful of RPM libs. Runs the autopilot
#                and the mission-control web console (port 8080) as a
#                non-root user.
#
#   docker run -p 8080:8080 <image>                                # both
#   docker run <image> bin/autopilot autopilot.yaml                # app only
#   docker run -p 8080:8080 <image> bin/mission_console autopilot.yaml 8080 web
#
# Tests are NOT run here -- the CI test job runs the full suite in the dev
# image before this image is built. The build context must include the
# umaa-cpp submodule (git submodule update --init --recursive).
# ---------------------------------------------------------------------------

ARG BASE_IMAGE=gitlab.bongo-barley.ts.net/poseidon/utility/development-containers/umaa-cyclone-cpp:latest
ARG RUNTIME_IMAGE=registry.access.redhat.com/ubi10/ubi-minimal:latest

# ===========================================================================
# Stage: build the autopilot from the pinned submodule, install + harvest
# ===========================================================================
FROM ${BASE_IMAGE} AS build
USER root
WORKDIR /src/autopilot
COPY . .
RUN test -f umaa-cpp/CMakeLists.txt \
    || { echo "ERROR: umaa-cpp submodule missing from the build context." >&2; \
         echo "Run 'git submodule update --init --recursive' before building." >&2; \
         exit 1; }

# App-only release build (runtime component: binaries, SDK lib, web root,
# config -- no headers or cmake package files).
RUN cmake --preset ci -DAUTOPILOT_BUILD_TESTS=OFF \
    && cmake --build --preset ci \
    && cmake --install build --component runtime --prefix /opt/autopilot

# The binaries resolve autopilot.yaml, CYCLONE_QOS_PROFILES.xml, log4cxx.xml,
# and the web root relative to the working directory -- surface them at the
# prefix root (relative symlinks survive the COPY into the runtime stage).
COPY docker/run-all.sh /opt/autopilot/run-all.sh
RUN chmod 0755 /opt/autopilot/run-all.sh \
    && ln -s share/autopilot/web /opt/autopilot/web \
    && ln -s share/autopilot/autopilot.yaml /opt/autopilot/autopilot.yaml \
    && ln -s share/umaa-cpp/config/CYCLONE_QOS_PROFILES.xml /opt/autopilot/CYCLONE_QOS_PROFILES.xml \
    && ln -s share/umaa-cpp/config/log4cxx.xml /opt/autopilot/log4cxx.xml

# Harvest the shared-library closure from the dev prefixes. System libraries
# (glibc, libstdc++, apr, libuuid, openssl, ...) are NOT harvested -- the
# runtime stage installs them as RPMs.
RUN cp -a /opt/umaa/lib64/libumaa_cyclone_cpp*.so* /opt/autopilot/lib64/ \
    && cp -a /opt/cyclonedds/lib64/libddsc*.so* /opt/autopilot/lib64/ \
    && cp -a /usr/local/lib64/libyaml-cpp*.so* \
             /usr/local/lib64/liblog4cxx*.so* \
             /usr/local/lib64/libfmt*.so* \
             /opt/autopilot/lib64/ \
    && cp -a /usr/local/lib/libGeographicLib*.so* /opt/autopilot/lib64/

# Gate: every binary must fully resolve against the harvested closure plus
# system libs. Catches harvest-list drift at build time, not in production.
RUN set -e; \
    for b in autopilot mission_console mission_runner; do \
      if LD_LIBRARY_PATH=/opt/autopilot/lib64 ldd "/opt/autopilot/bin/${b}" | grep "not found"; then \
        echo "ERROR: unresolved shared libraries in ${b}" >&2; exit 1; \
      fi; \
    done

# ===========================================================================
# Runtime stage: minimal UBI + RPM system libs + the app prefix
# ===========================================================================
FROM ${RUNTIME_IMAGE} AS runtime

# apr/apr-util (log4cxx), libuuid, libstdc++; shadow-utils only to create the
# user, then removed.
RUN microdnf -y install shadow-utils libstdc++ libuuid apr apr-util \
    && useradd --uid 1000 --create-home autopilot \
    && microdnf -y remove shadow-utils \
    && microdnf clean all \
    && rm -rf /var/cache/dnf /var/cache/yum

COPY --from=build /opt/autopilot /opt/autopilot
RUN echo /opt/autopilot/lib64 > /etc/ld.so.conf.d/autopilot.conf && ldconfig \
    && chown -R autopilot:autopilot /opt/autopilot

USER autopilot
WORKDIR /opt/autopilot
EXPOSE 8080

# All-in-one default: autopilot + mission-control web console.
CMD ["/opt/autopilot/run-all.sh"]
