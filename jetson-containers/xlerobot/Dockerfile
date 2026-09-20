#---
# name: xlerobot
# group: robots
# docs: docs.md
# depends: [lerobot, gstreamer]
# buildkit_device: nvidia.com/gpu=all
# requires: '>=36'
# test: []
#---
ARG BASE_IMAGE
FROM ${BASE_IMAGE}

WORKDIR /opt/lerobot

ARG XLEROBOT_REPO=tfoldi/XLeRobot \
    XLEROBOT_BRANCH=main \
    XLEROBOT_COMMIT=""

ADD xlerobot.left.json /data/models/huggingface/lerobot/calibration/robots/so_follower/xlreobot.left.json

ADD my_xlerobot_2wheels_lab.json /data/models/huggingface/lerobot/calibration/robots/xlerobot_2wheels/my_xlerobot_2wheels_lab.json

# Clone XLeRobot and install its robots (xlerobot, xlerobot_2wheels,
# xlerobot_mecanum) and shared model code as real lerobot plugins (see
# software/plugins/) - lerobot auto-discovers them by pip distribution name
# at CLI startup, so nothing gets copied into lerobot's own tree. The
# example scripts stay in the clone at /opt/xlerobot/software/examples/ and
# run directly from there. xlerobot_vr isn't converted to a plugin yet.
#
# XLEROBOT_COMMIT is only used here to bust the Docker layer cache: a plain
# `git clone --branch=X --depth=1` is cached by the literal branch name, not
# its current tip, so pushing new commits to the same branch silently keeps
# serving a stale clone unless something in this RUN's text actually changes.
# Pass --build-args "XLEROBOT_COMMIT:$(git rev-parse origin/branch)" to force
# a fresh clone.
RUN echo "XLEROBOT_COMMIT=${XLEROBOT_COMMIT}" && \
    git clone --branch=${XLEROBOT_BRANCH} --depth=1 https://github.com/${XLEROBOT_REPO} /opt/xlerobot && \
    pip install /opt/xlerobot/software/plugins/xlerobot_model \
                /opt/xlerobot/software/plugins/lerobot_robot_xlerobot \
                /opt/xlerobot/software/plugins/lerobot_robot_xlerobot_2wheels \
                /opt/xlerobot/software/plugins/lerobot_robot_xlerobot_mecanum && \
    cp /data/models/huggingface/lerobot/calibration/robots/so_follower/xlreobot.left.json \
    /data/models/huggingface/lerobot/calibration/robots/so_follower/None.json &&\
    pip install deepdiff pynput draccus rerun-sdk pygame


# Start JupyterLab server in the background by default
CMD true
