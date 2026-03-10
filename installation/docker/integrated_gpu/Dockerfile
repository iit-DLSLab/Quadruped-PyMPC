FROM ubuntu:22.04

# -----------------------------
# Basic configigurations
# -----------------------------
ENV DEBIAN_FRONTEND=noninteractive \
    CONDA_DIR=/opt/conda \
    PATH=/opt/conda/bin:$PATH

SHELL ["/bin/bash", "-lc"]

# -----------------------------
# System dependencies
# -----------------------------
RUN apt-get update && apt-get install -y --no-install-recommends \
      ca-certificates \
      wget \
      bzip2 \
      git \
      build-essential \
      cmake \
      pkg-config \
      python3-pip \
      python3-tk \
      gedit \
      libgl1-mesa-glx \
      libgl1-mesa-dri \
      mesa-utils \
      libglib2.0-0 \
      libxext6 \
      libsm6 \
      libxrender1 \
      byobu \
      vim \
    && rm -rf /var/lib/apt/lists/*

# -----------------------------
# Miniforge (conda)
# -----------------------------
RUN wget -q https://github.com/conda-forge/miniforge/releases/latest/download/Miniforge3-Linux-x86_64.sh -O /tmp/miniforge.sh \
    && bash /tmp/miniforge.sh -b -p "$CONDA_DIR" \
    && rm -f /tmp/miniforge.sh \
    && conda clean -afy

# -----------------------------
# Copy local repo for build
# -----------------------------
WORKDIR /workspace
COPY . /workspace

# -----------------------------
# Conda env (ROS2 Humble)
# -----------------------------
ARG CONDA_ENV_DIR=installation/mamba/integrated_gpu
ARG CONDA_ENV_FILE=mamba_environment_ros2_humble.yml
ARG CONDA_ENV_NAME=quadruped_pympc_ros2_env

RUN test -f "$CONDA_ENV_DIR/$CONDA_ENV_FILE" \
    || (echo "ERROR: Env file not found: /workspace/$CONDA_ENV_DIR/$CONDA_ENV_FILE" \
        && exit 1) \
    && conda env create -f "$CONDA_ENV_DIR/$CONDA_ENV_FILE" \
    && conda clean -afy

# ROS2 Humble toolchain expectations
RUN conda run -n "$CONDA_ENV_NAME" bash -lc "\
      set -euo pipefail && \
      conda install -n $CONDA_ENV_NAME -y -c conda-forge 'python=3.10' 'cmake<4' numpy && \
      python -m pip uninstall -y cmake || true && \
      conda clean -afy \
    "

# ---------------------------------------
# Build acados + install template
# ---------------------------------------
ENV ACADOS_SOURCE_DIR=/workspace/quadruped_pympc/acados
ENV LD_LIBRARY_PATH=/workspace/quadruped_pympc/acados/lib:$LD_LIBRARY_PATH

RUN conda run -n "$CONDA_ENV_NAME" bash -lc "\
      set -euo pipefail && \
      cd /workspace/quadruped_pympc/acados && \
      mkdir -p build && \
      cd build && \
      cmake -DACADOS_WITH_SYSTEM_BLASFEO:BOOL=OFF -DCMAKE_POLICY_VERSION_MINIMUM=3.9 .. && \
      make install -j\$(nproc) && \
      python -m pip install --no-cache-dir -e ../interfaces/acados_template \
    "

# -----------------------------
# Install Quadruped-PyMPC
# -----------------------------
RUN conda run -n "$CONDA_ENV_NAME" bash -lc "\
      set -euo pipefail && \
      cd /workspace && \
      python -m pip install --no-cache-dir -e . \
    "

# -----------------------------
# Build ROS2 messages workspace
# -----------------------------
RUN conda run -n "$CONDA_ENV_NAME" bash -lc "\
      set -euo pipefail && \
      cd /workspace/ros2/msgs_ws && \
      export Python_EXECUTABLE=\$CONDA_PREFIX/bin/python3 && \
      export Python_ROOT_DIR=\$CONDA_PREFIX && \
      colcon build --cmake-args \
        -DPython_EXECUTABLE=\$CONDA_PREFIX/bin/python3 \
        -DPython_ROOT_DIR=\$CONDA_PREFIX \
    "

# -----------------------------
# Conda environment ready in every new terminal
# -----------------------------
RUN cat >> /root/.bashrc <<EOF
source /opt/conda/etc/profile.d/conda.sh
conda activate $CONDA_ENV_NAME
if [ -f /workspace/ros2/msgs_ws/install/setup.bash ]; then
  source /workspace/ros2/msgs_ws/install/setup.bash
fi
export PS1="\[\e]0;\u@\h: \w\a\]\[\033[01;37m\]\u\[\033[00m\]@\[\033[01;32m\]\h\[\033[00m\]:\[\033[01;34m\]\w\[\033[00m\] "
EOF

# -----------------------------
# Interactive shell
# -----------------------------
CMD ["bash"]
