FROM nvidia/cuda:12.8.1-base-ubuntu24.04

SHELL ["/bin/bash", "-c"]

# Install ROS2.
ARG ROS_APT_SOURCE_VERSION=1.1.0

RUN apt-get update && apt-get install -y --no-install-recommends curl ca-certificates && \
    curl -L -o /tmp/ros2-apt-source.deb \
      "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${ROS_APT_SOURCE_VERSION}/ros2-apt-source_${ROS_APT_SOURCE_VERSION}.$(. /etc/os-release && echo ${UBUNTU_CODENAME:-${VERSION_CODENAME}})_all.deb" && \
    dpkg -i /tmp/ros2-apt-source.deb && \
    rm /tmp/ros2-apt-source.deb && \
    apt-get update && \
    apt-get install -y --no-install-recommends ros-jazzy-ros-base && \
    rm -rf /var/lib/apt/lists/*

# Install other dependencies.
RUN apt-get update && apt-get install -y \
    python3-pip \
    python3-tk \
    python3-colcon-common-extensions \
    ros-jazzy-rmw-cyclonedds-cpp \
    libxxhash-dev \
    build-essential \
    dos2unix \
    wget \
    && rm -rf /var/lib/apt/lists/*

# CUDA env. NOTE: we deliberately do NOT set LD_LIBRARY_PATH=/usr/local/cuda/lib64.
# torch is installed from pip wheels that bundle their own CUDA runtime + NVRTC,
# and pointing the loader at the base image's CUDA tree can shadow those bundled
# libs and cause nvrtc/builtins load failures at runtime.
ENV PATH=/usr/local/cuda/bin:${PATH}

ENV PIP_BREAK_SYSTEM_PACKAGES=1

# Python dependencies
WORKDIR /app

RUN pip3 install --no-cache-dir --ignore-installed --upgrade pip setuptools wheel

# Install the PyTorch stack from the CUDA 12.8 wheel index.
# The +cu128 local-version pins prevent accidental cu126/cu130 resolution.
ARG PYTORCH_INDEX_URL=https://download.pytorch.org/whl/cu128

RUN pip3 install --no-cache-dir --timeout 900 --retries 3 \
    torch==2.10.0+cu128 \
    torchvision==0.25.0+cu128 \
    torchaudio==2.10.0+cu128 \
    --index-url ${PYTORCH_INDEX_URL}

# Gate immediately so a wrong wheel fails the build here, not at runtime.
RUN python3 - <<'PY'
import torch
print("torch:", torch.__version__)
print("torch CUDA:", torch.version.cuda)
assert torch.__version__ == "2.10.0+cu128", torch.__version__
assert torch.version.cuda == "12.8", torch.version.cuda
PY

# Constraints pin the exact +cu128 build so no later step can re-resolve torch.
RUN cat >/tmp/torch-constraints.txt <<'EOF'
torch==2.10.0+cu128
torchvision==0.25.0+cu128
torchaudio==2.10.0+cu128
EOF

# Install numpy fresh, ignoring the Debian-managed numpy (apt's numpy has no pip
# RECORD file, so pip can't upgrade it normally). Scoped to numpy ONLY so we do
# NOT use --ignore-installed for the torch-dependent packages below, which would
# make pip re-resolve torch and pull a mismatched (e.g. cu130) wheel.
RUN pip3 install --no-cache-dir --ignore-installed numpy==2.2.6

# Science stack. No --ignore-installed here. The constraint keeps torch pinned,
# and --extra-index-url keeps the pinned +cu128 wheel satisfiable if any
# dependency references torch.
RUN pip3 install --no-cache-dir \
    --constraint /tmp/torch-constraints.txt \
    --extra-index-url ${PYTORCH_INDEX_URL} \
    matplotlib==3.10.8 \
    wandb==0.25.0 \
    pyyaml==6.0.1 \
    omegaconf==2.3.0 \
    tqdm==4.67.3 \
    diffusers==0.36.0 \
    accelerate==1.12.0 \
    lovely-tensors==0.1.21 \
    numpy==2.2.6 \
    scipy==1.17.1 \
    ema-pytorch==0.7.9 \
    scikit-learn==1.8.0 \
    pandas==3.0.1 \
    statsmodels==0.14.6 \
    torchinfo==1.8.0 \
    joblib==1.5.3 \
    einops==0.8.2 \
    spectrum==0.9.0

RUN pip3 install --no-cache-dir \
    --constraint /tmp/torch-constraints.txt \
    --extra-index-url ${PYTORCH_INDEX_URL} \
    mne==1.12.0 \
    mne-realtime==0.3.0 \
    mne_icalabel==0.8.1

# --no-deps is required: mneflow pins mne<=1.7 (would downgrade our mne==1.12.0),
# and py_neuromodulation forces scipy to backtrack to a version with no prebuilt
# wheel, causing a source build that fails without a Fortran compiler.
# All runtime requirements are satisfied by previously installed packages.
RUN pip3 install --no-cache-dir --no-deps mneflow==0.5.10 py_neuromodulation==0.1.7

# Final gate after ALL Python installs. At build time there is no GPU, so the
# complex-backward test is guarded behind cuda.is_available() and only runs at
# runtime if invoked; the version asserts always run.
RUN python3 - <<'PY'
import glob
import torch

print("FINAL torch:", torch.__version__)
print("FINAL torch CUDA:", torch.version.cuda)
print("CUDA available:", torch.cuda.is_available())

assert torch.__version__ == "2.10.0+cu128", torch.__version__
assert torch.version.cuda == "12.8", torch.version.cuda

print("NVRTC libs:")
for p in glob.glob("/usr/local/lib/python3.12/dist-packages/nvidia/**/libnvrtc*", recursive=True):
    print(" ", p)

if torch.cuda.is_available():
    x = torch.randn(32, device="cuda", dtype=torch.complex64, requires_grad=True)
    y = (-x).real.sum()
    y.backward()
    print("complex CUDA backward OK")
PY

# XXX: Install Git - not necessarily a long-term dependency, but it's convenient
#      for some users currently.
RUN apt-get update && apt-get install -y git

# Install modern pybind11 (system pybind11-dev is 2.11.1, too old for NumPy 2 ABI)
RUN apt-get update && apt-get remove -y pybind11-dev && rm -rf /var/lib/apt/lists/*
RUN pip3 install --no-cache-dir pybind11==3.0.4

# Copy ROS workspace and build packages

WORKDIR /app

# ROS interfaces
COPY interfaces/neurosimo_pipeline_interfaces interfaces/neurosimo_pipeline_interfaces/
COPY interfaces/neurosimo_project_interfaces interfaces/neurosimo_project_interfaces/
COPY interfaces/neurosimo_eeg_interfaces interfaces/neurosimo_eeg_interfaces/
COPY interfaces/neurosimo_system_interfaces interfaces/neurosimo_system_interfaces/
COPY interfaces/shared_stimulation_interfaces interfaces/shared_stimulation_interfaces/
COPY src/realtime src/realtime/
COPY src/filesystem_utils src/filesystem_utils/

RUN source /opt/ros/jazzy/setup.bash && colcon build

# ROS node
COPY src/common src/common/
COPY src/decider src/decider/
RUN source /opt/ros/jazzy/setup.bash && source install/setup.bash && colcon build --symlink-install --packages-select decider

# Copy CUDA test script
COPY src/decider/test_cuda.py /app

# Set the Cyclone DDS configuration
COPY config/cyclonedds.xml /config/
ENV CYCLONEDDS_URI=file:///config/cyclonedds.xml

# Copy entrypoint

WORKDIR /app

COPY src/decider/ros_entrypoint.sh .
RUN dos2unix ros_entrypoint.sh
RUN chmod +x ros_entrypoint.sh

ENTRYPOINT ["./ros_entrypoint.sh"]
