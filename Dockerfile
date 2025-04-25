# Use an official Ubuntu as a base image
FROM ubuntu:22.04


ARG USER_NAME=cmaybe
ARG GROUP_NAME=cau
ARG PROJECT_NAME=optimal-parking

# Set non-interactive mode to avoid prompts during installation
ENV DEBIAN_FRONTEND=noninteractive

# Setup timezone
RUN echo 'Etc/UTC' > /etc/timezone \
    && ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime \
    && apt-get update \
    && apt-get -y -q --no-install-recommends install \
    tzdata \
    && apt-get clean

# Install packages
RUN apt-get update \
    && apt-get install -y -q --no-install-recommends \
    apt-utils \
    bash-completion \
    clang-format \
    dirmngr \
    gdb \
    sshpass \
    sudo \
    vim \
    wget \
    tmux \
    && apt-get clean

# Update and install necessary dependencies
RUN apt-get update && apt-get install -y \
    build-essential \
    dirmngr \
    cmake \
    git \
    gnupg2 \
    python3-numpy \
    python3-matplotlib \
    python3 \
    python3-dev \
    python3-pip \
    && apt-get clean

# Dependency packages
RUN apt-get -q update \
    && apt-get -y -q --no-install-recommends install \
    libboost-all-dev \
    libpython3-dev \
    libyaml-cpp-dev \
    && apt-get clean

# Install CMake 3.27.4
RUN wget https://github.com/Kitware/CMake/releases/download/v3.27.4/cmake-3.27.4-linux-x86_64.tar.gz && \
    tar -xzvf cmake-3.27.4-linux-x86_64.tar.gz && \
    cd cmake-3.27.4-linux-x86_64 && \
    rm -rf /usr/local/man && \
    cp -r * /usr/local/ && \
    cd .. && \
    rm -rf cmake-3.27.4-linux-x86_64 cmake-3.27.4-linux-x86_64.tar.gz


# Install Eigen 3.4.0	
RUN git clone --branch 3.4.0 https://gitlab.com/libeigen/eigen.git /opt/eigen \
    && mkdir -p /opt/eigen/build && cd /opt/eigen/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DEIGEN_BUILD_DOC=OFF \
    -DBUILD_TESTING=OFF \
    .. \
    && make install \
    && cd /opt && rm -r /opt/eigen

# Install Google Test v1.16.0
RUN git clone --branch v1.16.0 https://github.com/google/googletest.git /opt/googletest \
    && mkdir -p /opt/googletest/build && cd /opt/googletest/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DBUILD_GMOCK=OFF \
    .. \
    && make install \
    && cd /opt && rm -r /opt/googletest

# Install Abseil 2.20250127.1
RUN git clone --branch 20250127.1 https://github.com/abseil/abseil-cpp.git /opt/abseil-cpp \
    && mkdir -p /opt/abseil-cpp/build && cd /opt/abseil-cpp/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POSITION_INDEPENDENT_CODE=ON \
    .. \
    && make install \
    && cd /opt && rm -r /opt/abseil-cpp


# Install OSQP v1.0.0
RUN git clone --branch v1.0.0 --recursive https://github.com/osqp/osqp.git /opt/osqp \
    &&  mkdir -p /opt/osqp/build && cd /opt/osqp/build \
    && cmake -DOSQP_ENABLE_64BIT_INT=ON .. \
    &&  make install \
    && cd /opt && rm -r /opt/osqp

# osqp-cpp
RUN git clone https://github.com/google/osqp-cpp.git  /opt/osqp-cpp \
    && mkdir -p /opt/osqp-cpp/build && cd /opt/osqp-cpp/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    .. \
    && make install \
    && cd /opt && rm -r /opt/osqp-cpp

# Install matplotlib-cpp
RUN git clone https://github.com/lava/matplotlib-cpp.git  --recursive /opt/matplotlib-cpp \
    && mkdir -p /opt/matplotlib-cpp/build && cd /opt/matplotlib-cpp/build \
    && cmake \
    -DCMAKE_BUILD_TYPE=Release \
    .. \
    && make install \
    && cd /opt && rm -r /opt/matplotlib-cpp

ENV LANG=C.UTF-8
ENV LC_ALL=C.UTF-8

# Add user info
ARG USER_UID=1000
ARG USER_GID=1000
RUN groupadd --gid ${USER_GID} ${GROUP_NAME} 

RUN useradd --create-home --shell /bin/bash \
    --uid ${USER_UID} --gid ${USER_GID} ${USER_NAME} \
    # Possible security risk
    && echo "${USER_NAME}:${GROUP_NAME}" | sudo chpasswd \
    && echo "${USER_NAME} ALL=(ALL) NOPASSWD:ALL" > "/etc/sudoers.d/${USER_NAME}"

# Make workspace 
RUN mkdir -p /home/${USER_NAME}/${PROJECT_NAME} \
    && chown -R ${USER_NAME}:${GROUP_NAME} /home/${USER_NAME}
ENV HOME=/home/${USER_NAME}
ENV LD_LIBRARY_PATH=/opt/blasfeo/lib:/opt/hpipm/lib

RUN echo "export USER=${USER_NAME}" >> ${HOME}/.bashrc \
    && echo "export GROUP=${GROUP_NAME}" >> ${HOME}/.bashrc 
# Shell
USER ${USER_NAME}
WORKDIR /home/${USER_NAME}/${PROJECT_NAME}
ENV SHELL="/bin/bash"