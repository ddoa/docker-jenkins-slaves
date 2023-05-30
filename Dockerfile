# base image: Ubuntu Focal (20.04)
FROM ubuntu:jammy

LABEL maintainer="ESD team"

# the user that will be used in the container
# for using variables/arguments in a dockerfile: https://docs.docker.com/engine/reference/builder/#arg
# Running this dockerfile with USERNAME=aUserName will will create a user with that name and corresponding home directoy 
ARG USERNAME=jenkins

# First add the user so that one can put anything in the home directory
# If any mounts are shared between the host and the container one look at the numeric UID's
# so that the numeric UID of both users are the same 
RUN useradd --create-home --shell /bin/bash $USERNAME \
    && yes password | passwd $USERNAME

# give user sudo access
RUN usermod -aG sudo $USERNAME

# needed for non-interactive install of packages
ARG DEBIAN_FRONTEND=noninteractive
RUN apt-get update && apt-get install -q -y apt-utils 2>&1 | grep -v "delaying package configuration, since apt-utils is not installed"

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && \
    apt-get install --no-install-recommends -q -y tzdata && \
    rm -rf /var/lib/apt/lists/*

# setup locale, ROS requires UTF-8
RUN apt-get update && \
    apt-get install -q -y --no-install-recommends locales && \
    locale-gen en_US en_US.UTF-8 && \
    update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8 && \
    rm -rf /var/lib/apt/lists/*
ENV LANG en_US.UTF-8

# install packages needed for installation
RUN apt-get update && \
    apt-get install -q -y  \
    dirmngr \
    gnupg2 \
    wget && \
    rm -rf /var/lib/apt/lists/*

# ROS: setup keys. APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE is added to mute a warning:
# https://stackoverflow.com/questions/48162574/how-to-circumvent-apt-key-output-should-not-be-parsed
RUN APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# ROS: setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu jammy main" > /etc/apt/sources.list.d/ros2-latest.list

# install ROS & related tools
RUN apt-get update && apt-get install -q -y  \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-humble-desktop \
    && rm -rf /var/lib/apt/lists/*	

# install Gazebo. APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE is added to mute a warning:
# https://stackoverflow.com/questions/48162574/how-to-circumvent-apt-key-output-should-not-be-parsed
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
    wget https://packages.osrfoundation.org/gazebo.key -O - | APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 apt-key add - &&\
    apt-get update && apt-get install -q -y	\ 
    ros-humble-ros-gz	\
    && rm -rf /var/lib/apt/lists/*

#RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
#    wget https://packages.osrfoundation.org/gazebo.key -O - | APT_KEY_DONT_WARN_ON_DANGEROUS_USAGE=1 apt-key add - &&\
#    apt-get update && \
#    apt-get install -q -y gazebo libgazebo-dev \
#    && rm -rf /var/lib/apt/lists/*
    
    
# install Gazebo ROS2 Humble packages
RUN apt-get update && \
    apt-get install -q -y ros-humble-gazebo-ros-pkgs \
    && rm -rf /var/lib/apt/lists/*

# install tools and utilities
RUN apt-get update && apt-get install -q -y --no-install-recommends \	
    tmux git mesa-utils locate mc nano \
    && rm -rf /var/lib/apt/lists/*

# packages for CLion remote development
# https://blog.jetbrains.com/clion/2020/01/using-docker-with-clion/
RUN apt-get update \
  && apt-get install -q -y --no-install-recommends \
    ssh build-essential gcc g++ gdb clang cmake rsync tar python3 \
  && rm -rf /var/lib/apt/lists/*

# set up SSH server for Clion
RUN ( \
    echo 'LogLevel DEBUG2'; \
    echo 'PermitRootLogin yes'; \
    echo 'PasswordAuthentication yes'; \
    echo 'Subsystem sftp /usr/lib/openssh/sftp-server'; \
  ) > /etc/ssh/sshd_config_test_clion \
  && mkdir /run/sshd

# convenience aliasses
COPY ./bash_aliases /home/$USERNAME/.bash_aliases

# enable autocompletion of commands
RUN apt-get update && apt-get install -q -y --no-install-recommends \	
    bash-completion \
    && rm /etc/apt/apt.conf.d/docker-clean \
    && rm -rf /var/lib/apt/lists/*

# install tools and utilities
RUN apt-get update && apt-get install -q -y --no-install-recommends \	
    libgz-cmake3-dev	\
    && rm -rf /var/lib/apt/lists/*

# update the database so we can find stuff with locate. Keep this command below all the install commands.
RUN updatedb

# define working directory
WORKDIR /data/

RUN \
  apt-get -q update && \
  apt-get install curl -y && \
  cd /usr/lib/jvm &&  curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
  tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
  rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz

RUN update-alternatives  --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives  --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac

# Install Sonar Scanner so the image can run a local SQ analysis
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.7.0.2747-linux.zip --quiet && unzip sonar-scanner-cli-4.7.0.2747-linux.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-cli-4.7.0.2747-linux/conf

# setup entrypoint
ENV ROS_DISTRO humble
RUN echo 'source "/opt/ros/$ROS_DISTRO/setup.bash"' >> /home/$USERNAME/.bashrc
