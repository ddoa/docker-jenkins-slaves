ARG FROM_IMAGE=ubuntu:focal
FROM $FROM_IMAGE

# setup timezone
RUN echo 'Etc/UTC' > /etc/timezone && \
    ln -s /usr/share/zoneinfo/Etc/UTC /etc/localtime && \
    apt-get update && apt-get install -q -y tzdata && rm -rf /var/lib/apt/lists/*

# install some packages
RUN apt-get update && apt-get install -q -y \
    bash-completion \
    dirmngr \
    gnupg2 \
    lsb-release \
    python \
    python3-pip \
    && rm -rf /var/lib/apt/lists/*

# setup keys
RUN apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys C1CF6E31E6BADE8868B172B4F42ED6FBAB17C654

# setup sources.list
RUN echo "deb http://packages.ros.org/ros2/ubuntu `lsb_release -sc` main" > /etc/apt/sources.list.d/ros2-latest.list 

# install bootstrap tools
RUN apt-get update && apt-get install --no-install-recommends -y \
    git \
    python3-colcon-common-extensions \
    python3-colcon-mixin \
    python3-rosdep \
    python3-vcstool \
    ros-foxy-ros-base \
    && rm -rf /var/lib/apt/lists/*

# install python packages
RUN pip3 install -U \
    argcomplete \
    flake8 \
    flake8-blind-except \
    flake8-builtins \
    flake8-class-newline \
    flake8-comprehensions \
    flake8-deprecated \
    flake8-docstrings \
    flake8-import-order \
    flake8-quotes \
    pytest-repeat \
    pytest-rerunfailures
# This is a workaround for pytest not found causing builds to fail
# Following RUN statements tests for regression of https://github.com/ros2/ros2/issues/722
RUN pip3 freeze | grep pytest \
    && python3 -m pytest --version

# bootstrap rosdep
#RUN rosdep init \
#    && rosdep update

# setup colcon mixin and metadata
# RUN colcon mixin add default \
#      https://raw.githubusercontent.com/colcon/colcon-mixin-repository/master/index.yaml && \
#    colcon mixin update && \
#    colcon metadata add default \
#      https://raw.githubusercontent.com/colcon/colcon-metadata-repository/master/index.yaml && \
#    colcon metadata update

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN \
  apt-get -q update && \
  apt-get install -y wget software-properties-common && \
  add-apt-repository ppa:linuxuprising/java && \
  apt-get -q update && \
  echo oracle-java15-installer shared/accepted-oracle-license-v1-2 select true | /usr/bin/debconf-set-selections && \
  apt-get install -y oracle-java15-installer oracle-java15-set-default && \
  rm -rf /var/lib/apt/lists/* && \/bin/bash -c '{ cd /tmp; rm -rf cppcheck-build cppcheck-2.2; curl -L https://github.com/danmar/cppcheck/archive/2.2.tar.gz | tar xz; mkdir cppcheck-build; cd cppcheck-build; cmake ../cppcheck-2.2/ -DCMAKE_BUILD_TYPE=Release -DHAVE_RULES=OFF; make; make install; cd; rm -rf /tmp/cppcheck-build /tmp/cppcheck-2.2;}' && \
  rm -rf /var/cache/oracle-jdk13-installer

# Define working directory.
WORKDIR /data

# Install repositories and keys
RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add -

RUN echo "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial-5.0 main" >> /etc/apt/sources.list

RUN echo "deb http://ppa.launchpad.net/ubuntu-toolchain-r/test/ubuntu xenial main" >> /etc/apt/sources.list

RUN /bin/echo -e -----BEGIN PGP PUBLIC KEY BLOCK-----\\n\\n\
mI0ESuBvRwEEAMi4cDba7xlKaaoXjO1n1HX8RKrkW+HEIl79nSOSJyvzysajs7zU\\n\
ow/OzCQp9NswqrDmNuH1+lPTTRNAGtK8r2ouq2rnXT1mTl23dpgHZ9spseR73s4Z\\n\
BGw/ag4bpU5dNUStvfmHhIjVCuiSpNn7cyy1JSSvSs3N2mxteKjXLBf7ABEBAAG0\\n\
GkxhdW5jaHBhZCBUb29sY2hhaW4gYnVpbGRziLYEEwECACAFAkrgb0cCGwMGCwkI\\n\
BwMCBBUCCAMEFgIDAQIeAQIXgAAKCRAek3eiup7yfzGKA/4xzUqNACSlB+k+DxFF\\n\
HqkwKa/ziFiAlkLQyyhm+iqz80htRZr7Ls/ZRYZl0aSU56/hLe0V+TviJ1s8qdN2\\n\
lamkKdXIAFfavA04nOnTzyIBJ82EAUT3Nh45skMxo4z4iZMNmsyaQpNl/m/lNtOL\\n\
hR64v5ZybofB2EWkMxUzX8D/FQ==\\n\
=LcUQ\\n\
-----END PGP PUBLIC KEY BLOCK-----\\n | apt-key add -

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/bash jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN echo "source /opt/ros/foxy/setup.bash" >> ~jenkins/.bashrc

RUN apt-get update && apt-get install -y git apt-utils libsdl2-dev qtbase5-dev

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 curl && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216-linux.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216-linux.zip -d /opt && rm sonar-scanner-cli-4.5.0.2216-linux.zip

COPY "sonar-scanner.properties" /opt/sonar-scanner-4.5.0.2216-linux/conf

RUN DEBIAN_FRONTEND="noninteractive" apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends \
 g++-9 doxygen plantuml valgrind rsync lftp lcov
 
 # Install clang tools
RUN curl -SL https://github.com/llvm/llvm-project/releases/download/llvmorg-11.0.0/clang%2bllvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04.tar.xz | tar -xJC . && \
 mv clang+llvm-11.0.0-x86_64-linux-gnu-ubuntu-20.04 clang_11.0.0 && \
 mv clang_11.0.0/bin/clang /usr/local/bin && \
 mv clang_11.0.0/bin/clang-tidy /usr/local/bin && \
 mv clang_11.0.0/bin/clang-format /usr/local/bin && \
 rm -rf clang_11.0.0

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-9 100 --slave /usr/bin/g++ g++ /usr/bin/g++-9

# Install CGAL
RUN apt-get update && apt-get install -y libcgal-dev libcgal-qt5-dev

# Install SDL & SDL_image
RUN apt-get update && apt-get install -y libsdl1.2-dev libsdl-image1.2-dev

# Install googletest framework
RUN apt-get update && apt-get install -y libgtest-dev

# setup bashrc
RUN cp /etc/skel/.bashrc ~/
RUN /usr/sbin/locale-gen en_US.UTF-8 &&\
    apt-get -q update && \
    DEBIAN_FRONTEND="noninteractive" apt-get -q upgrade -y -o Dpkg::Options::="--force-confnew" --no-install-recommends &&\
    DEBIAN_FRONTEND="noninteractive" apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends openssh-server locales gnupg2 dirmngr &&\
    apt-get -q autoremove &&\
    apt-get -q clean -y && rm -rf /var/lib/apt/lists/* && rm -f /var/cache/apt/*.bin &&\
    sed -i 's|session    required     pam_loginuid.so|session    optional     pam_loginuid.so|g' /etc/pam.d/sshd &&\
    mkdir -p /var/run/sshd

# Install Gazebo
RUN sh -c 'echo "deb http://packages.osrfoundation.org/gazebo/ubuntu-stable `lsb_release -cs` main" > /etc/apt/sources.list.d/gazebo-stable.list' &&\
 wget https://packages.osrfoundation.org/gazebo.key -O - | apt-key add - &&\
 apt-get update &&\
 apt-get install -y gazebo11 libgazebo11-dev

# Install Gazebo ROS2 Foxy packages
RUN apt-get install -y ros-foxy-gazebo-ros-pkgs

# Install Lanelet2 for ROS2
RUN apt install -y ros-foxy-lanelet2

# Install pip2 (can't be installed from apt because it has been removed since 20.04)
RUN curl https://bootstrap.pypa.io/get-pip.py -o get-pip.py &&\
 python get-pip.py

# Install mapnik and other dependencies for the Gazebo OSM plugin
RUN pip install numpy osmapi &&\
 apt install -y python3-mapnik python-lxml

RUN cd /tmp; rm -rf cppcheck-build cppcheck-2.2; curl -L https://github.com/danmar/cppcheck/archive/2.2.tar.gz | tar xz; mkdir cppcheck-build; cd cppcheck-build; cmake ../cppcheck-2.2/ -DCMAKE_BUILD_TYPE=Release -DHAVE_RULES=OFF; make; make install; cd; rm -rf /tmp/cppcheck-build /tmp/cppcheck-2.2;

RUN apt install -y mc findutils locate

ADD ros-gazebo-demo /data/ros-gazebo-demo

RUN echo "source /opt/ros/foxy/setup.bash" >> ~/.bashrc

RUN echo "source /opt/ros/foxy/setup.bash" >> /home/jenkins/.bashrc

RUN cat /opt/ros/foxy/lib/x86_64-linux-gnu/urdfdom/cmake/urdfdom-config.cmake | sed 's|set(urdfdom_INCLUDE_DIRS.*)|set(urdfdom_INCLUDE_DIRS "${CMAKE_CURRENT_LIST_DIR}/../../include" )|' | sudo tee  /opt/ros/foxy/lib/x86_64-linux-gnu/urdfdom/cmake/urdfdom-config.cmake

RUN updatedb

RUN chown -R jenkins /data/ros-gazebo-demo

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]

