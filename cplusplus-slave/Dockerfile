FROM osrf/ros:lunar-desktop-full
MAINTAINER Rody Middelkoop <rody.middelkoop@gmail.com>

# Add locales after locale-gen as needed
# Upgrade packages on image
# Preparations for sshd
RUN apt-get -q update && \
    DEBIAN_FRONTEND="noninteractive" apt-get -q upgrade -y -o Dpkg::Options::="--force-confnew" --no-install-recommends &&\
    DEBIAN_FRONTEND="noninteractive" apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends openssh-server locales gnupg2 dirmngr &&\
    /usr/sbin/locale-gen en_US.UTF-8 &&\
    apt-get -q autoremove &&\
    apt-get -q clean -y && rm -rf /var/lib/apt/lists/* && rm -f /var/cache/apt/*.bin &&\
    sed -i 's|session    required     pam_loginuid.so|session    optional     pam_loginuid.so|g' /etc/pam.d/sshd &&\
    mkdir -p /var/run/sshd

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

RUN \
  apt-get -q update && \
  echo "deb http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main" | tee /etc/apt/sources.list.d/webupd8team-java.list && \
  echo "deb-src http://ppa.launchpad.net/webupd8team/java/ubuntu xenial main" | tee -a /etc/apt/sources.list.d/webupd8team-java.list && \
  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys EEA14886 && \
  apt-get -q update && \
  echo oracle-java8-installer shared/accepted-oracle-license-v1-1 select true | debconf-set-selections && \
  apt-get update && \
  apt-get install -y oracle-java8-installer oracle-java8-set-default && \
  rm -rf /var/lib/apt/lists/* && \/bin/bash -c '{ cd /tmp; rm -rf cppcheck-build cppcheck-1.81; curl -L http://github.com/danmar/cppcheck/releases/download/1.81/cppcheck-1.81.tar.gz | tar xz; mkdir cppcheck-build; cd cppcheck-build; cmake ../cppcheck-1.81/ -DCMAKE_BUILD_TYPE=Release -DHAVE_RULES=OFF; make; make install; cd; rm -rf /tmp/cppcheck-build /tmp/cppcheck-1.81;}'; rm -rf /var/cache/oracle-jdk8-installer


# Define working directory.
WORKDIR /data

RUN wget -O - https://apt.llvm.org/llvm-snapshot.gpg.key | apt-key add -

RUN echo "deb http://apt.llvm.org/xenial/ llvm-toolchain-xenial main" >> /etc/apt/sources.list

RUN echo "deb http://ppa.launchpad.net/ubuntu-toolchain-r/test/ubuntu xenial main" >> /etc/apt/sources.list

RUN apt-key adv --keyserver pgp.mit.edu --recv-key BA9EF27F

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/bash jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN echo ". /opt/ros/lunar/setup.bash" >> ~jenkins/.bashrc

RUN apt-get update && apt-get install -y git apt-utils

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget sudo && wget https://sonarsource.bintray.com/Distribution/sonar-scanner-cli/sonar-scanner-2.8.zip --quiet && unzip sonar-scanner-2.8.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

# Get some ROS and development programs
RUN DEBIAN_FRONTEND="noninteractive" apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends \
python-rosinstall python-rosinstall-generator python-wstool g++-7 doxygen clang-6.0 clang-tidy-6.0 plantuml valgrind rsync lftp

RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-7 100 --slave /usr/bin/g++ g++ /usr/bin/g++-7
RUN /bin/bash -c '{ cd /tmp; rm -rf cppcheck-build cppcheck-1.81; curl -L http://github.com/danmar/cppcheck/releases/download/1.81/cppcheck-1.81.tar.gz | tar xz; mkdir cppcheck-build; cd cppcheck-build; cmake ../cppcheck-1.81/ -DCMAKE_BUILD_TYPE=Release -DHAVE_RULES=OFF; make; make install; cd; rm -rf /tmp/cppcheck-build /tmp/cppcheck-1.81;}'

# Install Boost libraries for OSM
RUN wget http://downloads.sourceforge.net/project/boost/boost/1.65.1/boost_1_65_1.tar.gz \
  && tar xfz boost_1_65_1.tar.gz \
  && rm boost_1_65_1.tar.gz \
  && cd boost_1_65_1 \
  && ./bootstrap.sh --prefix=/usr/local --with-libraries=program_options \
  && ./b2 install \
  && rm -rf boost_1_65_1

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
