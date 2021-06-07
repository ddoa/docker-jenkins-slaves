# This Dockerfile is used to build an image containing basic stuff to be used as a Jenkins slave build node.
FROM debian:latest
MAINTAINER Rody Middelkoop <rody.middelkoop@gmail.com>

# Add locales after locale-gen as needed
# Upgrade packages on image
# Preparations for sshd
run apt-get -q update &&\
    apt-get install -y locales gnupg2 && \
    DEBIAN_FRONTEND="noninteractive" apt-get -q upgrade -y -o Dpkg::Options::="--force-confnew" --no-install-recommends &&\
    DEBIAN_FRONTEND="noninteractive" apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends openssh-server &&\
    /usr/sbin/locale-gen en_US.UTF-8 &&\
    apt-get -q autoremove &&\
    apt-get -q clean -y && rm -rf /var/lib/apt/lists/* && rm -f /var/cache/apt/*.bin &&\
    sed -i 's|session    required     pam_loginuid.so|session    optional     pam_loginuid.so|g' /etc/pam.d/sshd &&\
    mkdir -p /var/run/sshd

ENV LANG en_US.UTF-8
ENV LANGUAGE en_US:en
ENV LC_ALL en_US.UTF-8

# JDK15
RUN apt-get update && apt-get install -y wget
RUN mkdir /usr/lib/jvm && wget https://download.java.net/java/GA/jdk15.0.2/0d1cfde4252546c6931946de8db48ee2/7/GPL/openjdk-15.0.2_linux-x64_bin.tar.gz && tar xvzf openjdk-15.0.2_linux-x64_bin.tar.gz -C /usr/lib/jvm

# Install OpenJDK8
RUN \
    cd /opt && wget https://github.com/AdoptOpenJDK/openjdk8-binaries/releases/download/jdk8u232-b09/OpenJDK8U-jdk_x64_linux_hotspot_8u232b09.tar.gz && \
    cd /usr/lib/jvm && tar xvzf /opt/OpenJDK8U-jdk_x64_linux_hotspot_8u232b09.tar.gz

ENV JAVA_HOME /usr/lib/jvm/jdk8u232-b09/
ENV PATH ${PATH}:{JAVA_HOME}/bin


# Define working directory.
WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN apt-get update && apt-get install -y git

#Atlassian Specific
RUN apt-get -q update && \
  apt-get install -y apt-transport-https && \
  sh -c 'echo "deb https://packages.atlassian.com/debian/atlassian-sdk-deb/ stable contrib" >>/etc/apt/sources.list' && \
  wget https://packages.atlassian.com/api/gpg/key/public && \
  apt-key add public && \
  apt-get -q update && \
  apt-get install -y atlassian-plugin-sdk

# Add this property to Jenkins ENV_INJECT plugin properties because .bashrc and other properties don't get loaded or are loaded for the root user instead of the jenkins user
ENV ATLAS_HOME /usr/share/atlassian-plugin-sdk-8.2.8

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt


COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

# Add docker-client to be able to build, run etc. docker containers
RUN apt-get install -y docker libnss3-dev libatk-bridge2.0-0 libxss1 libasound2 libgtk-3-0 libx11-6 libx11-xcb1 libdrm-dev libgbm-dev 


# Set JDK8 as default
RUN update-alternatives --install /usr/bin/java java /usr/lib/jvm/jdk8u232-b09/bin/java 1
RUN update-alternatives --install /usr/bin/javac javac /usr/lib/jvm/jdk8u232-b09/bin/javac 1

RUN apt-get install -y curl software-properties-common && curl -sL https://deb.nodesource.com/setup_16.x | bash -
RUN apt-get install -y nodejs

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
