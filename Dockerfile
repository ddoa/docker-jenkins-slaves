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

RUN \
  apt-get -q update && \
  apt-get install wget curl -y && \
  mkdir /usr/lib/jvm && cd /usr/lib/jvm &&  curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
  tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
  rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz

# Define working directory.
WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN apt-get update && apt-get install -y git
RUN apt-get install -y unzip && wget http://www-eu.apache.org/dist/maven/maven-3/3.5.4/binaries/apache-maven-3.5.4-bin.zip && cd /opt; mkdir /opt/apache-maven-3.5.0 ; cd /opt/apache-maven-3.5.0; unzip /data/apache-maven-3.5.4-bin.zip; mv /opt/apache-maven-3.5.0/apache-maven-3.5.4/* . && cd /data && wget http://apache.cs.uu.nl/maven/maven-3/3.6.3/binaries/apache-maven-3.6.3-bin.zip && cd /opt ; unzip /data/apache-maven-3.6.3-bin.zip

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 curl && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

# Install Scala
ENV SCALA_VERSION 2.13.3
ENV SBT_VERSION 1.5.5
ENV SBT_URL http://dl.bintray.com/sbt/debian/sbt-$SBT_VERSION.deb
ENV SCALA_URL http://downloads.typesafe.com/scala/$SCALA_VERSION/scala-$SCALA_VERSION.tgz

RUN \
  curl -fsL $SCALA_URL | tar xfz - -C /root/ && \
  echo 'export PATH=~/scala-$SCALA_VERSION/bin:$PATH' >> /root/.bashrc

# Install sbt
RUN apt-get update && \
  apt-get install apt-transport-https curl gnupg -yqq && \
  echo "deb https://repo.scala-sbt.org/scalasbt/debian all main" | tee /etc/apt/sources.list.d/sbt.list && \
  echo "deb https://repo.scala-sbt.org/scalasbt/debian /" | tee /etc/apt/sources.list.d/sbt_old.list && \
  curl -sL "https://keyserver.ubuntu.com/pks/lookup?op=get&search=0x2EE0EA64E40A89B84B2DF73499E82A75642AC823" | gpg --no-default-keyring --keyring gnupg-ring:/etc/apt/trusted.gpg.d/scalasbt-release.gpg --import && \
  chmod 644 /etc/apt/trusted.gpg.d/scalasbt-release.gpg && \
  apt-get update && \
  apt-get install sbt

# Add docker-client to be able to build, run etc. docker containers
RUN apt-get install -y docker

# Make Java15 the default
RUN update-alternatives  --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives  --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac


# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
