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
  echo "deb http://ppa.launchpad.net/linuxuprising/java/ubuntu bionic main" | tee /etc/apt/sources.list.d/linuxuprising-java.list && \
  apt-key adv --keyserver hkp://keyserver.ubuntu.com:80 --recv-keys 73C3DB2A && \
  apt-get -q update && \
  echo oracle-java13-installer shared/accepted-oracle-license-v1-2 select true | /usr/bin/debconf-set-selections && \
  apt-get update && \
  apt-get install -y oracle-java13-installer oracle-java13-set-default && \
  rm -rf /var/lib/apt/lists/* && \
  rm -rf /var/cache/oracle-jdk13-installer

# Define working directory.
WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN apt-get update && apt-get install -y git

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-2.8.zip --quiet && unzip sonar-scanner-2.8.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

RUN apt-get -y install libgd3
RUN apt-get -q update && apt-get -y install php7.3 \
  php-pear php7.3-cgi php7.3-cli php7.3-common php7.3-fpm \
  php7.3-gd php7.3-json php7.3-mysql php7.3-readline php7.3-xml \
  default-mysql-client php7.3-sqlite3

RUN apt-get remove libcurl4 && apt-get -y install curl

RUN apt-get -q update && \
  apt-get -y install php7.3-curl php7.3-mbstring bzip2

RUN php -r "copy('https://getcomposer.org/installer', 'composer-setup.php');" && php composer-setup.php

RUN ln -s /data/composer.phar /usr/local/bin/composer

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
