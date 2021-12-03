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
  apt-get install curl -y && \
  mkdir /usr/lib/jvm && cd /usr/lib/jvm &&  curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
  tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
  rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz

# Define working directory.
WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN apt-get update && apt-get install -y git

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt


COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

RUN apt-get -y install libgd3
RUN apt-get -q update && apt-get -y install php7.4 \
  php-pear php7.4-cgi php7.4-cli php7.4-common php7.4-fpm \
  php7.4-gd php7.4-json php7.4-mysql php7.4-readline php7.4-xml \
  default-mysql-client php7.4-sqlite3

RUN apt-get remove -y libcurl4 && apt-get -y install curl

RUN apt-get -q update && \
  apt-get -y install php7.4-curl php7.4-mbstring bzip2 libmcrypt-dev php7.4-dev

RUN php -r "copy('https://getcomposer.org/installer', 'composer-setup.php');" && php composer-setup.php

RUN pecl channel-update pecl.php.net && pecl install mcrypt-1.0.4

RUN bash -c "echo extension=/usr/lib/php/20180731/mcrypt.so > /etc/php/7.4/cli/conf.d/mcrypt.ini"

RUN ln -s /data/composer.phar /usr/local/bin/composer

# Add docker-client to be able to build, run etc. docker containers
RUN apt-get install -y docker

# Make Java15 the default
RUN update-alternatives  --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives  --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
