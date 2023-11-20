# This Dockerfile is used to build an image containing basic stuff to be used as a Jenkins slave build node.
FROM debian:latest
MAINTAINER Rody Middelkoop <rody.middelkoop@gmail.com>

# Add locales after locale-gen as needed
# Upgrade packages on image
# Preparations for sshd

run apt-get -q update &&\
    apt-get install -y locales && \
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

# Define working directory.
WORKDIR /data

RUN apt-get update && apt-get install -y git curl xz-utils man gpg

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/bash jenkins && echo "jenkins:jenkins" | chpasswd

# Node specific
RUN groupadd --gid 1001 node \
  && useradd --uid 1001 --gid node --shell /bin/bash --create-home node

RUN \
	apt-get update && \
	apt-get install -y ca-certificates curl gnupg && \
	mkdir -p /etc/apt/keyrings && \
	curl -fsSL https://deb.nodesource.com/gpgkey/nodesource-repo.gpg.key | gpg --dearmor -o /etc/apt/keyrings/nodesource.gpg

RUN \
	NODE_MAJOR=18;echo "deb [signed-by=/etc/apt/keyrings/nodesource.gpg] https://deb.nodesource.com/node_$NODE_MAJOR.x nodistro main" | tee /etc/apt/sources.list.d/nodesource.list && \
	apt-get update && \
	apt-get install nodejs -y


RUN mkdir "/home/jenkins/.npm-packages"
RUN chown jenkins /home/jenkins/.npm-packages

RUN \
  apt-get -q update && \
  apt-get install curl -y && \
  mkdir /usr/lib/jvm && cd /usr/lib/jvm &&  curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
  tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
  rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz

# GCC/G++
RUN DEBIAN_FRONTEND="noninteractive" apt-get -q update && apt-get -q install -y -o Dpkg::Options::="--force-confnew"  --no-install-recommends gcc g++ build-essential
RUN update-alternatives --install /usr/bin/gcc gcc /usr/bin/gcc-12 100 --slave /usr/bin/g++ g++ /usr/bin/g++-12

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-4.5.0.2216/conf

# Install Angular CLI

RUN npm install -g @angular/cli && chown -R jenkins /usr/lib/node_modules

RUN apt-get install -y gconf-service libasound2 libatk1.0-0 libc6 libcairo2 libcups2 libdbus-1-3 libexpat1 libfontconfig1 libgcc1 libgconf-2-4 libgdk-pixbuf2.0-0 libglib2.0-0 libgtk-3-0 libnspr4 libpango-1.0-0 libpangocairo-1.0-0 libstdc++6 libx11-6 libx11-xcb1 libxcb1 libxcomposite1 libxcursor1 libxdamage1 libxext6 libxfixes3 libxi6 libxrandr2 libxrender1 libxss1 libxtst6 ca-certificates fonts-liberation libnss3 lsb-release xdg-utils wget

RUN chown -R jenkins:jenkins /home/jenkins

#Python
RUN apt-get -q update && apt-get -y install python3 libfontconfig

# GTK LIB for Puppeteer/Chromium
RUN apt-get install -y libgbm-dev

# Make Java15 the default
RUN update-alternatives  --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives  --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
