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

# gpg keys listed at https://github.com/nodejs/node
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 4ED778F539E3634C779C87C6D7062848A1AB005C
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 141F07595B7B3FFE74309A937405533BE57C7D57
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 94AE36675C464D64BAFA68DD7434390BDBE9B9C5
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 74F12602B6F1C4E913FAA37AD3A89613643B6201
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 71DCFD284A79C3B38668286BC97EC7A07EDE3FC1
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 8FCCA13FEF1D0C2E91008E09770F7A9A5AE15600
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys C4F0DFFF4E8C1A8236409D08E73BC641CC11F4C8
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys C82FA3AE1CBEDC6BE46B9360C43CEC45C17AB93C
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys DD8F2338BAE7501E3DD5AC78C273792F7D83545D
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys A48C2BEE680E841632CD4E44F07496B3EB3C1762
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys 108F52B48DB57BB0CC439B2997B01419BD92F80A
RUN gpg --keyserver hkps://keys.openpgp.org --recv-keys B9E2F5981AA6E0CD28160D9FF13993A75599653C

ENV NPM_CONFIG_LOGLEVEL info
ENV NODE_VERSION 18.13.0

RUN curl -SLO "https://nodejs.org/dist/v$NODE_VERSION/node-v$NODE_VERSION-linux-x64.tar.xz" \
  && curl -SLO "https://nodejs.org/dist/v$NODE_VERSION/SHASUMS256.txt.asc" \
  && gpg --receive-keys 8FCCA13FEF1D0C2E91008E09770F7A9A5AE15600 \
  && gpg --batch --decrypt --output SHASUMS256.txt SHASUMS256.txt.asc \
  && grep " node-v$NODE_VERSION-linux-x64.tar.xz\$" SHASUMS256.txt | sha256sum -c - \
  && tar -xJf "node-v$NODE_VERSION-linux-x64.tar.xz" -C /usr/local --strip-components=1 \
  && rm "node-v$NODE_VERSION-linux-x64.tar.xz" SHASUMS256.txt.asc SHASUMS256.txt \
  && ln -s /usr/local/bin/node /usr/local/bin/nodejs

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

RUN npm install -g @angular/cli && chown -R jenkins /usr/local/lib/node_modules

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
