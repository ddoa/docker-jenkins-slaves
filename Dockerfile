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
RUN apt-get install -y unzip && wget http://apache.cs.uu.nl/maven/maven-3/3.5.4/binaries/apache-maven-3.5.4-bin.zip && cd /opt ; unzip /data/apache-maven-3.5.4-bin.zip
RUN apt-get install -y unzip && wget http://apache.cs.uu.nl/maven/maven-3/3.6.1/binaries/apache-maven-3.6.1-bin.zip && cd /opt ; unzip /data/apache-maven-3.6.1-bin.zip


# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-2.8.zip --quiet && unzip sonar-scanner-2.8.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

RUN apt-get update && apt-get install -y --force-yes git curl wget unzip libgconf-2-4 gdb libstdc++6 libglu1-mesa fonts-droid-fallback lib32stdc++6 python3 \
    && apt-get clean \
    && git clone -b beta https://github.com/flutter/flutter.git /usr/local/flutter \
    && chown -R jenkins:jenkins /usr/local/flutter

RUN apt-get update && apt-get install -y unzip expect sudo && wget https://services.gradle.org/distributions/gradle-4.4-bin.zip --quiet && unzip gradle-4.4-bin.zip -d /opt
ENV GRADLE_HOME /opt/gradle-4.4
ENV PATH $PATH:$GRADLE_HOME/bin

# Copy install tools
RUN mkdir /opt/tools
COPY tools/android-accept-licenses.sh /opt/tools/android-accept-licenses.sh
ENV PATH ${PATH}:/opt/tools

# Install Android SDK
RUN cd /opt && wget --output-document=android-sdk.tgz --quiet https://dl.google.com/android/android-sdk_r24.4.1-linux.tgz && \
  tar xzf android-sdk.tgz && \
  rm -f android-sdk.tgz && \
  chown -R root.root android-sdk-linux && \
  /opt/tools/android-accept-licenses.sh "android-sdk-linux/tools/android update sdk --all --no-ui --filter platform-tools,tools" && \
  /opt/tools/android-accept-licenses.sh "android-sdk-linux/tools/android update sdk --all --no-ui --filter platform-tools,tools,build-tools-25.0.0,build-tools-25.0.3,build-tools-26.0.2,build-tools-27.0.3,build-tools-28.0.3,android-25,android-26,android-27,android-28,addon-google_apis_x86-google-21,extra-android-support,extra-android-m2repository,extra-google-m2repository,extra-google-google_play_services,sys-img-armeabi-v7a-android-24"

# Install OpenJDK8
RUN \
    cd /opt && wget https://github.com/AdoptOpenJDK/openjdk8-binaries/releases/download/jdk8u232-b09/OpenJDK8U-jdk_x64_linux_hotspot_8u232b09.tar.gz && \
    cd /usr/lib/jvm && tar xvzf /opt/OpenJDK8U-jdk_x64_linux_hotspot_8u232b09.tar.gz

RUN export JAVA_HOME="/usr/lib/jvm/jdk8u232-b09/" && echo yes | /opt/android-sdk-linux/tools/bin/sdkmanager "extras;m2repository;com;android;support;constraint;constraint-layout;1.0.2" && echo yes | /opt/android-sdk-linux/tools/bin/sdkmanager "extras;m2repository;com;android;support;constraint;constraint-layout-solver;1.0.2"

#Install npm
RUN apt-get install -y curl \
  && curl -sL https://deb.nodesource.com/setup_10.x | bash - \
  && apt-get install -y nodejs \
  && curl -L https://www.npmjs.com/install.sh | sh

# Setup environment
ENV ANDROID_HOME /opt/android-sdk-linux
ENV ANDROID_SDK_HOME /opt/android-sdk-linux
ENV PATH ${PATH}:${ANDROID_HOME}/tools:${ANDROID_HOME}/platform-tools

RUN  /usr/local/flutter/bin/flutter doctor -v \
    && rm -rfv /flutter/bin/cache/artifacts/gradle_wrapper

# Setting flutter and dart-sdk to PATH so they are accessible from terminal
ENV PATH="/usr/local/flutter/bin:/usr/local/flutter/bin/cache/dart-sdk/bin:${PATH}"

# Correct permissions
RUN chown -R jenkins /usr/local/flutter

# Install lcov to convert lcovinfo to HTML
RUN apt-get update -qq -y && apt-get install lcov -y

# Standard SSH port
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
