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

# Define working directory.
WORKDIR /data

RUN apt-get update && apt-get install -y wget
RUN mkdir /usr/lib/jvm && wget https://download.java.net/java/GA/jdk15.0.2/0d1cfde4252546c6931946de8db48ee2/7/GPL/openjdk-15.0.2_linux-x64_bin.tar.gz && tar xvzf openjdk-15.0.2_linux-x64_bin.tar.gz -C /usr/lib/jvm

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins &&\
    echo "jenkins:jenkins" | chpasswd

RUN apt-get update && apt-get install -y git
RUN apt-get install -y unzip && wget http://apache.cs.uu.nl/maven/maven-3/3.5.4/binaries/apache-maven-3.5.4-bin.zip && cd /opt ; unzip /data/apache-maven-3.5.4-bin.zip
RUN apt-get install -y unzip && wget http://apache.cs.uu.nl/maven/maven-3/3.6.3/binaries/apache-maven-3.6.3-bin.zip && cd /opt ; unzip /data/apache-maven-3.6.3-bin.zip


# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-2.8/conf

RUN apt-get update && apt-get install -y --force-yes git curl wget unzip libgconf-2-4 gdb libstdc++6 libglu1-mesa fonts-droid-fallback lib32stdc++6 python3 \
    && apt-get clean \
    && git clone -b beta https://github.com/flutter/flutter.git /usr/local/flutter \
    && chown -R jenkins:jenkins /usr/local/flutter

RUN apt-get update && apt-get install -y unzip expect sudo && wget https://services.gradle.org/distributions/gradle-4.4-bin.zip --quiet && unzip gradle-4.4-bin.zip -d /opt
ENV GRADLE_HOME /opt/gradle-4.4
ENV PATH $PATH:$GRADLE_HOME/bin

# Install OpenJDK11 (minimum version to use together with Android)
RUN \
    cd /opt && wget https://github.com/AdoptOpenJDK/openjdk11-binaries/releases/download/jdk-11.0.9.1%2B1/OpenJDK11U-jdk_x64_linux_hotspot_11.0.9.1_1.tar.gz && \
    cd /usr/lib/jvm && tar xvzf /opt/OpenJDK11U-jdk_x64_linux_hotspot_11.0.9.1_1.tar.gz

ENV JAVA_HOME /usr/lib/jvm/jdk-11.0.9.1+1
ENV PATH ${PATH}:{JAVA_HOME}/bin

# Install Android SDK
RUN mkdir -p .android && touch .android/repositories.cfg
RUN wget -O sdk-tools.zip https://dl.google.com/android/repository/commandlinetools-linux-9477386_latest.zip
RUN mkdir /opt/android-sdk 
RUN unzip sdk-tools.zip && mv cmdline-tools latest && mkdir cmdline-tools && mv latest cmdline-tools && mv cmdline-tools /opt/android-sdk && rm sdk-tools.zip
RUN cd /opt/android-sdk/cmdline-tools/latest/bin && yes | ./sdkmanager --licenses

RUN export JAVA_HOME="/usr/lib/jvm/jdk-11.0.9.1+1" && cd /opt/android-sdk/cmdline-tools/latest/bin && ./sdkmanager "cmdline-tools;9.0" "patcher;v4" "platform-tools" "platforms;android-29" "platforms;android-30" "platforms;android-33" "platforms;android-28" "tools" 
RUN export JAVA_HOME="/usr/lib/jvm/jdk-11.0.9.1+1" && echo yes | /opt/android-sdk/cmdline-tools/latest/bin/sdkmanager "extras;m2repository;com;android;support;constraint;constraint-layout;1.0.2" && echo yes | /opt/android-sdk/cmdline-tools/latest/bin/sdkmanager "extras;m2repository;com;android;support;constraint;constraint-layout-solver;1.0.2"

#Install npm
RUN apt-get install -y curl \
  && curl -sL https://deb.nodesource.com/setup_14.x | sudo bash - \
  && apt-get install -y nodejs \
  && chown -R jenkins:jenkins /usr/lib/node_modules 

USER jenkins
RUN npm install -g puppeteer
USER root

# Setup environment
ENV ANDROID_HOME /opt/android-sdk
ENV ANDROID_SDK_HOME /opt/android-sdk
ENV PATH ${PATH}:${ANDROID_HOME}/tools:${ANDROID_HOME}/platform-tools:${ANDROID_HOME}/cmdline-tools/latest/bin

RUN  /usr/local/flutter/bin/flutter doctor -v \
    && rm -rfv /flutter/bin/cache/artifacts/gradle_wrapper
RUN chown -R jenkins:jenkins /usr/local/flutter && chgrp -R jenkins /usr/local/flutter && chown -R jenkins:jenkins /root/.pub-cache && chmod 755 /root && chown -R jenkins /opt/android-sdk

# Setting flutter and dart-sdk to PATH so they are accessible from terminal
ENV PATH="/usr/local/flutter/bin:/usr/local/flutter/bin/cache/dart-sdk/bin:${PATH}"

# Install lcov to convert lcovinfo to HTML
RUN apt-get update -qq -y && apt-get install lcov -y

# Add docker-client to be able to build, run etc. docker containers
RUN apt-get install -y docker

# Set JDK8 as default
RUN update-alternatives --install /usr/bin/java java /usr/lib/jvm/jdk-11.0.9.1+1/bin/java 1
RUN update-alternatives --install /usr/bin/javac javac /usr/lib/jvm/jdk-11.0.9.1+1/bin/javac 1

# Added Kotlin
RUN cd /usr/lib && \
    wget https://github.com/JetBrains/kotlin/releases/download/v1.8.0/kotlin-compiler-1.8.0.zip && \
    unzip kotlin-compiler-*.zip && \
    rm kotlin-compiler-*.zip && \
    rm -f kotlinc/bin/*.bat

ENV PATH $PATH:/usr/lib/kotlinc/bin

# Standard SSH port
EXPOSE 22

# Default command
#CMD ["/usr/sbin/sshd", "-D"]
