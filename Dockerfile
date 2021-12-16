# Authors: Rick Jellema & Rick van Druten
# Location: Hogeschool Arnhem en Nijmegen
# Goal: Docker agent for Jenkins Elixir Job
# This Dockerfile is used to build an image containing Erlang, Elixir and Mix.
# From Erlang runs on Debian:Buster 10.
FROM erlang:22
MAINTAINER Rody Middelkoop <rody.middelkoop@gmail.com>

# Elixir expects utf8.
ENV ELIXIR_VERSION="v1.12.3"
ENV LC_ALL en_US.UTF-8 
ENV LANG en_US.UTF-8  
ENV LANGUAGE en_US:en   

# Install Elixir.
RUN set -xe \
	&& ELIXIR_DOWNLOAD_URL="https://github.com/elixir-lang/elixir/archive/${ELIXIR_VERSION}.tar.gz" \
	&& ELIXIR_DOWNLOAD_SHA256="c5affa97defafa1fd89c81656464d61da8f76ccfec2ea80c8a528decd5cb04ad" \
	&& curl -fSL -o elixir-src.tar.gz $ELIXIR_DOWNLOAD_URL \
	&& echo "$ELIXIR_DOWNLOAD_SHA256  elixir-src.tar.gz" | sha256sum -c - \
	&& mkdir -p /usr/local/src/elixir \
	&& tar -xzC /usr/local/src/elixir --strip-components=1 -f elixir-src.tar.gz \
	&& rm elixir-src.tar.gz \
	&& cd /usr/local/src/elixir \
	&& make install clean \
	&& find /usr/local/src/elixir/ -type f -not -regex "/usr/local/src/elixir/lib/[^\/]*/lib.*" -exec rm -rf {} + \
	&& find /usr/local/src/elixir/ -type d -depth -empty -delete

# Jenkins configuration
# Check for and install new updates
RUN apt-get update && apt-get upgrade -y

# Install SSH, dockerfile when build runs on debian, but does not have OpenSSH pre-installed.
RUN apt-get install openssh-server -y

# Create /var/run/sshd directory, otherwise OpenSSH won't start automatically and docker container will shut down.
RUN mkdir -p /var/run/sshd

# Define Jenkins workspace
WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/sh jenkins && echo "jenkins:jenkins" | chpasswd
RUN apt-get update && apt-get install -y git

#JDK15 for connecting Jenkins master and slave using JNLP
RUN \
apt-get -q update && \
apt-get install curl -y && \
mkdir -p /usr/lib/jvm && \
cd /usr/lib/jvm && curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz


# Setup Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt
COPY "sonar-scanner.properties" /opt/sonar-scanner-4.5.0.2216/conf

# Make Java JDK15 the default JDK. Necessary for Sonar scanner.
RUN update-alternatives --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac

# Expose SSH port 22
EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]











