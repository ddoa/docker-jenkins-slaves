FROM debian:bookworm

# See: 
# 	Why not: https://docs.docker.com/engine/faq/#why-is-debian_frontendnoninteractive-discouraged-in-dockerfiles
# 	But how you may use it: https://github.com/moby/moby/issues/4032 for more info.
# 
# Basically adding ENV saves the setting in the image but putting it in the run-command, before 
# the "apt-get install", should just uses it in that command. Does not seem to work?
ENV DEBIAN_FRONTEND=noninteractive

WORKDIR /data

# Set user jenkins to the image
RUN useradd -m -d /home/jenkins -s /bin/bash jenkins && echo "jenkins:jenkins" | chpasswd

# Include contrib and non-free
RUN echo "deb http://ftp.nl.debian.org/debian bookworm main contrib non-free" > /etc/apt/sources.list

# Installing as the first package in the first layer prohibits warnings in the remainder
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
	apt-utils

# Install all the non-X-windows tools needed for development
# To work around some obscure error "-o APT::Immediate-Configure=false" has to be used	
RUN apt-get update && apt-get upgrade -y && apt-get install -o APT::Immediate-Configure=false -y \
	build-essential	\
	m4	\
	libtool	\
	automake	\
	autoconf	\
	autoconf-archive	\
	gdb	\
	gdbserver	\
	git	\
	subversion	\
	cppcheck	\
	lcov	\
	valgrind	\
	linux-perf	\
	doxygen	\
	doxygen-gui\
	cmake	\
	git	\
	wget	\
	libboost-all-dev	\
	bear	\
	clang	\
	clang-format	\
	clang-tidy	\
	clang-tools	\	
	locales	

# Install some non-X-Windows utilities	
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
	mc	\
	locate	\
	man	\
	sudo

# Install the wxWidgets prerequisites
RUN apt-get update && apt-get upgrade -y && apt-get install -y \
	libgtk-3-dev

RUN apt-get update && apt-get upgrade -y && apt-get install -y \
	libwxgtk3.2-1	\
	libwxgtk3.2-dev  
  
# Install Java 15 so the image can be used by Jenkins as a worker
RUN \
  apt-get -q update && \
  apt-get install curl -y && \
  mkdir /usr/lib/jvm && cd /usr/lib/jvm &&  curl -O https://download.java.net/java/GA/jdk15.0.1/51f4f36ad4ef43e39d0dfdbaf6549e32/9/GPL/openjdk-15.0.1_linux-x64_bin.tar.gz && \
  tar -xvzf openjdk-15.0.1_linux-x64_bin.tar.gz && \
  rm -rf openjdk-15.0.1_linux-x64_bin.tar.gz

RUN update-alternatives  --install /usr/bin/java java /usr/lib/jvm/jdk-15.0.1/bin/java 1000 && update-alternatives  --install /usr/bin/javac javac /usr/lib/jvm/jdk-15.0.1/bin/javac 1001
RUN update-alternatives --set java /usr/lib/jvm/jdk-15.0.1/bin/java && update-alternatives --set javac /usr/lib/jvm/jdk-15.0.1/bin/javac

# Install Sonar Scanner so the image can run a local SQ analysis
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.7.0.2747-linux.zip --quiet && unzip sonar-scanner-cli-4.7.0.2747-linux.zip -d /opt

COPY "sonar-scanner.properties" /opt/sonar-scanner-cli-4.7.0.2747-linux/conf

# Update the database so we can find stuff. Keep this as the last command.
RUN updatedb
