FROM reactnativecommunity/react-native-android:2.1

RUN curl -sS https://dl.yarnpkg.com/debian/pubkey.gpg | apt-key add - 

# Sonar Scanner
RUN apt-get update && apt-get install -y unzip wget bzip2 && wget https://binaries.sonarsource.com/Distribution/sonar-scanner-cli/sonar-scanner-cli-4.5.0.2216.zip --quiet && unzip sonar-scanner-cli-4.5.0.2216.zip -d /opt

RUN useradd -m -d /home/jenkins -s /bin/bash jenkins && echo "jenkins:jenkins" | chpasswd

RUN chown -R jenkins:jenkins /home/jenkins
RUN chown -R jenkins:jenkins /opt/android

# Standard SSH port

# Add docker-client to be able to build, run etc. docker containers
RUN apt-get install -y docker openssh-server 

RUN service ssh start

EXPOSE 22

# Default command
CMD ["/usr/sbin/sshd", "-D"]
