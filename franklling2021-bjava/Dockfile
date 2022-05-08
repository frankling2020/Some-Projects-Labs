# base
FROM ubuntu:20.04

# MAINTAINER
MAINTAINER frankling <carofrank2000@gmail.com>

ENV DEBIAN_FRONTEND noninteractive

# Basic Setup
RUN mkdir ~/config
ADD config/ /root/config/

RUN sed -i s@/archive.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list \
	&& sed -i s@/security.ubuntu.com/@/mirrors.aliyun.com/@g /etc/apt/sources.list \
	&& apt-get update \
	&& apt-get upgrade -y \
	&& apt-get remove -y --purge ssh \ 
	&& apt-get install -y ssh pdsh vim net-tools rsync software-properties-common \
	&& add-apt-repository -y ppa:linuxuprising/java \
	&& apt-get install -y openjdk-8-jdk\
	&& apt-get update \
	&& apt-get clean \
	&& apt clean

# Hadoop Download
RUN cd ~ \
	&& wget https://mirrors.tuna.tsinghua.edu.cn/apache/hadoop/core/hadoop-3.3.2/hadoop-3.3.2.tar.gz -O hadoop.tar.gz \
	&& tar -xzf hadoop.tar.gz && rm hadoop.tar.gz && mv hadoop-3.3.2 hadoop \
	&& echo 'export JAVA_HOME=/usr/lib/jvm/java-8-openjdk-amd64' | tee -a ~/hadoop/etc/hadoop/hadoop-env.sh

# SSH Setup
RUN ssh-keygen -t rsa -P '' -f ~/.ssh/id_rsa \
	&& cat ~/.ssh/id_rsa.pub >> ~/.ssh/authorized_keys \
	&& cat ~/config/add_profile | tee -a /etc/profile \
	&& service ssh --full-restart \
	&& cat ~/config/add_bashrc | tee -a ~/.bashrc \
	&& bash ~/.bashrc

# Restore
ENV DEBIAN_FRONTEND interactive

# Hadoop configuration
RUN mv -fb ~/config/*.xml ~/hadoop/etc/hadoop/

	
# RUN: 
# sudo docker build -f Dockerfile -t bjava ./
# sudo docker run -it --name jbase -h hadoop bjava
