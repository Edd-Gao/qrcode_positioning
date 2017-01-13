FROM ros:indigo-ros-base

#update the source list
RUN apt-get update; exit 0

#install https transport util for apt
RUN apt-get install apt-transport-https -y

#substitute the original mirror with tsinghua tuna mirror
RUN sh -c '. /etc/lsb-release && echo "deb https://mirrors.tuna.tsinghua.edu.cn/ros/ubuntu/ $DISTRIB_CODENAME main" > /etc/apt/sources.list.d/ros-latest.list'
ADD ./sources.list /etc/apt/sources.list

#update apt-get source list
RUN apt-get update

#install necessary ros packages

#install image processing related packages
RUN apt-get install ros-indigo-camera-calibration-parsers -y