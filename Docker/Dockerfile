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

#install necessary packages
RUN apt-get install g++ -y

#create and init catkin_ws
RUN mkdir -p /catkin_ws/src
RUN bash -c "source /opt/ros/indigo/setup.bash;catkin_init_workspace /catkin_ws/src"
RUN bash -c "source /opt/ros/indigo/setup.bash;catkin_make -C /catkin_ws"

#export source setup.bash to .bashrc
RUN echo "source /catkin_ws/devel/setup.bash" >> /.bashrc

#install necessary ros packages

#install image processing related packages
RUN apt-get install ros-indigo-camera-calibration-parsers -y
RUN apt-get install ros-indigo-ecl-command-line -y