#!/bin/bash

sitl_num=$1	#sitl drone numbers
rc_script=$2	#rc script name
src_path=$3	#firmware dir path
swarm_sitl_launcher_path=$4

sim_port=15019
mav_port=15010
mav_port2=15011

mav_oport=15015
mav_oport2=15016

port_step=10

pkill px4
sleep 2

mkdir -p ${swarm_sitl_launcher_path}/tmp/SDFs

mkdir -p $swarm_sitl_launcher_path/posix

cd posix

user=`whoami`
n=1
while [ $n -le $sitl_num ]; do
 if [ ! -d $n ]; then
  mkdir -p $n
  cd $n

  mkdir -p rootfs/fs/microsd
  mkdir -p rootfs/eeprom
  touch rootfs/eeprom/parameters

  cp ${src_path}/ROMFS/px4fmu_common/mixers/quad_w.main.mix ./
  cat $swarm_sitl_launcher_path/init/$rc_script | sed s/_SIMPORT_/${sim_port}/ | sed s/_MAVPORT_/${mav_port}/g | sed s/_MAVOPORT_/${mav_oport}/ | sed s/_MAVPORT2_/${mav_port2}/ | sed s/_MAVOPORT2_/${mav_oport2}/ > rcS

  cd ..
 fi

 cd $n
 
 sudo -b ${src_path}/build_posix_sitl_default/src/firmware/posix/px4 -d rcS #>out.log 2>err.log

 cd ..

 cat ${swarm_sitl_launcher_path}/models/iris/iris.sdf | sed s/_SIMPORT_/${sim_port}/ > ${swarm_sitl_launcher_path}/tmp/SDFs/iris_${n}.sdf

 n=$(($n + 1))
 sim_port=$(($sim_port + $port_step))
 mav_port=$(($mav_port + $port_step))
 mav_port2=$(($mav_port2 + $port_step))
 mav_oport=$(($mav_oport + $port_step))
 mav_oport2=$(($mav_oport2 + $port_step))
done

