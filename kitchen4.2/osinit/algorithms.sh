#! /bin/bash
### BEGIN INIT INFO
# Provides:          luban
# Required-Start:
# Required-Stop:
# Default-Start: 2 3 4 5
# Default-Stop: 0 1 6
# Short-Description: Start luban system.
# Description:
### END INIT INFO

LUBAN_VERSION="v1.1.3"
PATH=/sbin:/usr/sbin:/bin:/usr/bin

#**********process defines**********
root_path="/opt/knowin"
log_path="${root_path}/logs"
coredump_path="${log_path}/coredump"
rosconf_file="${root_path}/ros.env"
log_file="${log_path}/luban.service.log"
config_path="${root_path}/configs"
config_file="/media/psf/Home/Projects/sd/git/kitchen3.0/config/config.json"
application_path="~/catkin_ws/src/kitchen3.0/src"
application_path="/media/psf/Home/Projects/sd/git/kitchen3.0/src"

monitor_enable="on"
statistics_enable="on"
restart_enable="on"

chmod 777 -R ${log_path}/*
chmod 777 -R ${config_path}/*

ulimit -c unlimited

#find and set library path
lib_path=`find -L ${application_path} -name lib -type d`
export PYTHONPATH=${lib_path}:${lib_path}/comm:${lib_path}/adapter:${lib_path}/pylib:$PYTHONPATH
export LD_LIBRARY_PATH=${lib_path}:$LD_LIBRARY_PATH
source /opt/ros/${ROS_DISTRO}/setup.bash
source $rosconf_file

:<<!
process_names=(ui_bridge
				manager
				core
				core
				proxy
				adapter_assist
				adapter_utility
				adapter_arm.py
				adapter_arm.py
				)
process_args=("" "" "core_1" "core_2" "" "" "" "left" "right")
process_args_sim=("" "" "core_1" "core_2" "" "--sim true" "" "left sim" "right sim")
!

#------------------------------------------------------------------------------#
#coredump setup
#echo ${coredump_path}/core-%e-%p-%t > /proc/sys/kernel/core_pattern
#remove old core files
#------------------------------------------------------------------------------#
coredump_config() {
	coredump_head="core-"
	mkdir -p $coredump_path
	echo ${coredump_path}/${coredump_head}%e-%p-%t > /proc/sys/kernel/core_pattern
}

coredump_manager() {
	max_coredump_count=10
	coredump_head="core-"

	cd ${coredump_path}
	num_remove=`ls | grep ${coredump_head} | wc | awk '{print $1}'`
	if [ ${num_remove} -gt ${max_coredump_count} ]; then
		num_remove=$(($num_remove-${max_coredump_count}))
		ls -t | grep ${coredump_head} | tail -n ${num_remove} | xargs rm -f
		do_log "luban remove ${num_remove} old coredump files..."
	fi
	cd -
}

#------------------------------------------------------------------------------#
#parse system configs in luban config file
#jq usage example:
#         jq -r '.[1].others[]|.bin+"@@@"+.args+">>>"' config/config.json
#------------------------------------------------------------------------------#
modules=(algorithms actuator)
parse_config() {
		do_log "start parsing configfile[${config_file}]"
		process_names=()
		process_args=()
		process_srcPath=()
		break_char="@@@"
		break_char2="---"
		end_char=">>>"
		for m in "${modules[@]}"; do
			app_args=`jq -r ''.[1].${m}[]\|.bin+\"${break_char}\"+.srcPath+\"${break_char2}\"+.args+\"${end_char}\"'' $config_file`
			# echo "<###$app_args###>"
			OLD_IFS="$IFS"
			IFS="$end_char"
			arr=($app_args)
			IFS="$OLD_IFS"
			for line in "${arr[@]}"; do
				if [ ! -z "$line" ]; then
					line=`echo $line|sed -e "s/\n//g"`
					#echo -e "line:[$line]"
					bin=${line%${break_char}*}
					arg=${line#*${break_char2}}
					srcPath=${line%*${break_char2}*}
					srcPath=${srcPath#*${break_char}}
					# echo "<<<" ${bin} ${arg} ${srcPath} ">>>"
					if [ ! -z "${bin}" ]; then
						len=${#process_names[@]}
						process_names[$len]="${bin}"
						process_args[$len]="${arg}"
						process_srcPath[$len]="${srcPath}"
						if [ "${start_mode}" == "startsim" ]; then
							if [ ! -z "$arg" ]; then
								process_args[$len]="${arg} --sim all"
							else
								process_args[$len]="--sim all"
							fi
						fi
						len=$(($len+1))
						do_log "[$len] parse configfile-> cmd[${bin}] args[${arg}] srcPath[${srcPath}]"
						# echo "[$len] parse configfile-> cmd[${bin}] args[${arg}]"
					fi
				fi
			done
		done
}

#------------------------------------------------------------------------------#
#shell log output
#
#
#------------------------------------------------------------------------------#
do_log() {
	dates=`date`
	echo -e "[$dates] $1"
	echo -e "[$dates] $1" >> $log_file
}

start_mode="start"
split_mode="allinone"
#------------------------------------------------------------------------------#
#run applications
#
#
#------------------------------------------------------------------------------#
#
do_run() {
	apath=${application_path}
	if [ ! -z $3 ]; then
		apath=${application_path}"/"${3}
	fi
	echo "srcPath: " ${apath}
	fpath=`find -L $apath  -name $1 -type f`
	if [ ! -z "$fpath" ]; then
		command="$fpath $2"
		py=`echo "$1" | grep ".py"`
		if [ ! -z "$py" ]; then
			command="cd $apath;python $command"
		fi

		do_log "restarting $command"
		if [ "${split_mode}" == "split" ]; then
			gnome-terminal --geometry=50x40 -x bash -c "$command; exec bash;"
		else
			# echo "<<<<<"  $command  ">>>>>"
			$command 2>>${log_file} &
		fi
	else
		if [ ! -z $3 ]; then
			command=$1
			echo "<<<<<<<<<<<<<"  $command  ">>>>>"
		else
			do_log "can't found path for $1"
		fi
	fi
	sleep 1
	coredump_manager
}

#------------------------------------------------------------------------------#
#monitor all applications
#
#
#------------------------------------------------------------------------------#
do_monitor() {
	if [ $monitor_enable = "on" ]; then
		cnt=0
		for i in "${process_names[@]}"; do
			if [ ! -z "${process_args[$cnt]}" ]; then
				full_name="${process_names[$cnt]} ${process_args[$cnt]}"
			else
				full_name="${process_names[$cnt]}"
			fi

			#isrunning check
			isrun=`ps aux | grep -v grep | grep -v "bash -c" | grep $application_path | grep "/$full_name$" | awk '{print "CPU:"$3 " MEM:"$4 " START:"$9}'`
			tag=`printf "[%28s]" "$full_name"`
			if [ -z "$isrun" ]; then
				do_log "$tag NOT running!"

				if [ $restart_enable = "on" ]; then
					do_run "${process_names[$cnt]}" "${process_args[$cnt]}" "${process_srcPath[$cnt]}"
				fi
			fi

			cnt=$(($cnt+1))
		done
	fi
}

#------------------------------------------------------------------------------#
#memory&cpu statistics
#
#
#------------------------------------------------------------------------------#
do_statistics() {
	if [ $statistics_enable = "on" ]; then
		cnt=0
		for i in "${process_names[@]}"; do
			if [ ! -z "${process_args[$cnt]}" ]; then
				full_name="${process_names[$cnt]} ${process_args[$cnt]}"
			else
				full_name="${process_names[$cnt]}"
			fi

			#memory&cpu statistics
			stats=`ps aux | grep -v grep | grep -v "bash -c" | grep $application_path | grep "/$full_name$" | awk '{print "CPU:"$3 " MEM:"$4 " START:"$9}'`
			if [ -z "$stats" ]; then
				stats="offline"
			fi
			tag=`printf "[%48s]" "$full_name"`
			do_log "*****$tag\t$stats-----"

			cnt=$(($cnt+1))
		done
	fi
}

#------------------------------------------------------------------------------#
#start all applications
#
#
#------------------------------------------------------------------------------#
do_start() {
	max_log_count=10
	backup_head="backup-lubanlog"
	dates=`date "+%Y-%m-%d-%H:%M:%S"`
	renamed_log="${backup_head}-${dates}"
	cd ${log_path}
	mkdir ${renamed_log}
	mv *.log* ${renamed_log}/
	num_remove=`ls | grep ${backup_head} | wc | awk '{print $1}'`
	if [ ${num_remove} -gt ${max_log_count} ]; then
		num_remove=$(($num_remove-${max_log_count}))
		ls -t | grep ${backup_head} | tail -n ${num_remove} | xargs rm -rf
		do_log "luban remove ${num_remove} old log directory..."
	fi
	cd -
	do_monitor
}

#------------------------------------------------------------------------------#
#stop all applications
#
#
#------------------------------------------------------------------------------#
do_stop() {
	cnt=0
	for i in "${process_names[@]}"; do
		if [ ! -z "${process_args[$cnt]}" ]; then
			full_name="${process_names[$cnt]} ${process_args[$cnt]}"
		else
			full_name="${process_names[$cnt]}"
		fi

		#memory&cpu statistics
		pid=`ps aux |grep -v grep | grep $application_path | grep "/$full_name" | awk '{print $2}'`
		if [ ! -z "$pid" ]; then
			kill -9 $pid
			tag=`printf "[%28s]" "$full_name"`
			do_log "$tag pid($pid) killed"
		fi

		cnt=$(($cnt+1))
	done
}

#------------------------------------------------------------------------------#
#daemon thread
#
#
#------------------------------------------------------------------------------#
do_daemon() {
	while :
	do
		sleep 5
		do_monitor
		sleep 5
		do_monitor
		do_statistics
	done
}

do_log "***************luban.service start********************"
do_log "***************version:${LUBAN_VERSION}***************"
#main process entry
case "$1" in
  start)
  	do_log "start luban now..."
	start_mode="start"
	split_mode="$2"
	parse_config
	do_start
	$0 daemon $* &
	coredump_config
	do_log "start luban done."
	exit 0
	;;
  startsim)
  	do_log "start luban in sim mode now..."
	start_mode="startsim"
	split_mode="$2"
	parse_config
	do_start
	$0 daemon $* &
	coredump_config
	do_log "start luban done."
	exit 0
	;;
  restart)
	do_log "luban is restarting..."
	ps aux |grep -v grep | grep "$0 daemon" | awk '{print $2}'|xargs kill -9
	parse_config
	do_stop
	sleep 1
	do_start
	$0 daemon &
	coredump_config
	do_log "luban restart done."
	exit 0
	;;
  stop)
	do_log "stop luban now...."
	ps aux |grep -v grep | grep "$0 daemon" | awk '{print $2}'|xargs kill -9
	parse_config
	do_stop
	do_log "stop luban done."
	exit 0
	;;
  status)
	do_log "statistics luban now...."
	parse_config
  	do_statistics
	coredump_config
	do_log "statistics luban done."
	exit 0
	;;
  daemon)
	do_log "start daemon now...."
	start_mode="$2"
	split_mode="$3"
	parse_config
	do_daemon
	do_log "daemon done."
	exit 0
	;;
  *)
	echo "Usage: $0 start|startsim|stop|restart|status" >&2
	exit 3
	;;
esac
