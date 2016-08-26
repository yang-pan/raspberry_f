#!/bin/sh

# timeout[min] 1440[min]=24h
#timeout_min=1440
timeout_min=10

# 空き容量が10MB(10240[KB])より少なければ終了する
min_storage=10240

while [ 1 ]
do
	avail=`df | grep rootfs | awk {'print $4'}`
	echo `date`, timeout_min:${timeout_min}, avail[Byte]:${avail}
	timeout_min=`expr ${timeout_min} - 1`

	# タイムアウト
	if [ x"${timeout_min}" = "x0" ]; 
	then
		echo timeout!! finish!!
		kill `ps aux | grep Sensor_log | grep -v grep | awk {'print $2'}`
		break
	fi
	# 容量不足チェック
	if [ ${avail} -le ${min_storage} ];
	then
		echo strage sukunai!! finish!!
		kill `ps aux | grep Sensor_log | grep -v grep | awk {'print $2'}`
		break
	fi

#	sleep 1
	sleep 60
done

