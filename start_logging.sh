#!/bin/sh

logfile=Sensor_log_`date +%Y%m%d_%H%M%S`.bin
echo logfile name: ${logfile}

./Sensor_log ${logfile}

