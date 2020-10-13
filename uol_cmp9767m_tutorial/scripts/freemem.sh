#!/bin/bash

while true; do val=$(cat /proc/meminfo | grep MemFree | cut -f2 -d":" | tr -s " "); echo $val; rostopic pub /memfree std_msgs/String "$val" -1; done
