#!/bin/bash
# roscore
killall -9 roscore
killall -9 rosmaster
# kill all jobs
kill -9 $(jobs -p)
killall -9 python3
