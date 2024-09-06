#!/usr/bin/env python
import time
import sys

if len(sys.argv) > 1:
    sleep_time = int(sys.argv[1])
else:
    sleep_time = 1  # Default to 1 second if no argument is provided

time.sleep(sleep_time)