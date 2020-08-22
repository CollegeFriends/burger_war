#!/bin/bash

roscd burger_AI

# resetする
bash bash_scripts/reset.sh

# set judge server state "running"
bash judge/test_scripts/set_running.sh localhost:5000

# launch robot control node