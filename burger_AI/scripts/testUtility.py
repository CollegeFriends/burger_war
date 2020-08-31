#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess

# Sim環境を立ち上げる
def start_sim():
    try:
        subprocess.call("bash_scripts/setup_sim.sh",shell=True)        
    except:
        import traceback
        traceback.print_exc()

# Resetをかける
def reset_sim():
    try:    
        subprocess.call("bash_scripts/reset.sh",shell=True)        
    except:
        import traceback
        traceback.print_exc()

        
        
