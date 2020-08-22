#!/usr/bin/env python
# -*- coding: utf-8 -*-

import os
import subprocess

# Resetをかける
def reset_sim():
    try:    
        subprocess.call("bash_scripts/reset.sh",shell=True)        
    except:
        import traceback
        traceback.print_exc()

        
        
