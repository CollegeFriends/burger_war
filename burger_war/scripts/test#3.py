from test_util import *
from controlBot2 import ControlBot
import numpy as np
import time
import math
# 周回コースを回る
# 要した時間を確認
reset_sim(judge=True)
a = ControlBot()     

pattern = "B"
try:        
    if pattern == "A":
        a.put_goal(-0.75,-0.75,None)
        a.put_goal(-1.3,0.0,None)
        a.put_goal(-0.75,0.75,None)
        a.put_goal(0.0,1.3,None)
        a.put_goal(0.75,0.75,None)
        a.put_goal(1.3,0.0,None)
        a.put_goal(0.75,-0.75,None)
        a.put_goal(0.0,-1.3,None)          
    elif pattern == "B":        
        a.put_goal(0,-1.3,math.pi/2.0)
        a.put_goal(0,-0.5,math.pi/2.0)         # 1                    
        a.put_goal(0,-1.3,math.pi/2.0)
        a.put_goal(-0.53,-0.83,math.pi/2.0)     # 2
        a.put_goal(-0.75,-0.75,None)            
        a.put_goal(-1.3,0.0,None)                
        a.put_goal(-0.53,-0.3,-math.pi/2.0)    # 3
        a.put_goal(-0.5,0,0.0)                 # 4
        a.put_goal(-0.53,0.3,math.pi/2.0)      # 5
        a.put_goal(-1.3,0.0,None)
        a.put_goal(-0.75,0.75,None)            
        a.put_goal(-0.53, 0.83,-math.pi/2.0)    # 6        
        
        a.put_goal(0.0,1.3,None)
        a.put_goal(0,0.5,-math.pi/2.0)         # 7
        a.put_goal(0.0,1.3,None)                

        a.put_goal(0.53,0.83,-math.pi/2.0)      # 8
        a.put_goal(0.75,0.75,None)            
        a.put_goal(1.3,0.0,None)                
        a.put_goal(0.5,0.3,math.pi/2.0,timeout=30.0)       # 9
        a.put_goal(0.5,0,math.pi)                           # 10
        a.put_goal(0.5,-0.3,-math.pi/2.0,timeout=35.0)     # 11
        a.put_goal(1.3,0.0,None)
        a.put_goal(0.75,-0.75,None)            
        a.put_goal(0.53, -0.83,math.pi/2.0)               # 12       
        a.put_goal(0,-1.3,math.pi/2.0)
        
    base_time = a.clk
    a.strategy(END=True)
    t = a.clk
    print("要した時間 : {}".format(t-base_time))
except: 
    import traceback
    traceback.print_exc()   
    pass
    
