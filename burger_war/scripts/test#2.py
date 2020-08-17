from test_util import *
from controlBot import ControlBot
import optuna
import numpy as np
cnt = 0

def objective(trial):
    global cnt
    cnt += 1
    try:    
        reset_sim()
        if False:
            Kp_dist = trial.suggest_uniform('Kp_dist',0.95,1.05)
            Ki_dist = trial.suggest_uniform('Ki_dist',0.0001,0.0002)        
        else:
            Kp_dist = 0.9731625194682382
            Ki_dist = 0.00016981785117853974    
            
        if False:
            Kp_dir = trial.suggest_uniform('Kp_dir',4,5)
            Ki_dir = trial.suggest_uniform('Ki_dir',0.00025,0.00035)
        else:
            Kp_dir = 4.524248499803513
            Ki_dir = 0.0002950570968987406
    
        if True:
            Kp_dir2 = trial.suggest_uniform('Kp_dir2',1,5)
            Ki_dir2 = trial.suggest_uniform('Ki_dir2',0.01,0.1)    
        else:
            Kp_dir2 = 3.2620494609668675
            Ki_dir2 = 0.004936654954648014
        
        
        score = 0
        for i in range(8):
            reset_sim()
            a = ControlBot(mode="dir2",type=i)           
            rospy.init_node('control_node') 
            a.gain_dist = np.array([Kp_dist,Ki_dist,0.0])
            a.gain_dir = np.array([Kp_dir, Ki_dir, 0.0])
            a.gain_dir2 = np.array([Kp_dir2, Ki_dir2, 0.0])
        
            score += a.strategy(timeout=30)        
            print(i,score)           

    except:
        import traceback
        traceback.print_exc()

    with open('log.csv', 'a') as f:
        print("{},{},{},{}".format(cnt,score,Kp_dir2,Ki_dir2), file=f)
    return score

study = optuna.create_study()
study.optimize(objective,n_trials=100)

print(study.best_params)