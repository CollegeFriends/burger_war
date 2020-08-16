from test_util import *
from controlBot import ControlBot

reset_sim()
a = ControlBot()           
a.strategy(timeout=10.0)