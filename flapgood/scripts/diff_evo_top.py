from iterative_sdf import create_new_sdf
from cost_process import max_vel_cost
import numpy as np
from scipy.optimize import OptimizeResult
import scipy as sc
import shutil
import os
import subprocess
from util.message import Message

dir_path = os.path.dirname(os.path.abspath(__file__))
data_dir_path = os.path.join(dir_path, '..', 'opt_data')
def start_diff_evo(iterations: int = 10, title: str = 'beta_test', popsize: int = 10):
    title_dir_path = os.path.join(data_dir_path, title)
    if not os.path.exists(title_dir_path):
        os.makedirs(data_dir_path)
    Message.debug("TESTING: " + title, True)
    Message.info("folder path: " + title_dir_path)
    
    #We are optimizing for link lengths: a,b,c,d,e, and fixed point mag
    original_lengths = np.array([15.25, 25.125 + 15.125, 31.035, 60.035 , 15.125, 53.43])
    
    #unit scale
    original_lengths = original_lengths / 10
    
    parameter_lower_bound = original_lengths * 0.5
    parameter_lower_bound[5] = 53.43
    parameter_upper_bound = original_lengths * 2
    parameter_upper_bound[5] = 53.43
    bounds = sc.optimize.Bounds(parameter_lower_bound, parameter_upper_bound)

    # -A - B + C + fixed > 0
    # 
    A = np.array([[-1, -1,  1,  0,  0,  1],
                  [-1,  1,  0,  0,  0,  0],
                  [-1,  0,  1,  0,  0,  0],
                  [-1,  0,  0,  0,  0,  1],
                  [ 0,  1, -1,  0,  0,  0],
                  [ 0,  1,  0,  0,  0,  -1],
                ])

    linear_constraint_lower_bound = np.full(A.shape[0], [1e-3]) # so its not equal to 0, removing eq not supp by scipy xd
    linear_constraint_upper_bound = np.full(A.shape[0], [np.inf])
    linear_constraints = sc.optimize.LinearConstraint(A, linear_constraint_lower_bound, linear_constraint_upper_bound)
    
    sc.optimize.differential_evolution(sim_start, bounds, constraints=linear_constraints, x0= original_lengths, strategy='best1bin', maxiter=iterations, popsize=popsize, polish=False, callback=opt_callback)
def sim_start(opt_params):
    Message.data("sim iter start \n opt_params: " + str(opt_params))
    
    
def opt_callback():
    pass