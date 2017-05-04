import numpy as np
import random
import itertools as it

def OTG(start, goal, T, lateral_cost_functions, longitudinal_cost_functions):
    """
    Performs optimal trajectory generation.
    
    Args:
      start [float]: 
        the start state [s, s_dot, s_double_dot, d, d_dot, d_double_dot]
        
      goal  [float]: 
        the start state [s, s_dot, s_double_dot, d, d_dot, d_double_dot]
        
      T float:
        desired duration of maneuver
        
      lateral_cost_functions [function]:
        an array of cost functions to be applied to lateral 
        (s direction) trajectories
        
      longitudinal_cost_functions [function]:
        an array of cost functions to be applied to longitudinal 
        (d direction) trajectories
    
    Returns:
      A length two tuple. The first element is a length 6 array of values
      which correspond to the coefficients in the fifth degree (jerk minimized)
      LATERAL trajectory represented by
      
      s(t) = a0 + a1*t + a2*t**2 + a3*t**3 + a4*t**4 + a5*t**5
      
      the second element is a similar representation of the longitudinal 
      trajectory d(t).
      
    """
    
    # first generate the trajectories
    lateral_trajectories = generate_trajectories(np.append(start[:3], goal[:3]), T)
    longitudinal_trajectories = generate_trajectories(np.append(start[3:], goal[3:]), T)
    
    # combined will store tuples of the form (cost, s(t), d(t))
    combined = []
    for lat_traj in lateral_trajectories:
        
        # calculate cost associated with lateral trajectory
        lat_cost = 0
        for lateral_cf in lateral_cost_functions:
            lat_cost += lateral_cf(lat_traj)
        for long_traj in longitudinal_trajectories:
            
            # calculate cost associated with lateral trajectory
            long_cost = 0
            for long_cf in longitudinal_cost_functions:
                long_cost += long_cf(long_traj)
                
            # add total cost and both trajectories to combined list
            total_cost = lat_cost + long_cost
            combined.append((total_cost, lat_traj, long_traj))
    
    # python will calculate min based on first element of each trajectory tuple (which is cost)     
    best = min(combined) 
    best_lat = best[1]
    best_long = best[2]
    
    return best_lat, best_long
    
def generate_trajectories(bounds, T, N=3):
    """
    Generate jerk minimized trajectories from start state (bounds[:3]) to
    end state (bounds[3:]) by perturbing the end state N times.
    
    Args:
      bounds:
        a length 6 array. If calculating a lateral trajectory these entries
        would be [s_i, s_i_dot, s_i_double_dot, s_f, s_f_dot, s_f_double_dot]
        where i means initial, f means final, and dots are derivatives wrt time.
        
      T:
        duration of maneuver
        
      N:
        number of trajectories to generate.
        
    Returns:
      N length 6 arrays, each of which correspond to the 6 coefficients in a
      5th degree polynomial.
    """
    # all bounds is an array of boundary conditios
    all_bounds =  [np.array(bounds)]
    
    # create N-1 perturbations of the goal state.
    new_goals = np.random.normal(bounds[3:], 0.5, (N-1,3))
    for g in new_goals:
        
        # the new bound keeps the original start state but uses new goal state
        new_bound = np.append(bounds[:3], g)
        all_bounds.append(new_bound)
    
    trajs = [MJT(b,T) for b in all_bounds]
    return trajs

def MJT(bounds, T):
    """
    Solver to calculate trajectory of duration T subject to boundary conditions.
    
    Args:
      bounds:
        Length 6 array. For a lateral trajectory the entries would correspond to:
        [s_i, s_i_dot, s_i_double_dot, s_f, s_f_dot, s_f_double_dot]
        
      T:
        Duration of maneuver
    
    Returns:
      A length 6 array which corresponds to the 6 coefficients in a
      5th degree polynomial.
    """
    b = np.array(bounds)
    A = np.array([
    [1, 0, 0, 0, 0,  0],
    [0, 1, 0, 0, 0,  0],
    [0, 0, 1, 0, 0,  0],
    [1, T, T**2, T**3, T**4, T**5],
    [0, 1, 2*T, 3*T**2, 4*T**3, 5*T**4],
    [0, 0, 2, 6*T, 12*T**2, 20*T**3],
        ])
    return np.linalg.solve(A,b)