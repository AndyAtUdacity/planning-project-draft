from collections import namedtuple
from math import pi, sqrt, atan2
import math
from constants import *

Location = namedtuple( "Location", ["s",  "lane"])
Lane     = namedtuple("Lane",      ["id", "number"]) # 0 for number means closest to center yellow line
Goal     = Location
Pose     = namedtuple("Pose",      ["s",  "d", "lane"])
State    = namedtuple("State",     ["s",  "d", "lane", "yaw"])

class CircleMap(object):
    def __init__(self, radius, num_lanes, lane_width=4.0):
        self.r = radius
        self.num_lanes = num_lanes
        self.lane_width = lane_width
        self.speed_limit = SPEED_LIMIT# about 65 mph
        
    def get_curvature_at(self, s, d, lane_num):
        r = self.r + lane_num * self.lane_width + d
        return 1.0 / r
    
    def get_speed_limit(self, s, lane_num):
        return self.speed_limit
    
    def to_frenet(self, x, y):
        r = sqrt(x**2 + y**2)
        if r < self.radius:
            lane = 0
        elif r > self.radius * self.num_lanes * self.lane_width:
            lane = self.num_lanes
        else:
            best_lane_distance = 100000000
            best_lane = 0
            for i in range(self.num_lanes):
                ref_rad = self.r + (i + 0.5) * self.num_lanes
                diff = abs(r - ref_rad)
                if diff < best_lane_distance:
                    best_lane_distance = diff
                    best_lane = i
            lane = best_lane
        
        ref_rad = self.radius + (0.5 + lane) * self.lane_width
        d = r - ref_rad
        s = (ref_rad * theta) % (2*pi)
        return Pose(s, d, lane)
    
    def distance_between(self, pose1, pose2):
        """
        returns ds and dd:  driving distance between pose1 and pose2 if
        vehicle1 were to move a distance ds longitudinally in their frame followed 
        by movement of dd laterally
        """
        r1 = self.r + (0.5 + pose1.lane) * self.lane_width + pose1.d
        r2 = self.r + (0.5 + pose2.lane) * self.lane_width + pose2.d
        
        circ1 = r1 * 2 * pi
        circ2 = r2 * 2 * pi
        
        theta1 = pose1.s / circ1
        theta2 = pose2.s / circ2
        
        dtheta = (theta2 - theta1) % (2*pi)
        ds = r1 * dtheta
        dd = r2 - r1
        return ds, dd
        
class Vehicle(object):
    def __init__(self, L, W, max_turn=35.0 * 180.0 / pi, max_acc=9.8):
        self.L = L
        self.W = W
        self.max_turn = max_turn
        self.max_acc = MAX_ACC
        self.pose = None
        self.speed = None
        
    def set_pose(self, pose):
        self.pose = pose
        
    def set_velocity(self, v):
        # print "set v to {}".format(v)
        self.speed = v
        
    def set_lane(self, lane):
        self.pose = Pose(self.s, self.d, lane)
    
    @property
    def s(self): return self.pose.s
    
    @property
    def d(self): return self.pose.d
    
    @property
    def lane(self): return self.pose.lane

    @property 
    def lane_num(self): return self.pose.lane
    
    def _advance(self, dt):
        new_s = self.s + self.speed * dt
        self.pose = Pose(new_s, self.d, self.lane)
        
    def _get_coords(self, track):
        r = track.radius + (self.lane_num + 0.5) * track.lane_width
        circum = 2.0 * math.pi * r
#         print "circum is {} and s is {}".format(circum, self.s)
        theta = (2*math.pi * float(self.s % circum) / circum) % (2*math.pi)
        r = r + self.pose.d
        _x = r * math.cos(theta) * -1
        _y = r * math.sin(theta) * -1
        x = _x + CANVAS_WIDTH / 2
        y = _y + CANVAS_HEIGHT  / 2
        return x,y
    
    def update(self, dt, track):
        old_x, old_y = self._get_coords(track)
        self._advance(dt)
        x, y = self._get_coords(track)
        return x, y

PredictedPose = namedtuple("PredictedPose", ["pose", "timestamp"])

class PredictedVehicle(object):
    def __init__(self, data):
        self._id = data["id"]
        self.length = data["length"]
        self.width = data["width"]
        self.predictions = self.process_predictions(data["predictions"])
        
    def process_predictions(self, data):
        new_preds = []
        for p in data:
            pose = Pose(p['s'], p['d'], p['lane'])
            timestamp = p["timestamp"]
            new_preds.append(PredictedPose(pose, timestamp))
        return tuple(new_preds)

    @property
    def pose(self):
        return self.predictions[0].pose

    @property
    def s(self):
        return self.pose.s

    @property
    def d(self):
        return self.pose.d

    @property 
    def lane(self):
        return self.pose.lane

class Decider(object):
    STATES = [
        "ready_to_drive",
        "follow_lane",
        "lane_change_left",
        "lane_change_right",
        "track_gap_left", # used to prepare for a lane change left
        "track_gap_right"
    ]
    
    ALLOWED_TRANSITIONS = [
        [0, 1, 0, 0, 0, 0],
        [0, 1, 1, 1, 1, 1],
        [0, 1, 1, 0, 0, 0],
        [0, 1, 0, 1, 0, 0],
        [0, 1, 1, 0, 1, 0],
        [0, 1, 0, 1, 0, 1]
    ]
    
    def __init__(self, vehicle):
        self.ego = vehicle
        self.state = self.STATES[0]
        self.last_suggested_state = None
        self.available_predictions = {}
        self.goal = None
        self.map = None
    
    def set_map(self, M):
        self.map = M
    
    def set_goal(self, goal):
        self.goal = goal
    
    # should be run in background thread
    def _update_with_new_predictions(self, predictions):
        new_vehicles = {}
        for v_id, v in predictions["vehicles"].items():
            pv = PredictedVehicle(v)
            new_vehicles[pv._id] = pv
        self.available_predictions = new_vehicles
    
    def update_with_new_localization(self, localization):
        x = localization["x"]
        y = localization["y"]
        
        pose = self.map.to_frenet(x,y)
        self.ego.set_pose(pose)
        
    def get_next_states(self):
        idx = self.STATES.index(self.state)
        return [s for s,allowed in zip(self.STATES, self.ALLOWED_TRANSITIONS[idx]) if allowed]
    
    def get_behavior(self):
        possible_next_states = self.get_next_states()

        costs = []
        for state in possible_next_states:
            cost = self.calculate_cost(state)
            costs.append((cost,state))
        _, next_state = min(costs)
        # print
        # for cost, state in costs:
        #     print "{}:\t{}".format(state,cost)
        # print
        self.state = next_state
        return next_state

    def calculate_cost(self, state):
        # cost functions generally designed to output numbers between -1 and 1
        # unless dangerous or illegal behavior
        cost_functions = [
            self.lane_change_cost,
            self.velocity_cost,
            self.gap_size_cost
        ]
        weights = [
            LANE_CHANGE_WEIGHT,
            VELOCITY_WEIGHT,
            GAP_SIZE_WEIGHT
        ]
        cost = 0
        for cost_func, weight in zip(cost_functions, weights):
            cost += weight * cost_func(state)
        return cost

    def lane_change_cost(self, state):
        if state not in ["lane_change_left", "lane_change_right"]:
            return 0
        diffs = {"lane_change_left":-1, "lane_change_right":+1}
        next_lane = self.ego.lane + diffs[state]
        # if next_lane >= self.map.num_lanes or next_lane < 0:
        #     return DANGEROUS_COST
        if abs(self.goal.lane - next_lane) < abs(self.goal.lane - self.ego.lane):
            # next_lane is closer to goal than current lane
            return -1
        else:
            return 1

    def velocity_cost(self, state):
        speed_limit = self.map.get_speed_limit(self.ego.s, self.ego.lane)
        velocity_for_state = self.get_velocity_for_state(state)

        # print
        # print "  v     - {}".format(velocity_for_state)
        # print "  state - {}".format(state)

        # print "velocity for state {}".format(velocity_for_state)
        if velocity_for_state > speed_limit: 
            return ILLEGAL_COST
        return (speed_limit - velocity_for_state) / speed_limit # between 0 and 1
    
    def get_velocity_for_state(self, state):
        if state == "ready_to_drive": return 0
        SL = self.map.get_speed_limit(self.ego.s, self.ego.lane)
        leading = None
        if state == "follow_lane": 
            leading = self.get_leading_vehicle()
            if not leading:
                return SL
            dist = self.map.distance_between(self.ego.pose, leading.pose)[0] 
            # print "dist is {}\nego l is   {}".format(dist,self.ego.L)
            if dist > (DESIRED_CAR_LENGTH_SEPARATION * self.ego.L):
                return SL
            else:
                return min(self.velocity_for(leading), SL)
        if state.startswith("lane_change"):
            diffs = {"lane_change_left":-1, "lane_change_right":+1}
            other_lane = self.ego.lane + diffs[state]
            leading = self.get_leading_vehicle(in_lane=other_lane)
            if not leading:
                return SL
            if self.map.distance_between(self.ego.pose, leading.pose)[0] > (DESIRED_CAR_LENGTH_SEPARATION * self.ego.L):
                return SL
            else:
                return min(self.velocity_for(leading), SL)
        if state.startswith("track_gap"):
            diffs = {"track_gap_left":-1, "track_gap_right":+1}
            other_lane = self.ego.lane + diffs[state]
            leading = self.get_leading_vehicle(in_lane=other_lane)
            if not leading:
                return SL
            if self.map.distance_between(self.ego.pose, leading.pose)[0] > (DESIRED_CAR_LENGTH_SEPARATION * self.ego.L):
                return SL
            else:
                return min(self.velocity_for(leading), SL)
        print "why am i here"
        return SL
    
    def get_leading_vehicle(self, in_lane=None):
        if in_lane is None:
            in_lane = self.ego.lane
        possibilities = []
        for v_id, v in self.available_predictions.items():
            pp = v.predictions[0][0]
            s, d = self.map.distance_between(self.ego.pose, pp)
            if s > 0:
                possibilities.append((s, v.predictions[0]))
        if len(possibilities) == 0: 
            return None 
        else:
            dist, leading = min(possibilities, key=lambda x: x[0])
            # print "got leading, distance is {}".format(dist)
            return leading

    def get_trailing_vehicle(self, in_lane=None):

        if in_lane is None:
            in_lane = self.ego.lane
        possibilities = []
        for v_id, v in self.available_predictions.items():
            pp = v.predictions[0][0]
            s, d = self.map.distance_between(self.ego.pose, pp)
            if -s > 0:
                possibilities.append((-s, v.predictions[0]))
        if len(possibilities) == 0: 
            return None 
        else:
            dist, trailing = min(possibilities, key=lambda x: x[0])
            print "got trailing, distance is {}".format(dist)
            return trailing
    
    def velocity_for(self, predicted_vehicle, in_secs=0):
        if predicted_vehicle.pose.lane == 0: return FAST
        return SLOW
        pass

    def gap_size_cost(self, state):
        if state.startswith("track_gap"): return 0
        if state not in ["lane_change_left", "lane_change_right"]:
            return 0
        diffs = {"lane_change_left":-1, "lane_change_right":+1}
        other_lane = self.ego.lane + diffs[state]
        leading = self.get_leading_vehicle(other_lane)
        trailing = self.get_trailing_vehicle(other_lane)
        if not leading and not trailing:
            return 0

        if leading and trailing:
            gap = (leading.s - trailing.s)
            if gap < 0:
                print "ERROR GAP < 0"
            center = (leading.s + trailing.s) / 2.0
            c_s, c_d = self.map.distance_between(self.pose, Pose(center, 0, other_lane))
            room = gap - abs(2 * c_s)
            if room < 0 :
                print "ERROR BUFF < 0"
        elif leading:
            c_s, c_d = self.map.distance_between(self.ego.pose, leading.pose)
            room = c_s
        elif trailing:
            c_s, c_d = self.map.distance_between(self.ego.pose, trailing.pose)
            room = -c_s

        if room < self.ego.L:
            return DANGEROUS_COST

        cost = (1.0 - (room / (DESIRED_CAR_LENGTH_SEPARATION * self.ego.L )))
        truncated = max(cost, 0.0)
        # print "cost for state {} is {}. Normalized: {}".format(state,cost,truncated)
        return truncated




        return 0

        # next evaluate the cost associated with all the available transitions

        # COSTS INCLUDE: 
        # 1. Does this make progress to the goal 
        #    in terms of number of lane changes remaining. 
        #    Idea is to penalize lane changes if they aren't necessary.
        # 2. Velocity: We want to drive as close to the speed 
        #    limit as possible. (NOTE: Include speed limit in map)
        # 3. Gap size: when making lane changes we prefer to aim for 
        #    larger gaps than smaller ones.
        # 4. If you wanted to go further you might need a  