import Tkinter as tk
from collections import namedtuple, defaultdict
from datetime import datetime
import math
import time
import json
from constants import *

Track = namedtuple("Track", ["radius", "num_lanes", "lane_width"])

class Car(object):
    def __init__(self, s, d, speed, lane_num):
        self.s = s
        self.d = d
        self.speed = speed
        self.lane_num = lane_num
        
    def _advance(self, dt):
        self.s += self.speed * dt
        
    def _get_coords(self, track):
        r = track.radius + (self.lane_num + 0.5) * track.lane_width
        circum = 2.0 * math.pi * r
        theta = (2*math.pi * float(self.s % circum) / circum) % (2*math.pi)
        r = r + self.d
        _x = r * math.cos(theta) * -1
        _y = r * math.sin(theta) * -1
        x = _x + CANVAS_WIDTH / 2
        y = _y + CANVAS_HEIGHT  / 2
        return x,y
    
    def update(self, dt, track):
        old_x, old_y = self._get_coords(track)
        self._advance(dt)
        x, y = self._get_coords(track)
        dx = old_x - x
        dy = old_y - y
        return x, y
        

class Simulation(object):
    car_size = 6
    def __init__(self):
        self.root = tk.Tk()
        self.canvas = tk.Canvas(self.root, width=CANVAS_WIDTH, height = CANVAS_HEIGHT)
        self.canvas.pack()
        self.canvas.pack()
        self.cars = []
        self.animated_cars = []
        self.root.after(0, self.animation)
        self.output = {"vehicles" : {} }
        self.start_time = None
        self.current_time = 0.0
        self.current_frame = 0
        self.ego_data = None
        self.ego_id = None
        self.text_id = None
#         self.root.mainloop()

    def draw_track(self, track, w, h):
        diam = track.radius * 2
        top_left = (w - diam) / 2
        bot_right = w - top_left
        self.track = track
        for i in range(track.num_lanes + 1):
            self.canvas.create_oval(top_left-i*track.lane_width,
                               top_left-i*track.lane_width, 
                               bot_right+i*track.lane_width, 
                               bot_right+i*track.lane_width)

    def add_car(self, L, s, d, lane_num, ego=False):
        speed = SLOW
        if lane_num == 0:
            speed = FAST
        car = Car(s,d, speed, lane_num)
        self.cars.append(car)
        r = self.track.radius + (lane_num + 0.5) * self.track.lane_width

        circum = 2.0 * math.pi * r

        theta = 2*math.pi * (s % circum) / circum

        _x = r * math.cos(theta) * -1
        _y = r * math.sin(theta) * -1

        x = _x + CANVAS_WIDTH / 2
        y = _y + CANVAS_HEIGHT  / 2

        color = "red"
        if ego: color = "blue"

        car_id = (self.canvas.create_oval(x-L, y-L, x+L, y+L, outline="white", fill=color))

        if ego: 
            self.ego_id = car_id
        self.animated_cars.append(car_id)
        if self.current_frame == 0:
            self.output["vehicles"][car_id] = {
                "id" : car_id,
                "length" : L,
                "width": L,
                "predictions" : [{
                    "s" : s,
                    "d" : d,
                    "timestamp": self.current_time,
                    "lane" : lane_num
                }
                ]
            }
        else:
            print "ERROR"
        
    def get_r_c(self, lane_num):
        r = self.track.radius + (lane_num + 0.5) * self.track.lane_width
        c = 2.0 * math.pi * r
        return r,c
        
    def set_text(self, text):
        if not self.text_id:
            self.text_id = self.canvas.create_text(CANVAS_WIDTH / 2, CANVAS_HEIGHT / 2, {"text" : text})
        else:
            self.canvas.itemconfigure(self.text_id, text=text)
    def add_cars(self, L, lane_num, N):
        radius, circ = self.get_r_c(lane_num)
        for i in range(N):
            self.add_car(L, circ * float(i) / N, 0, lane_num)
         
    def animate(self, with_ego_data=None):
        if with_ego_data:
            self.ego_data = with_ego_data

        self.root.mainloop()

    def animation(self):
        track = 0
        dt = SECS_PER_LOOP
        print "calling animation"
        seen = False
        self.current_frame = 0
        self.start_time = 0.0
        L = self.car_size

        if self.ego_data:
            pred = self.ego_data["ego"]["predictions"][self.current_frame]
            s = pred["s"]
            d = pred["d"]
            speed = pred["v"]
            lane_num = pred["lane"]
            self.add_car(L, s, d, lane_num, ego=True)


        while self.current_frame < TOTAL_FRAMES:
            self.set_text("{}".format(self.current_frame))
            self.current_time = self.start_time + dt * self.current_frame
            self.current_frame += 1
            
            # if self.ego_data:
                

            for i, car in enumerate(self.cars):
                animated_car = self.animated_cars[i]
                if animated_car == self.ego_id:
                    pred = self.ego_data["ego"]["predictions"][self.current_frame]
                    car.s = pred["s"]
                    car.d = pred["d"]
                    car.speed = pred["v"]
                    car.lane_num = pred["lane"]
                    x,y = car._get_coords(self.track)

                else:
                    x, y = car.update(dt, self.track)

                    self.output["vehicles"][animated_car]["predictions"].append({
                        "s" : car.s,
                        "d" : car.d,
                        "timestamp" : self.current_time,
                        "lane" : car.lane_num
                    })
                self.canvas.coords(animated_car, x-L, y-L, x+L, y+L)

                self.canvas.update()
            seen = True
            time.sleep(dt / ANIMATION_SPEEDUP)

def make_animation(data=None):
    s = Simulation()
    t = Track(TRACK_RADIUS, NUM_LANES, LANE_WIDTH)
    s.draw_track(t, CANVAS_WIDTH, CANVAS_HEIGHT)
    s.add_cars(s.car_size, 0, CARS_PER_LANE[0])
    s.add_cars(s.car_size, 1, CARS_PER_LANE[1])
    s.animate(with_ego_data=data)
    with open(ACTOR_DATA_FILE_NAME, 'wb') as f:
        json.dump(s.output, f)  
