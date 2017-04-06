# planning-project-draft

Some example files showing what the inputs to and output from the student's path planner might look like.

**`example_map.json`**

This is the most uncertain of the files here. Disregard for now.

**`example_sensor_fusion.json`**

An example of what the data that simulates sensor fusion output. Note that it includes the state of all nearby vehicles as well as current lane and the identity of any vehicles which are leading or following each vehicle (including on adjacent lanes).

**`example_localization.json`**

An example of the simulated data (noiseless) from localization.

**`example_output.json`**

An example of the student's expected output. Note that a `path` would normally have many more waypoints. This data will be used by a provided control module to steer the car.

