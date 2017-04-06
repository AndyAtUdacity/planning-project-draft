# planning-project-draft

Some example files showing what the inputs to and output from the student's path planner might look like.

**`example_map.json`**

This is one implementation of a map based partially on the [Road Network Description File](https://www.grandchallenge.org/grandchallenge/docs/RNDF_MDF_Formats_031407.pdf) format used in the DARPA urban challenge. This map is a cyclical one with 150 "segments" of 4 lane highway.

This is the most uncertain of the files here. Feedback is much appreciated.

**`example_sensor_fusion.json`**

An example of the data that simulates sensor fusion output. Note that it includes the state of all nearby vehicles as well as current lane and the identity of any vehicles which are leading or following each vehicle (including on adjacent lanes).

I wasn't entirely sure how to handle the leading / following data so feedback there will be especially helpful.

**`example_localization.json`**

An example of the simulated data (noiseless) from localization.

**`example_output.json`**

An example of the student's expected output. Note that a `path` would normally have many more waypoints. This data will be used by a provided control module to steer the car. A real path would have many more points.

