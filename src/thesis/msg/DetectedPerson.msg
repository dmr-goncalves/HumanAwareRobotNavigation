# Message describing a detection of a person


# Unique id of the detection
string 			         name

# (Pseudo-)probabilistic value between 0.0 and 1.0 describing the detector's confidence in the detection
float64         	 	 confidence

# 3D pose (position + orientation) of the *center* of the detection
geometry_msgs/Point 	 position
geometry_msgs/Quaternion orientation

# Tracked person velocity
geometry_msgs/Point 	 velocity

# Workstation where the person is located
string                   workstation

# Workstation wheight
int8 					 weight
