std_msgs/Header header
string id                                    #name of identifying the object (milk, orange juice, etc) or color (green,blue, etc)
float32 confidence                           #value in [0,1] indicating the probability of a correct identification
                                             #in the case of color objects indiates the grasping priority
geometry_msgs/Vector3 size                   #Size in meters: size in x, y and z
geometry_msgs/Pose pose                      #Centroid and orientation
geometry_msgs/Vector3[] bounding_box         #Two vectors indicating the bounding box
int32 x					     #top left x
int32 y					     #top left y
int32 width				     #top widht
int32 height				     #top height
