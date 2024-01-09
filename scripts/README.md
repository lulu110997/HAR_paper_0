How to use scripts
```
python3 bag2csv.py name_bag # This will convert the skeletal data to a csv format
python3 read_bags_skel.py {option}  # This will display the recorded body and/or hand skeleton data. option: 'body', 'hand' or 'both'
```

read_bags_skel plots the joint skel to the image that is within 1/15 s of the skeleton headers. This is because the ts on the
PoseArray msgs for the skeleton poses are edited during the matching process. This ensures that all skeleton data are plotted.
**This threshold may need to be adjusted (in the script).**

obtain_*_skeleton.py are nodes which publish skeleton data from either nuitrack or mediapipe.  

The **hand skeleton** runs at **60hz**. However, due to the way mediapipe works, it is not guaranteed to have an output per
input image. Furthermore, since mediapipe requires a ts input in milliseconds, the
rospy.get_time() is rounded to milliseconds and then converted back to seconds before it is published. This may cause some
messages to have the same ts.  
The **body skeleton** runs at **30hz**. 