# Robot Eye Gazes in Human-to-Robot Handovers
We implement and compare three gaze behaviors on a Sawyer cobot for human-to-robot handovers based on insights from human-human handovers. 

Paper: T. Faibish*, A. Kshirsagar*, G. Hoffman and Y. Edan. “Human Preferences for Robot Eye Gaze in Human-to-Robot Handovers.” International Journal of Social Robotics, 2022
(https://alapkshirsagar.github.io/papers/handoversgaze-ijsr)


## Running Code
### Terminal 1:
```
roscore
```

### Terminal 2:
```
rosrun rosserial_python serial_node.py /dev/ttyACM0
```

### Terminal 3:
```
python src/tair_files/fixed_head.py
```

### Terminal 4:
```
python src/tair_files/eye_gaze.py <em>gaze_number</em>
```

### Terminal 5:
```
python src/tair_files/handover_gaze_controller.py
```
