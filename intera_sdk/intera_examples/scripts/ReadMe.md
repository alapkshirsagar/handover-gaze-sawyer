Terminal 1:
roscore

Terminal 2:
rosrun rosserial_python serial_node.py /dev/ttyACM0

Terminal 3:
python src/handover_gaze_controller.py
