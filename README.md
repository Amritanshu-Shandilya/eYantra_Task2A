# eYantra_Task2A
Submisson for Task 2A of eYantra Robotic Competition

### Problem Statement
[EYantra Themebook Page](https://portal.e-yantra.org/themeBook/hb/Task_2/task_2a_prob_statement.html)

#### Install the dependencies
```
pip3 install opencv-contrib-python==4.7.0.72
pip3 install numpy==1.21.5
```
On getting error like pip3 not found : 
```sudo apt install python3-pip```

### Steps to run the program
Clone the repository, `cd` into it and write these commands :
```
colcon build
source install/setup.bash
ros2 launch hb_task2a task2a.launch.py
```
You should see a Gazebo window looking like this :
![Starting screen](/gazebo.png)
