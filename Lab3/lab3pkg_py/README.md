#### $ cd ~/catkin_NETID

## Part 1 Coordinate Transformation
### Open First Terminal
#### $ catkin_make
#### $ source devel/setup.bash  
#### $ roslaunch ur3_driver vision_driver.launch  

### Open Second Terminal
#### $ source devel/setup.bash  
#### $ rosrun lab3pkg_py lab3_image_tf_exec.py  


## Part 2 Particle Filters
### Open First Terminal
#### $ catkin_make
#### $ source devel/setup.bash  
#### $ roslaunch ur3_driver vision_driver.launch  

### Open Second Terminal
#### $ source devel/setup.bash  
#### $ rosrun lab3pkg_py lab3_image_exec.py  

### Open Third Terminal
#### $ source devel/setup.bash  
#### $ rosrun lab3pkg_py lab3_move_exec.py

## OR

### Open First Terminal
#### $ catkin_make
#### $ source devel/setup.bash  
#### $ roslaunch ur3_driver vision_driver.launch  

### Open Second Terminal
#### $ source devel/setup.bash  
#### $ roslaunch lab3pkg_py lab3_exec.launch  

