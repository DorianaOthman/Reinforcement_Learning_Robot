To launch gazebo use the following:
$ roslaunch project3 wallfollow.launch

To run training, use the following. The filename is what you want the name of the file with the saved q_table to be
$ rosrun project3 project3_code.py train <"filename">

To run testing for project3, use the following. The filename is the name of the file with your q_table data
$ rosrun project3 project3_code.py test <"filename">

To run testing for project4, use the following. The filename is the name of the file with your q_table data
$ rosrun project3 project4_code.py test <"filename">

