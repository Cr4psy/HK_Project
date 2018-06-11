Please keep this folder organized.

The ROS folder is used to contain, you guessed it, all the code that's gonna run on ROS. When you want to develop a new feature (say images fusion), create a corresponding package in the "src" folder (itself located in the "catkin_ws"). Your package folder should contain the following sub-folder :

1) msg if you have to define your own messages (if not, don't create it !)
2) srv folder if you have to define your own services (if none of the node of your package's gonna used services, please don't create this folder)
3) include folder to contain all header files (.h or .hpp)
4) src folder containing your code (.cpp files).


I've created as well a foler named "experimentation" if you want to try out some code without messing with ROS even though I would suggest NOT doing so
and directly write your code as a ROS node. The git branching system makes it easy to try out new features and not merge your code in the master branch if it doesn't work. So use the branching system extensively !

Please note that the catkin-ws root folder should contain four folders ("working spaces") : include, src, develop and build. However, the
develop and build folders are not saved in the remote repository as we've decided not to track them with git. 

