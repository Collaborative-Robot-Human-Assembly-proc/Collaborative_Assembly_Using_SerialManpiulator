# Collaborative_Assembly_Using_SerialManpiulator 
the project provides service to assemble different product with the sole requirment of providing a CAD file for the desired assembly product.The assembly process can be done with safe human robot interaction.we use hardware components ABBirb120 and two Intel realsense cameras(D435i).

this repo will help you acheive the following : 
1) safe human robot interaction
2) mechanical parts detection
3) motion algorithms that make online path planning

# modification 
there are multiple modification in moveit plugins so make sure you use the moveit submodule existing in this parent repo as it is a forked version of this organization

# installation steps 
1) `git clone git@github.com:Collaborative-Robot-Human-Assembly-proc/Collaborative_Assembly_Using_SerialManpiulator.git`

2) `git submodule init`

3) `catkin_make_isolated`

4) echo "/home/<user-name>/Collaborative_Assembly_Using_SerialManipulator/devel/setup.bash" >> ~/.bashrc

this will get you enviroments ready to go but still more steps will be added on how to utilize the rest of the modules


# project  short video



https://github.com/Collaborative-Robot-Human-Assembly-proc/Collaborative_Assembly_Using_SerialManpiulator/assets/84791015/9828f55e-a640-41cc-aebb-80925a9eee7c




