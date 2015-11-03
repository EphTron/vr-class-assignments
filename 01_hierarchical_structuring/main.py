#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua

### import application libraries
from lib.SimpleViewingSetup import *
from lib.SolarSystem import SolarSystem
from lib.Device import *
from lib.Navigation import SteeringNavigation


### global variables ###
#NAVIGATION_MODE = "Spacemouse"
#NAVIGATION_MODE = "New Spacemouse"
NAVIGATION_MODE = "Keyboard"


def start():

    ## create scenegraph
    scenegraph = avango.gua.nodes.SceneGraph(Name = "scenegraph")
    
    ## init solar system
    solar_system = SolarSystem()
    solar_system.my_constructor(scenegraph.Root.value)

    ## init navigation technique
    steering_navigation = SteeringNavigation()
    steering_navigation.set_start_transformation(avango.gua.make_trans_mat(0.0,0.1,0.3))
  
    if NAVIGATION_MODE == "Spacemouse":
        deviceInput = SpacemouseInput()
        deviceInput.my_constructor("gua-device-spacemouse")
        
        steering_navigation.my_constructor(deviceInput.mf_dof, deviceInput.mf_buttons, 1.0) # connect navigation with spacemouse input
        
    elif NAVIGATION_MODE == "New Spacemouse":
        deviceInput = SpacemouseInput()
        deviceInput.my_constructor("gua-device-spacemouse")
            
        steering_navigation.my_constructor(deviceInput.mf_dof, deviceInput.mf_buttons, 0.2) # connect navigation with spacemouse input

    elif NAVIGATION_MODE == "Keyboard":
        deviceInput = KeyboardInput()
        deviceInput.my_constructor("gua-device-keyboard0")

        steering_navigation.my_constructor(deviceInput.mf_dof, deviceInput.mf_buttons) # connect navigation with keyboard input

    else:    
        print("Error: NAVIGATION_MODE " + NAVIGATION_MODE + " is not known.")
        return


    # setup viewer
    viewingSetup = SimpleViewingSetup(scenegraph, "mono", False)
    #viewingSetup = SimpleViewingSetup(scenegraph, "anaglyph", False)
    viewingSetup.connect_navigation_matrix(steering_navigation.sf_nav_mat)
    steering_navigation.set_rotation_center_offset(viewingSetup.get_head_position())

    viewingSetup.run(locals(), globals())



### helper functions
def print_graph(root_node):
  stack = [(root_node, 0)]
  while stack:
    node, level = stack.pop()
    print("│   " * level + "├── {0} <{1}>".format(
      node.Name.value, node.__class__.__name__))
    stack.extend(
      [(child, level + 1) for child in reversed(node.Children.value)])


if __name__ == '__main__':
    start()

