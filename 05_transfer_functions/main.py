#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua

### import application libraries
from lib.SceneManager import *
from lib.SimpleViewingSetup import *
from lib.Device import *
from lib.Navigation import *
from lib.Manipulation import *

<<<<<<< HEAD
#SPACEMOUSE_MODE = "OLD"
=======
# SPACEMOUSE_MODE = "OLD"
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
SPACEMOUSE_MODE = "NEW"


## print the subgraph under a given node to the console
def print_graph(root_node):
  stack = [(root_node, 0)]
  while stack:
    node, level = stack.pop()
    print("│   " * level + "├── {0} <{1}>".format(
      node.Name.value, node.__class__.__name__))
    stack.extend(
      [(child, level + 1) for child in reversed(node.Children.value)])


def start():

  # create scenegraph
  scenegraph = avango.gua.nodes.SceneGraph(Name = "scenegraph")
  
  
  scene_manager = SceneManager(scenegraph.Root.value)

  '''
  # init input device
  gyromouse_device = MouseDevice()
  gyromouse_device.my_constructor("device-gyromouse")
  '''
  
  # init manipulation input devices
  if SPACEMOUSE_MODE == "OLD":
    spacemouse_input = SpacemouseInput1()
    spacemouse_input.my_constructor("device-spacemouse1")  

  elif SPACEMOUSE_MODE == "NEW":
    spacemouse_input = SpacemouseInput2()
    spacemouse_input.my_constructor("device-spacemouse2")

  mouse_input = MouseInput()
  mouse_input.my_constructor("device-mouse")
  
  # init navigation input device
  keyboard_input = KeyboardInput()
  keyboard_input.my_constructor("device-keyboard")


  # init navigation technique
  steering_navigation = SteeringNavigation()
  steering_navigation.my_constructor(keyboard_input.mf_dof, keyboard_input.mf_buttons) # connect navigation with keyboard input
  steering_navigation.set_start_transformation(avango.gua.make_trans_mat(0.0,0.05,0.3))


  # setup viewer
  viewer = SimpleViewingSetup(scenegraph, scenegraph.Root.value, "mono", False)
  #viewer = SimpleViewingSetup(scenegraph, "anaglyph", False)
  viewer.connect_navigation_matrix(steering_navigation.sf_nav_mat)
  steering_navigation.set_rotation_center_offset(viewer.get_head_position())

  # init manipulation techniques
  manipulation_manager = ManipulationManager()
  manipulation_manager.my_constructor(viewer.navigation_node, mouse_input, spacemouse_input)


  viewer.run(locals(), globals())

if __name__ == '__main__':
  start()

