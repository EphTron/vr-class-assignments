#!/usr/bin/python

### import guacamole libraries ###
import avango
import avango.gua

### import framework libraries ###
from lib.ViewingSetup import ViewingSetup
from lib.Device import *
from lib.Navigation import SteeringNavigation
from lib.Scene import SceneManager


### global variables ###
NAVIGATION_MODE = "Spacemouse"
#NAVIGATION_MODE = "New Spacemouse"
#NAVIGATION_MODE = "Keyboard"


def start():

    ## create scenegraph
    scenegraph = avango.gua.nodes.SceneGraph(Name = "scenegraph")

    # init scene
    sceneManager = SceneManager()
    sceneManager.my_constructor(PARENT_NODE = scenegraph.Root.value)


    ## init viewing setup
    hostname = open('/etc/hostname', 'r').readline()
    hostname = hostname.strip(" \n")

    
    if hostname == "eris": # Mitsubishi 3D-TV workstation
        viewingSetup = ViewingSetup( SCENEGRAPH = scenegraph
                                   , SCREEN_RESOLUTION = avango.gua.Vec2ui(1920, 1080) # in pixels
                                   , SCREEN_DIMENSIONS = avango.gua.Vec2(1.445, 0.81) # in meter
                                   #, SCREEN_MATRIX = avango.gua.make_identity_mat()
                                   , STEREO_FLAG = True
                                   , STEREO_MODE = avango.gua.StereoMode.CHECKERBOARD
                                   , HEADTRACKING_FLAG = True
                                   , HEADTRACKING_STATION = "tracking-art-glasses-1" # wired 3D-TV glasses on boreas workstation
                                   , TRACKING_TRANSMITTER_OFFSET = avango.gua.make_trans_mat(-1.0, -(0.58 + 0.975), 0.26 + 3.48) * avango.gua.make_rot_mat(90.0,0,1,0) # transformation into tracking coordinate system
                                   )

        pointer_station_name = "device-pointer-1"
        pointer_tracking_name = "tracking-art-pointer-1"
        transmitter_offset = avango.gua.make_trans_mat(-1.0, -(0.58 + 0.975), 0.26 + 3.48) * avango.gua.make_rot_mat(90.0,0,1,0) # transformation into tracking coordinate system
                                                 
                                                 
    elif hostname == "arachne": # Samsung 3D-TV workstation
        viewingSetup = ViewingSetup( SCENEGRAPH = scenegraph
                                   , SCREEN_RESOLUTION = avango.gua.Vec2ui(1920, 1080) # in pixels
                                   , SCREEN_DIMENSIONS = avango.gua.Vec2(1.235, 0.7) # in meter
                                   , SCREEN_MATRIX = avango.gua.make_rot_mat(90.0,0,0,1)
                                   , STEREO_FLAG = True
                                   , STEREO_MODE = avango.gua.StereoMode.CHECKERBOARD
                                   , HEADTRACKING_FLAG = True
                                   , HEADTRACKING_STATION = "tracking-art-glasses-2" # wired 3D-TV glasses on arachne workstation
                                   , TRACKING_TRANSMITTER_OFFSET = avango.gua.make_trans_mat(0.48, -(0.64 + 0.975), 0.48 + 3.48) * avango.gua.make_rot_mat(90.0,0,1,0) # transformation into tracking coordinate system
                                   )

        pointer_station_name = "device-pointer-2"
        pointer_tracking_name = "tracking-art-pointer-2"
        transmitter_offset = avango.gua.make_trans_mat(0.48, -(0.64 + 0.975), 0.48 + 3.48) * avango.gua.make_rot_mat(90.0,0,1,0) # transformation into tracking coordinate system
                                                 

                                              
    elif hostname == "pan": # Asus 3D-Monitor workstation 
        viewingSetup = ViewingSetup( SCENEGRAPH = scenegraph
                                   , SCREEN_RESOLUTION = avango.gua.Vec2ui(1920, 1080) # in pixels
                                   , SCREEN_DIMENSIONS = avango.gua.Vec2(0.535, 0.305) # in meter                            
                                   #, SCREEN_MATRIX = avango.gua.make_identity_mat()
                                   , STEREO_FLAG = True
                                   , STEREO_MODE = avango.gua.StereoMode.ANAGLYPH_RED_CYAN
                                   , HEADTRACKING_FLAG = True
                                   , HEADTRACKING_STATION = "tracking-pst-glasses-1" # glasses on pan workstation
                                   , TRACKING_TRANSMITTER_OFFSET = avango.gua.make_trans_mat(0.0,-0.11,0.77) # transformation into tracking coordinate system
                                   )

        pointer_station_name = "device-gyromouse"
        pointer_tracking_name = "tracking-pst-pointer-1"
        transmitter_offset = avango.gua.make_trans_mat(0.0,-0.11,0.77) # transformation into tracking coordinate system                                   

    elif hostname == "agenor": # DELL-monitor workstation with PST tracker
        viewingSetup = ViewingSetup( SCENEGRAPH = scenegraph
                                   , SCREEN_RESOLUTION = avango.gua.Vec2ui(2560, 1440) # in pixels
                                   , SCREEN_DIMENSIONS = avango.gua.Vec2(0.595,0.335) # in meter                            
                                   #, SCREEN_MATRIX = avango.gua.make_identity_mat()
                                   , STEREO_FLAG = True
                                   , STEREO_MODE = avango.gua.StereoMode.ANAGLYPH_RED_CYAN
                                   , HEADTRACKING_FLAG = True
                                   , HEADTRACKING_STATION = "tracking-pst-glasses-1" # glasses on pan workstation
                                   , TRACKING_TRANSMITTER_OFFSET = avango.gua.make_trans_mat(0.0,-0.12,0.71) # transformation into tracking coordinate system
                                   )

        pointer_station_name = "device-pointer-1"
        pointer_tracking_name = "tracking-pst-pointer-1"
        transmitter_offset =  avango.gua.make_trans_mat(0.0,-0.12,0.71) # transformation into tracking coordinate system


    else: # other desktop setups
        viewingSetup = ViewingSetup( SCENEGRAPH = scenegraph       
                                   , SCREEN_RESOLUTION = avango.gua.Vec2ui(2560, 1440) # in pixels
                                   , SCREEN_DIMENSIONS = avango.gua.Vec2(0.595, 0.32) # in meter                            
                                   #, SCREEN_MATRIX = avango.gua.make_identity_mat()
                                   )

        pointer_station_name = ""
        pointer_tracking_name = ""
        transmitter_offset = avango.gua.make_identity_mat() # transformation into tracking coordinate system


    ## init navigation technique
    if NAVIGATION_MODE == "Spacemouse":
        deviceInput = SpacemouseInput()
        deviceInput.my_constructor("device-old-spacemouse"
                                  , TRANSLATION_FACTOR = 0.6
                                  , ROTATION_FACTOR = 0.4
                                  )
        
    elif NAVIGATION_MODE == "New Spacemouse":
        deviceInput = NewSpacemouseInput()
        deviceInput.my_constructor( DEVICE_STATION = "device-new-spacemouse"
                                  , TRANSLATION_FACTOR = 0.6
                                  , ROTATION_FACTOR = 0.4
                                  )

    elif NAVIGATION_MODE == "Keyboard":
        deviceInput = KeyboardInput()
        deviceInput.my_constructor("device-keyboard")

    else:    
        print("Error: NAVIGATION_MODE " + NAVIGATION_MODE + " is not known.")
        return


    steering_navigation = SteeringNavigation()
    steering_navigation.my_constructor( SCENEGRAPH = scenegraph
                                      , PARENT_NODE = viewingSetup.get_navigation_node()
                                      , MF_NAV_DOF = deviceInput.mf_dof
                                      , POINTER_STATION_NAME = pointer_station_name
                                      , POINTER_TRACKING_NAME = pointer_tracking_name
                                      , TRACKING_TRANSMITTER_OFFSET = transmitter_offset
                                      )
    steering_navigation.sf_nav_mat.value = avango.gua.make_trans_mat(0.0,0.5,3.0) # initial navigation matrix
    steering_navigation.sf_head_mat.connect_from(viewingSetup.head_node.Transform) # connect headtracking matrix into navigation class


    viewingSetup.connect_navigation_matrix(steering_navigation.sf_nav_mat)
    viewingSetup.run(locals(), globals())


### helper functions ###

## print the subgraph under a given node to the console
def print_graph(root_node):
  stack = [(root_node, 0)]
  while stack:
    node, level = stack.pop()
    print("│   " * level + "├── {0} <{1}>".format(
      node.Name.value, node.__class__.__name__))
    stack.extend(
      [(child, level + 1) for child in reversed(node.Children.value)])


## print all fields of a fieldcontainer to the console
def print_fields(node, print_values = False):
  for i in range(node.get_num_fields()):
    field = node.get_field(i)
    print("→ {0} <{1}>".format(field._get_name(), field.__class__.__name__))
    if print_values:
      print("  with value '{0}'".format(field.value))


if __name__ == '__main__':
    start()

