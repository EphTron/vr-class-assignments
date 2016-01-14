#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua


### import application libraries
from lib.SimpleViewingSetup import SimpleViewingSetup
from lib.Avatar import Avatar
from lib.Mover import Mover
from lib.Scene import Scene



def start():

    ## create scenegraph
    scenegraph = avango.gua.nodes.SceneGraph(Name = "scenegraph")

    ## init viewing setup
    viewingSetup = SimpleViewingSetup(SCENEGRAPH = scenegraph, PARENT_NODE = scenegraph.Root.value, STEREO_MODE = "mono")
    #viewingSetup = SimpleViewingSetup(SCENEGRAPH = scenegraph, PARENT_NODE = scenegraph.Root.value, STEREO_MODE = "anaglyph")

    ## init game avatar
    avatar = Avatar(PARENT_NODE = scenegraph.Root.value)

    ## init avatar movemer
    mover = Mover()
    mover.my_constructor(SCENEGRAPH = scenegraph, START_MATRIX = avango.gua.make_trans_mat(0.1, 0.14, 0.0), VIEWER = viewingSetup.viewer)

    ## init scene
    scene = Scene(PARENT_NODE = scenegraph.Root.value)

    ## init field connections 
    avatar.avatar_transform.Transform.connect_from(mover.sf_mat) # connect mover output matrix to avatar matrix
    viewingSetup.connect_navigation_matrix(avatar.avatar_transform.Transform) # connect avatar matrix to camera matrix
    viewingSetup.viewer.DesiredFPS.connect_from(mover.input.sf_max_fps) # change viewer FPS during runtime

    ## start application/render loop
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

