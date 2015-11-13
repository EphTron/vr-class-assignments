#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua


### import application libraries
from lib.SimpleViewingSetup import SimpleViewingSetup
from lib.Scene import Scene
from lib.Crane import Crane


def start():

    ## create scenegraph
    scenegraph = avango.gua.nodes.SceneGraph(Name = "scenegraph")

    ## init viewing setup
    viewingSetup = SimpleViewingSetup(SCENEGRAPH = scenegraph, PARENT_NODE = scenegraph.Root.value, STEREO_MODE = "mono")
    #viewingSetup = SimpleViewingSetup(SCENEGRAPH = scenegraph, PARENT_NODE = scenegraph.Root.value, STEREO_MODE = "anaglyph")

    ## init scene
    scene = Scene(PARENT_NODE = scenegraph.Root.value)

    ## init crane
    crane = Crane(PARENT_NODE = scenegraph.Root.value, TARGET_LIST = scene.box_list)


    ## init field connections
    viewingSetup.viewer.DesiredFPS.connect_from(crane.input.sf_max_fps) # change viewer FPS during runtime


    print_graph(scenegraph.Root.value)

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

