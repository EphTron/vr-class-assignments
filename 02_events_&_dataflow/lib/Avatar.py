#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua


################################
## Avatar represents a monkey avatar in the scene to be controlled by a connected keyboard.
################################

class Avatar: 
  
    # constructor
    def __init__( self
                , PARENT_NODE = None
                ):


        ### resources ###

        _loader = avango.gua.nodes.TriMeshLoader() # init trimesh loader to load external meshes

        ## init avatar nodes
        self.avatar_geometry = _loader.create_geometry_from_file("avatar_geometry", "data/objects/monkey.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.avatar_geometry.Transform.value = avango.gua.make_scale_mat(0.03)
        self.avatar_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.2, 0.2, 1.0))

        self.avatar_transform = avango.gua.nodes.TransformNode(Name = "avatar_node")
        self.avatar_transform.Children.value = [self.avatar_geometry]
        PARENT_NODE.Children.value.append(self.avatar_transform)


 
