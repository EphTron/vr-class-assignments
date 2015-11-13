#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed


class Hook(avango.script.Script):

    ## internal fields
    sf_mat = avango.gua.SFMatrix4()
 
    # constructor
    def __init__(self):
        self.super(Hook).__init__()


    def my_constructor( self    
                      , PARENT_NODE = None
                      , SIZE = 0.1
                      , TARGET_LIST = []
                      ):

        ### variables ###
        self.color = avango.gua.Vec4(0.0,0.0,0.0,1.0)

        ### external resources ###
        self.TARGET_LIST = TARGET_LIST

        ### resources ###
        
        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external tri-meshes

        ## ToDo: init hook node(s)
        self.hook_geometry = _loader.create_geometry_from_file("hooker", 
                                                               "data/objects/Hooker.obj",
                                                               avango.gua.LoaderFlags.DEFAULTS)
        _hand_trans_mat = avango.gua.make_trans_mat(-0.0086,0.0075,0.0)
        _hand_rot_mat_1 = avango.gua.make_rot_mat(90.0,0.0,1.0,0.0)
        _hand_rot_mat_2 = avango.gua.make_rot_mat(180.0,1.0,0.0,0.0)
        
        self.hook_geometry.Transform.value = _hand_trans_mat * _hand_rot_mat_1 * _hand_rot_mat_2 *\
                                             avango.gua.make_scale_mat(avango.gua.Vec3(SIZE,SIZE,SIZE))
        self.hook_geometry.Material.value.set_uniform("Color", self.color)
        self.hook_geometry.Material.value.set_uniform("Emissivity", 0.0)
        self.hook_geometry.Material.value.set_uniform("Roughness", 0.15)
        self.hook_geometry.Material.value.EnableBackfaceCulling.value = False

        self.hook_transform = avango.gua.nodes.TransformNode()

        PARENT_NODE.Children.value.append(self.hook_transform)
        self.hook_transform.Children.value.append(self.hook_geometry)


        ## ToDo: init field connections
        # ...


    ### callback functions ###
    
    @field_has_changed(sf_mat)
    def sf_mat_changed(self):
        _pos = self.sf_mat.value.get_translate() # world position of hook
        
        for _node in self.TARGET_LIST: # iterate over all target nodes
            _bb = _node.BoundingBox.value # get bounding box of a node
            #print(_node.Name.value, _bb.contains(_pos))
            
            if _bb.contains(_pos) == True: # hook position inside bounding box of this node
                _node.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0)) # highlight color
            else:
                _node.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,1.0,1.0,1.0)) # default color

       
