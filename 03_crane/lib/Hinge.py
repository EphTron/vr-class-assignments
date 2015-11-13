#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed


class Hinge(avango.script.Script):

    ## input fields
    sf_rot_value = avango.SFFloat()

    ### class variables ###

    # Number of Hinge instances that have already been created.
    number_of_instances = 0
   
    # constructor
    def __init__(self):
        self.super(Hinge).__init__()

        ## get unique id for this instance
        self.id = Hinge.number_of_instances
        Hinge.number_of_instances += 1



    def my_constructor( self    
                      , PARENT_NODE = None
                      , DIAMETER = 0.1
                      , HEIGHT = 0.1
                      , ROT_OFFSET_MAT = avango.gua.make_identity_mat() # the rotation offset relative to the parent coordinate system
                      , ROT_AXIS = avango.gua.Vec3(0,1,0) # the axis to rotate arround with the rotation input (default is head axis)
                      , ROT_CONSTRAINT = [-45.0, 45.0] # intervall with min and max rotation of this hinge
                      ):


        ### variables ###
        self.rot_value = 0
        self.accumulation_rot_mat = avango.gua.make_identity_mat()

        ### parameters ###
        
        self.rot_axis = ROT_AXIS
        
        self.rot_constraint = ROT_CONSTRAINT

        self.color = avango.gua.Vec4(0.0,0.0,0.0,1.0)


        ### resources ###

        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external tri-meshes
        self.hinge_geometry = _loader.create_geometry_from_file("arm_segment_"+str(self.id), 
                                                                "data/objects/cylinder.obj",
                                                                avango.gua.LoaderFlags.DEFAULTS)        
        self.hinge_geometry.Transform.value = ROT_OFFSET_MAT *\
                                              avango.gua.make_scale_mat(avango.gua.Vec3(DIAMETER,HEIGHT,DIAMETER))
        self.hinge_geometry.Material.value.set_uniform("Color", self.color)
        self.hinge_geometry.Material.value.set_uniform("Emissivity", 0.0)
        self.hinge_geometry.Material.value.set_uniform("Roughness", 0.15)

        self.hinge_transform = avango.gua.nodes.TransformNode()

        PARENT_NODE.Children.value.append(self.hinge_transform)
        self.hinge_transform.Children.value.append(self.hinge_geometry)

    ### functions ###

    def get_node(self):
      return self.hinge_transform

    def get_geometry(self):
      return self.hinge_geometry
        
    ### callback functions ###
    
    @field_has_changed(sf_rot_value)
    def sf_rot_value_changed(self):
        self.rot_value += self.sf_rot_value.value
        
        if(self.rot_value > self.rot_constraint[1]):
            self.rot_value = self.rot_constraint[1]
        elif(self.rot_value < self.rot_constraint[0]):
            self.rot_value = self.rot_constraint[0]

        _temp_rot_mat = avango.gua.make_rot_mat(self.rot_value,self.rot_axis)
        self.hinge_transform.Transform.value = _temp_rot_mat
               
