#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua


### import application libraries
from lib.KeyboardInput import KeyboardInput
from lib.Hinge import Hinge
from lib.ArmSegment import ArmSegment
from lib.Hook import Hook


class Crane:
  
    # constructor
    def __init__( self
                , PARENT_NODE = None
                , TARGET_LIST = []
                ):



        ### resources ###

        ## init internal classes
        self.input = KeyboardInput()
        
        ## init base node for whole crane
        self.base_node = avango.gua.nodes.TransformNode(Name = "base_node")
        self.base_node.Transform.value = avango.gua.make_trans_mat(0.0,-0.1,0.0)
        PARENT_NODE.Children.value.append(self.base_node)
        
        
        ## ToDo: init first hinge && connect rotation input
        self.hinge_1 = Hinge()
        self.hinge_1.my_constructor( PARENT_NODE = self.base_node,
                                DIAMETER = 0.08,
                                HEIGHT = 0.02,
                                ROT_OFFSET_MAT = avango.gua.make_identity_mat(), # the rotation offset relative to the parent coordinate system
                                ROT_AXIS = avango.gua.Vec3(0,1,0), # the axis to rotate arround with the rotation input (default is head axis)
                                ROT_CONSTRAINT = [-180.0, 180.0]) # intervall with min and max rotation of this hinge
        self.hinge_1.sf_rot_value.connect_from(self.input.sf_rot_value0)
                      
                      
        

        ## ToDo: init first arm-segment
        self.arm_1 = ArmSegment( PARENT_NODE = self.hinge_1.get_node(),
                                 DIAMETER = 0.01,
                                 LENGTH = 0.1,
                                 ROT_OFFSET_MAT = avango.gua.make_identity_mat()) # the rotation offset relative to the parent coordinate system)




        ## ToDo: init second hinge && connect rotation input 
        rot_offset_mat = avango.gua.make_rot_mat(90.0,1.0,0.0,0.0)
        self.hinge_2 = Hinge()
        self.hinge_2.my_constructor( PARENT_NODE = self.arm_1.get_top_node(),
                                      DIAMETER = 0.03,
                                      HEIGHT = 0.012,
                                      ROT_OFFSET_MAT = rot_offset_mat, # the rotation offset relative to the parent coordinate system
                                      ROT_AXIS = avango.gua.Vec3(0,0,1), # the axis to rotate arround with the rotation input (default is head axis)
                                      ROT_CONSTRAINT = [0.0, 90.0]) # intervall with min and max rotation of this hinge
        self.hinge_2.sf_rot_value.connect_from(self.input.sf_rot_value1)


        ## ToDo: init second arm-segment
        self.arm_2 = ArmSegment( PARENT_NODE = self.hinge_2.get_node(),
                                 DIAMETER = 0.008,
                                 LENGTH = 0.05,
                                 ROT_OFFSET_MAT = avango.gua.make_identity_mat()) 

        ## ToDo: init third hinge && connect rotation input 
        self.hinge_3 = Hinge()
        self.hinge_3.my_constructor( PARENT_NODE = self.arm_2.get_top_node(),
                                     DIAMETER = 0.02,
                                     HEIGHT = 0.01,
                                     ROT_OFFSET_MAT = rot_offset_mat, # the rotation offset relative to the parent coordinate system
                                     ROT_AXIS = avango.gua.Vec3(0,0,1), # the axis to rotate arround with the rotation input (default is head axis)
                                     ROT_CONSTRAINT = [-90.0, 90.0]) # intervall with min and max rotation of this hinge
        self.hinge_3.sf_rot_value.connect_from(self.input.sf_rot_value2)

        ## ToDo: init third arm-segment
        self.arm_3 = ArmSegment( PARENT_NODE = self.hinge_3.get_node(),
                                 DIAMETER = 0.008,
                                 LENGTH = 0.05,
                                 ROT_OFFSET_MAT = avango.gua.make_identity_mat()) 


        ## ToDo: init hook
        self.hook = Hook()
        self.hook.my_constructor( PARENT_NODE = self.arm_3.get_top_node(),
                                  SIZE = 0.05,
                                  TARGET_LIST = [])
 
 
