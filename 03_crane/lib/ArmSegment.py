#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua


class ArmSegment:

    ### class variables ###

    # Number of ArmSegment instances that have already been created.
    number_of_instances = 0
    
  
    # constructor
    def __init__( self
                , PARENT_NODE = None
                , DIAMETER = 0.1
                , LENGTH = 0.1
                , ROT_OFFSET_MAT = avango.gua.make_identity_mat() # the rotation offset relative to the parent coordinate system
                ):

        ## get unique id for this instance
        self.id = ArmSegment.number_of_instances
        ArmSegment.number_of_instances += 1

        self.color = avango.gua.Vec4(1.0,0.84,0.00,1.0)
        ### resources ###
        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external tri-meshes
        
        ## ToDo: init arm node(s)
        self.arm_geometry = _loader.create_geometry_from_file("arm_segment_"+str(self.id), 
                                                              "data/objects/cylinder.obj",
                                                              avango.gua.LoaderFlags.DEFAULTS)
        self.arm_geometry.Transform.value = avango.gua.make_trans_mat(avango.gua.Vec3(0.0,LENGTH/2,0.0))*\
                                            ROT_OFFSET_MAT *\
                                            avango.gua.make_scale_mat(avango.gua.Vec3(DIAMETER,LENGTH,DIAMETER))
        self.arm_geometry.Material.value.set_uniform("Color", self.color)
        self.arm_geometry.Material.value.set_uniform("Emissivity", 0.6)
        self.arm_geometry.Material.value.set_uniform("Roughness", 0.40)

        self.arm_transform_bottom = avango.gua.nodes.TransformNode()
        self.arm_transform_top = avango.gua.nodes.TransformNode()
        self.arm_transform_top.Transform.value = avango.gua.make_trans_mat(0.0,LENGTH,0.0)

        PARENT_NODE.Children.value.append(self.arm_transform_bottom)
        self.arm_transform_bottom.Children.value = [self.arm_geometry, self.arm_transform_top]


        ### functions ###

    def get_node(self):
        return self.arm_transform_bottom

    def get_bottom_node(self):
        return self.arm_transform_bottom

    def get_top_node(self):
        return self.arm_transform_top



                
        
