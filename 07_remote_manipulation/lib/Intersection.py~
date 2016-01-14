#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua

# import framework libraries
# ...

# import python libraries
# ...



class Intersection:

    # constructor
    def __init__( self
                , SCENEGRAPH = None
                ):

                    
        ### external references ###
        self.SCENEGRAPH = SCENEGRAPH
      
        ### parameters ###
        self.white_list = []   
        self.black_list = ["invisible"]
        
            
        ### resources ###
        self.ray = avango.gua.nodes.Ray()
      
        self.pick_options = avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT \
                            | avango.gua.PickingOptions.GET_POSITIONS \
                            | avango.gua.PickingOptions.GET_NORMALS \
                            | avango.gua.PickingOptions.GET_WORLD_POSITIONS \
                            | avango.gua.PickingOptions.GET_WORLD_NORMALS
      
        
    
    ### functions ###

    def calc_pick_result(self, PICK_MAT = avango.gua.make_identity_mat(), PICK_LENGTH = 10.0, PICK_DIRECTION = avango.gua.Vec3(0.0,0.0,-1.0)):

        # update ray parameters
        self.ray.Origin.value = PICK_MAT.get_translate()

        _vec = avango.gua.make_rot_mat(PICK_MAT.get_rotate_scale_corrected()) * PICK_DIRECTION
        _vec = avango.gua.Vec3(_vec.x,_vec.y,_vec.z)

        self.ray.Direction.value = _vec * PICK_LENGTH

        # intersect
        _mf_pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)

        return _mf_pick_result

        
