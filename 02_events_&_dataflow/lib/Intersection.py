#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script

### import python libraries
import math



class Intersection(avango.script.Script):

    ## input fields
    sf_pick_mat = avango.gua.SFMatrix4()

    ## output fields
    mf_pick_result = avango.gua.MFPickResult()


    ## constructor
    def __init__(self):
        self.super(Intersection).__init__()


    def my_constructor(self, SCENEGRAPH, SF_MAT, PICK_LENGTH, PICK_DIRECTION, WHITE_LIST = [], BLACK_LIST = []):

        ### external references ###
        self.SCENEGRAPH = SCENEGRAPH


        ### parameters ###
        
        self.white_list = WHITE_LIST
        self.black_list = BLACK_LIST

        self.pick_length = PICK_LENGTH
        self.pick_direction = PICK_DIRECTION
                

        ## @var pick_options
        # Picking options for the intersection process.        
        self.pick_options = "avango.gua.PickingOptions.GET_POSITIONS"
        self.pick_options += " | avango.gua.PickingOptions.GET_NORMALS"
        self.pick_options += " | avango.gua.PickingOptions.GET_WORLD_POSITIONS"
        self.pick_options += " | avango.gua.PickingOptions.GET_WORLD_NORMALS"
        #self.pick_options += " | avango.gua.PickingOptions.GET_TEXTURE_COORDS"
        self.pick_options += " | avango.gua.PickingOptions.PICK_ONLY_FIRST_OBJECT"

        self.pick_options = eval(self.pick_options)


        ### resources ###  
                
        self.ray = avango.gua.nodes.Ray()
  
  
        ## init field connections
        self.sf_pick_mat.connect_from(SF_MAT)
 
    

    ### callback functions ###

    def evaluate(self): # evaluated once every frame
        ## update ray parameters
        self.ray.Origin.value = self.sf_pick_mat.value.get_translate()
        self.ray.Direction.value = self.pick_direction * self.pick_length
    
        ## calc intersection
        _pick_result = self.SCENEGRAPH.ray_test(self.ray, self.pick_options, self.white_list, self.black_list)
        self.mf_pick_result.value = _pick_result.value

