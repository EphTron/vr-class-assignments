#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

# import application libraries
from lib.KeyboardInput import KeyboardInput
from lib.Intersection import Intersection

# import python libraries
import time


class Mover(avango.script.Script):
  
    ## output field
    sf_mat = avango.gua.SFMatrix4()


    ## constructor
    def __init__(self):
        self.super(Mover).__init__()


    def my_constructor( self 
                      , SCENEGRAPH = None
                      , START_MATRIX = avango.gua.make_identity_mat()
                      , VIEWER = None
                      ):


        ## init internal classes
        self.input = KeyboardInput()
        self.input.my_constructor(VIEWER = VIEWER)

        self.accumulator = Accumulator()
        self.accumulator.my_constructor(START_MATRIX = START_MATRIX)

        self.groundFollowing = GroundFollowing()
        self.groundFollowing.my_constructor(SCENEGRAPH = SCENEGRAPH, START_MATRIX = START_MATRIX)


        ## init field connections
        self.accumulator.sf_move_vec.connect_from(self.input.sf_move_vec) # connect input into accumulator
        
        self.sf_mat.connect_from(self.accumulator.sf_mat) # connect output of accumulator to mover output matrix
        

        # ToDo: integrate the groundFollowing into the dataflow
        # evtl. replace existing field connections with new one
        # pay attention to potential loops in the dataflow --> use weak field connections to prevent loops
        # ...
        


################################
## Movement accumulator takes the (relative) movement vector of a KeyboardInput instance and applies
## it to an accumulated (absolute) matrix.
################################

class Accumulator(avango.script.Script):

    ## input fields
    sf_move_vec = avango.gua.SFVec3()

    ## output field
    sf_mat = avango.gua.SFMatrix4()
  
  
    ## constructor
    def __init__(self):
        self.super(Accumulator).__init__()


    def my_constructor( self
                      , START_MATRIX = avango.gua.make_identity_mat()
                      ):

        ## set initial state                    
        self.sf_mat.value = START_MATRIX


    ###  callback functions ###
    def evaluate(self): # evaluated once every frame if any input field has changed
        print("movement input", self.sf_move_vec.value)

        # ToDO: accumulate movement input to output matrix        
        # self.sf_mat.value = ...
        
        pass



################################
## GroundFollowing takes a matrix from an MovementAccumulator instance and corrects it with
## respect to gravity in the scene. The result can then be applied to the avatar's transformation node.
################################

class GroundFollowing(avango.script.Script):

    ## input fields
    sf_mat = avango.gua.SFMatrix4()

    ## ouput fields
    sf_modified_mat = avango.gua.SFMatrix4()

    ## internal fields
    mf_pick_result = avango.gua.MFPickResult()


    ## constructor
    def __init__(self):
        self.super(GroundFollowing).__init__()


    def my_constructor( self
                      , SCENEGRAPH = None
                      , START_MATRIX = avango.gua.make_identity_mat()
                      ):

        ### parameters ###
        self.fall_velocity = 0.004 # in meter/sec

        self.pick_length = 1.0 # in meter
        self.pick_direction = avango.gua.Vec3(0.0,-1.0,0.0)


        ### variables ###
        self.fall_vec = avango.gua.Vec3(0.0, 0.0, 0.0)

        self.sav_time = time.time()


        ## init internal classes
        self.gravity_intersection = Intersection()
        self.gravity_intersection.my_constructor(SCENEGRAPH, self.sf_mat, self.pick_length, self.pick_direction)

        ## init field connections
        self.mf_pick_result.connect_from(self.gravity_intersection.mf_pick_result)

        ## set initial state  
        self.sf_modified_mat.value = START_MATRIX
        

    ###  callback functions ###
    def evaluate(self): # evaluated once every frame if any input field has changed
    
        ## compute gravity response
        #print(len(self.mf_pick_result.value))
        if len(self.mf_pick_result.value) > 0: # intersection found
            _pick_result = self.mf_pick_result.value[0] # get first intersection target from list

            _distance = _pick_result.Distance.value # distance from ray matrix to intersection point
            _distance -= 0.025 # subtract half avatar height from distance
            _distance -= self.fall_velocity

            if _distance > 0.01: # avatar above ground
                self.fall_vec = avango.gua.Vec3(0.0, self.fall_velocity * -1.0, 0.0)

            else: # avatar (almost) on ground
                self.fall_vec.y = 0.0
            
            #print(self.fall_vec)

            self.sf_modified_mat.value = avango.gua.make_trans_mat(self.fall_vec) * \
                                         self.sf_mat.value
            
        else: # no intersection found
            self.sf_modified_mat.value = self.sf_mat.value # no changes needed


            
