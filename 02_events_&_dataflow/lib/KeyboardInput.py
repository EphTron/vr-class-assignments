#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon


# import python libraries
import time



################################
## KeyboardInput gets events from a connected keyboard and maps them to a movement vector for the avatar.
## Furthermore, it modifies this movement vector when a jump action is triggered by a keyboard event.
################################

class KeyboardInput(avango.script.Script):

    ## input fields
    sf_move_left = avango.SFBool()
    sf_move_right = avango.SFBool()
    sf_jump = avango.SFBool()
    sf_fps_toogle = avango.SFBool()

    ## output fields
    sf_move_vec = avango.gua.SFVec3()
    sf_max_fps = avango.SFFloat()
    sf_max_fps.value = 60.0 # initial value

    ## constructor
    def __init__(self):
        self.super(KeyboardInput).__init__()


    def my_constructor( self
                      , VIEWER = None
                      ):
        

        ### external references ###
        self.VIEWER = VIEWER

        ### parameters ###
        self.jump_duration = 1.0 # in sec
        self.jump_velocity = 0.006 # in meter per frame
        self.move_velocity = 0.003 # in meter/sec

        ### variables ###
        self.input_x = 0.0
        self.input_y = 0.0
        
        self.jump_flag = False
        self.jump_start_time = None

        self.sav_time = time.time()

        ### resources ###
        
        ## init sensor
        self.keyboard_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.keyboard_sensor.Station.value = "device-keyboard"

        ## init field connections
        self.sf_move_left.connect_from(self.keyboard_sensor.Button8) # left arrow key
        self.sf_move_right.connect_from(self.keyboard_sensor.Button9) # right arrow key
        self.sf_jump.connect_from(self.keyboard_sensor.Button18) # space key
        self.sf_fps_toogle.connect_from(self.keyboard_sensor.Button19) # left ctrl key

        # ToDo: change global evaluate policy here
        # ...
    
    
    ### callback functions ###     

    @field_has_changed(sf_fps_toogle)
    def sf_fps_toogle_changed(self):
    
        if self.sf_fps_toogle.value == True: # key pressed
            
            if self.sf_max_fps.value == 60.0:
                self.sf_max_fps.value = 20.0 # set slow application/render framerate
                print("slow app:", self.sf_max_fps.value)
            else:
                self.sf_max_fps.value = 60.0 # set fast application/render framerate
                print("fast app:", self.sf_max_fps.value)


    @field_has_changed(sf_jump)
    def sf_jump_changed(self):
    
        if self.sf_jump.value == True: # key pressed
            print("jump started")
            self.jump_flag = True
      
            self.jump_start_time = time.time()


    @field_has_changed(sf_move_left)
    def sf_move_left_changed(self):
        print("key left", self.sf_move_left.value)


    @field_has_changed(sf_move_right)
    def sf_move_right_changed(self):
        print("key right", self.sf_move_right.value)


    def evaluate(self): # evaluated every frame if any input field has changed  
        print("evaluate")        

        _lf_time = time.time() - self.sav_time # calc duration of last frame --> need for frame-rate independent mapping
        self.sav_time = time.time()

                                     
        if self.sf_move_left.value == True: # key pressed
            self.input_x = self.move_velocity * -1.0 # left movement input
        
        elif self.sf_move_right.value == True: # key pressed
            self.input_x = self.move_velocity # right movement input
        
        else:
            self.input_x = 0.0
    

        if self.jump_flag == True: # jump in process
            # ToDo: generate y-axis input (jump) here --> should decrease over jump-duration
            # ...
            
            # ToDo: stop jump procedure after jump duration has exceeded
            # ...
            
            pass

        ## apply actual input (of this frame) to movement vector
        self.sf_move_vec.value = avango.gua.Vec3(self.input_x, self.input_y, 0.0)
    

