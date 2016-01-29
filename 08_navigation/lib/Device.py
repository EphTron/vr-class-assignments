#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon

### import python libraries
# ...

class MultiDofInput(avango.script.Script):

    ### fields ###
    
    ## output fields
    mf_dof = avango.MFFloat()
    mf_dof.value = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # init 7 channels

    mf_buttons = avango.MFBool()
    mf_buttons.value = [False, False, False] # init 3 buttons
  
  
    ## Map an input value to a certain interval.
    # @param VALUE The value to be mapped.
    # @param OFFSET The offset to be applied to VALUE, MIN and MAX.
    # @param MIN The minimum value of the old interval.
    # @param MAX The maximum value of the old interval.
    # @param NEG_THRESHOLD The negative threshold to be used.
    # @param POS_THRESHOLD The positive threshold to be used.
    def filter_channel(self, VALUE, OFFSET, MIN, MAX, NEG_THRESHOLD, POS_THRESHOLD):
        VALUE -= OFFSET
        MIN -= OFFSET
        MAX -= OFFSET

        #print("+", VALUE, MAX, POS_THRESHOLD)
        #print("-", VALUE, MIN, NEG_THRESHOLD)

        if VALUE > 0:
            _pos = MAX * POS_THRESHOLD * 0.01

            if VALUE > _pos: # above positive threshold
                VALUE = min( (VALUE - _pos) / (MAX - _pos), 1.0) # normalize interval

            else: # below positive threshold
                VALUE = 0


        elif VALUE < 0:
            _neg = MIN * abs(NEG_THRESHOLD) * 0.01

            if VALUE < _neg:
                VALUE = max( (VALUE - _neg) / abs(MIN - _neg), -1.0)

            else: # above negative threshold
                VALUE = 0
    
        return VALUE


class SpacemouseInput(MultiDofInput):
 
    def my_constructor( self
                      , DEVICE_STATION = ""
                      , TRANSLATION_FACTOR = 1.0
                      , ROTATION_FACTOR = 1.0                      
                      ):

        ### parameters ###
        self.translation_factor = TRANSLATION_FACTOR
        self.rotation_factor = ROTATION_FACTOR

        ### resources ###

        ## init device sensors
        self.device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.device_sensor.Station.value = DEVICE_STATION


        ## init callback triggers
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)


    ### callback functions ###    
  
    def frame_callback(self): # evaluated every frame   
        # button input
        _button1 = self.device_sensor.Button0.value
        _button2 = self.device_sensor.Button1.value
        
        #print(_button1, _button2)
        
        _flag = False
        _buttons = self.mf_buttons.value
        
        if _button1 != _buttons[0]: # trigger changes
            _flag = True

        if _button2 != _buttons[1]: # trigger changes
            _flag = True
         
        if _flag == True: # forward input once per frame (unless there is no input at all)
            self.mf_buttons.value = [_button1,_button2,False]
          
        _x = self.device_sensor.Value0.value
        _y = self.device_sensor.Value1.value * -1.0
        _z = self.device_sensor.Value2.value
        _rx = self.device_sensor.Value3.value
        _ry = self.device_sensor.Value4.value * -1.0
        _rz = self.device_sensor.Value5.value

        #print(_x, _y, _z, _rx, _ry, _rz)
        
        if _x != 0.0:
            _x = self.filter_channel(_x, 0.0, -0.76, 0.82, 3, 3)
        
        if _y != 0.0:
            _y = self.filter_channel(_y, 0.0, -0.7, 0.6, 3, 3)
          
        if _z != 0.0:
            _z = self.filter_channel(_z, 0.0, -0.95, 0.8, 3, 3)
            
        if _rx != 0.0:
            _rx = self.filter_channel(_rx, 0.0, -0.82, 0.8, 12, 12)
         
        if _ry != 0.0:
            _ry = self.filter_channel(_ry, 0.0, -0.5, 0.6, 12, 12)
        
        if _rz != 0.0:
            _rz = self.filter_channel(_rz, 0.0, -0.86, 0.77, 12, 12)


        _x *= self.translation_factor
        _y *= self.translation_factor
        _z *= self.translation_factor
        _rx *= self.rotation_factor
        _ry *= self.rotation_factor
        _rz *= self.rotation_factor                

         
        self.mf_dof.value = [_x,_y,_z,_rx,_ry,_rz,0.0] # propagate input via field connection



class NewSpacemouseInput(MultiDofInput):

    def my_constructor( self
                      , DEVICE_STATION = ""
                      , TRANSLATION_FACTOR = 1.0
                      , ROTATION_FACTOR = 1.0                      
                      ):

        ### parameters ###
        self.translation_factor = TRANSLATION_FACTOR
        self.rotation_factor = ROTATION_FACTOR


        ### resources ###

        ## init device sensors
        self.device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.device_sensor.Station.value = DEVICE_STATION


        ## init callback triggers
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)


    ### callback functions ###    
  
    def frame_callback(self): # evaluated every frame   
        # button input
        _button1 = self.device_sensor.Button0.value
        _button2 = self.device_sensor.Button1.value
        
        #print(_button1, _button2)
        
        _flag = False
        _buttons = self.mf_buttons.value
        
        if _button1 != _buttons[0]: # state has changed
            _flag = True

        if _button2 != _buttons[1]: # state has changed
            _flag = True
         
        if _flag == True: # forward input once per frame (unless there is no input at all)
            self.mf_buttons.value = [_button1,_button2,False]
          
        _x = self.device_sensor.Value0.value
        _y = self.device_sensor.Value1.value * -1.0
        _z = self.device_sensor.Value2.value
        _rx = self.device_sensor.Value3.value
        _ry = self.device_sensor.Value4.value * -1.0
        _rz = self.device_sensor.Value5.value
        

        #print(_x, _y, _z, _rx, _ry, _rz)
        
        if _x != 0.0:
            _x = MultiDofInput.filter_channel(self, _x, 0.0, -350.0, 350.0, -3, 3)
        
        if _y != 0.0:
            _y = MultiDofInput.filter_channel(self, _y, 0.0, -350.0, 250.0, -3, 3)        
          
        if _z != 0.0:
            _z = MultiDofInput.filter_channel(self, _z, 0.0, -350.0, 350.0, -3, 3)
            
        if _rx != 0.0:
            _rx = MultiDofInput.filter_channel(self, _rx, 0.0, -350.0, 350.0, -6, 6)        
         
        if _ry != 0.0:
            _ry = MultiDofInput.filter_channel(self, _ry, 0.0, -350.0, 350.0, -6, 6)
        
        if _rz != 0.0:
            _rz = MultiDofInput.filter_channel(self, _rz, 0.0, -350.0, 350.0, -6, 6)
         

        _x *= self.translation_factor
        _y *= self.translation_factor
        _z *= self.translation_factor
        _rx *= self.rotation_factor
        _ry *= self.rotation_factor
        _rz *= self.rotation_factor                


        self.mf_dof.value = [_x,_y,_z,_rx,_ry,_rz,0.0] # propagate input via field connection
        


class KeyboardInput(MultiDofInput):
   
    def my_constructor( self
                      , DEVICE_STATION = ""
                      , TRANSLATION_FACTOR = 1.0
                      , ROTATION_FACTOR = 1.0                      
                      ):

        ### parameters ###
        self.translation_factor = TRANSLATION_FACTOR
        self.rotation_factor = ROTATION_FACTOR


        ### resources ###

        ## init device sensors
        self.device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.device_sensor.Station.value = DEVICE_STATION


        ## init callback triggers
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)

        
    ### callback functions ###
    
    def frame_callback(self): # evaluated every frame  
        _x = 0.0
        _y = 0.0
        _z = 0.0
        _rx = 0.0
        _ry = 0.0
        _rz = 0.0

        ## handle button input
        if self.device_sensor.Button0.value == True: # w
            _z = -1.0

        if self.device_sensor.Button1.value == True: # a
            _x = -1.0

        if self.device_sensor.Button2.value == True: # s
            _z = 1.0

        if self.device_sensor.Button3.value == True: # d
            _x = 1.0

        if self.device_sensor.Button4.value == True: # arrow left
            _ry = 1.0

        if self.device_sensor.Button5.value == True: # arrow right
            _ry = -1.0

        if self.device_sensor.Button6.value == True: # arrow up
            _rx = -1.0

        if self.device_sensor.Button7.value == True: # arrow down
            _rx = 1.0

        if self.device_sensor.Button8.value == True: # q
            _rz = 1.0

        if self.device_sensor.Button9.value == True: # e
            _rz = -1.0

        if self.device_sensor.Button10.value == True: # page up
            _y = 1.0

        if self.device_sensor.Button11.value == True: # page down
            _y = -1.0
            

        _x *= self.translation_factor
        _y *= self.translation_factor
        _z *= self.translation_factor
        _rx *= self.rotation_factor
        _ry *= self.rotation_factor
        _rz *= self.rotation_factor

        self.mf_dof.value = [_x,_y,_z,_rx,_ry,_rz,0.0] # propagate input via field connection

