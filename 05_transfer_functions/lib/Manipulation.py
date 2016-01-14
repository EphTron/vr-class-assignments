#!/usr/bin/python

#### import guacamole libraries
import avango
import avango.gua 
import avango.script
from avango.script import field_has_changed
import avango.daemon

### import python libraries
# ...

   
class ManipulationManager(avango.script.Script):

  ### input fields
  sf_key_1 = avango.SFFloat()
  sf_key_2 = avango.SFFloat()
  sf_key_3 = avango.SFFloat()
  sf_key_4 = avango.SFFloat()
  sf_key_5 = avango.SFFloat()
  sf_key_6 = avango.SFFloat()

  sf_key_8 = avango.SFFloat()
  sf_key_9 = avango.SFFloat()
  sf_key_0 = avango.SFFloat()

  
  mf_dof = avango.MFFloat()
  mf_buttons = avango.MFBool()

  ### internal fields
  sf_hand_mat = avango.gua.SFMatrix4()

  
  # constructor
  def __init__(self):
    self.super(ManipulationManager).__init__()
    

  def my_constructor(self, PARENT_NODE, MOUSE_DEVICE, SPACEMOUSE_DEVICE):

    # references
    self.mouse_device = MOUSE_DEVICE
    self.spacemouse_device = SPACEMOUSE_DEVICE

    # variables
    self.manipulation_technique = None
    self.dragged_objects = []
    self.parent_node = PARENT_NODE
    self.scene_root = PARENT_NODE

    while self.scene_root.Parent.value:
      self.scene_root = self.scene_root.Parent.value

    print(self.scene_root.Name.value)

    self.lf_hand_mat = avango.gua.make_identity_mat() # last frame hand matrix
    
    ### init hand geometry
    _loader = avango.gua.nodes.TriMeshLoader() # init trimesh loader to load external meshes
    
    self.hand_geometry = _loader.create_geometry_from_file("hand_geometry", "data/objects/hand.obj", avango.gua.LoaderFlags.DEFAULTS)
    self.hand_geometry.Transform.value = avango.gua.make_rot_mat(45.0,1,0,0) * \
                                          avango.gua.make_rot_mat(180.0,0,1,0) * \
                                          avango.gua.make_scale_mat(0.06)
    self.hand_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.86, 0.54, 1.0))
    self.hand_geometry.Material.value.set_uniform("Emissivity", 0.9)
    self.hand_geometry.Material.value.set_uniform("Metalness", 0.1)  
    
    self.hand_transform = avango.gua.nodes.TransformNode(Name = "hand_transform")
    self.hand_transform.Children.value = [self.hand_geometry]
    self.hand_transform.Transform.value = avango.gua.make_identity_mat()
    self.parent_node.Children.value.append(self.hand_transform)
    self.sf_hand_mat.connect_from(self.hand_transform.WorldTransform)
    
    # init manipulation techniques
    self.isotonic_position_control_manipulation = IsotonicPositionControlManipulation()
    self.isotonic_position_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.elastic_position_control_manipulation = ElasticPositionControlManipulation()
    self.elastic_position_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.isotonic_rate_control_manipulation = IsotonicRateControlManipulation()
    self.isotonic_rate_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.elastic_rate_control_manipulation = ElasticRateControlManipulation()
    self.elastic_rate_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.isotonic_acceleration_control_manipulation = IsotonicAccelerationControlManipulation()
    self.isotonic_acceleration_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.elastic_acceleration_control_manipulation = ElasticAccelerationControlManipulation()
    self.elastic_acceleration_control_manipulation.my_constructor(self.hand_transform.Transform, self.mf_dof)

    self.set_manipulation_technique(0)

    # init keyboard sensor
    self.keyboard_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
    self.keyboard_sensor.Station.value = "device-keyboard"

    self.sf_key_1.connect_from(self.keyboard_sensor.Button12) # key 1
    self.sf_key_2.connect_from(self.keyboard_sensor.Button13) # key 2
    self.sf_key_3.connect_from(self.keyboard_sensor.Button14) # key 3
    self.sf_key_4.connect_from(self.keyboard_sensor.Button15) # key 4
    self.sf_key_5.connect_from(self.keyboard_sensor.Button16) # key 5
    self.sf_key_6.connect_from(self.keyboard_sensor.Button17) # key 6


  ### functions
  def set_manipulation_technique(self, INT):

    self.manipulation_technique = INT 

    # disable prior manipulation technique
    self.isotonic_position_control_manipulation.enable_manipulation(False)
    self.elastic_position_control_manipulation.enable_manipulation(False)
    self.isotonic_rate_control_manipulation.enable_manipulation(False)
    self.elastic_rate_control_manipulation.enable_manipulation(False)
    self.isotonic_acceleration_control_manipulation.enable_manipulation(False)      
    self.elastic_acceleration_control_manipulation.enable_manipulation(False)

    # remove field connections    
    self.mf_dof.disconnect()
    self.mf_buttons.disconnect()
    
    if self.manipulation_technique == 0: # isotonic position control
 
      self.isotonic_position_control_manipulation.enable_manipulation(True)
      
      # init field connections      
      self.mf_dof.connect_from(self.mouse_device.mf_dof)
      self.mf_buttons.connect_from(self.mouse_device.mf_buttons)
    
    elif self.manipulation_technique == 1: # elastic position control
    
      self.elastic_position_control_manipulation.enable_manipulation(True)

      # init field connections      
      self.mf_dof.connect_from(self.spacemouse_device.mf_dof)    
      self.mf_buttons.connect_from(self.spacemouse_device.mf_buttons)
    
    elif self.manipulation_technique == 2: # isotonic rate control
    
      self.isotonic_rate_control_manipulation.enable_manipulation(True)
    
      # init field connections
      self.mf_dof.connect_from(self.mouse_device.mf_dof)
      self.mf_buttons.connect_from(self.mouse_device.mf_buttons)
    
    elif self.manipulation_technique == 3: # elastic rate control
    
      self.elastic_rate_control_manipulation.enable_manipulation(True)

      # init field connections      
      self.mf_dof.connect_from(self.spacemouse_device.mf_dof)
      self.mf_buttons.connect_from(self.spacemouse_device.mf_buttons)
    
    elif self.manipulation_technique == 4: # isotonic acceleration control

      self.isotonic_acceleration_control_manipulation.enable_manipulation(True)

      # init field connections      
      self.mf_dof.connect_from(self.mouse_device.mf_dof)
      self.mf_buttons.connect_from(self.mouse_device.mf_buttons)

    elif self.manipulation_technique == 5: # elastic acceleration control
    
      self.elastic_acceleration_control_manipulation.enable_manipulation(True)

      # init field connections      
      self.mf_dof.connect_from(self.spacemouse_device.mf_dof)
      self.mf_buttons.connect_from(self.spacemouse_device.mf_buttons)
    

  def start_dragging(self):
  
    _hand_mat = self.sf_hand_mat.value

    # travers all scenegraph nodes
    for _node in self.scene_root.Children.value:

      _name = _node.Name.value

      if _name.count("monkey") > 0: # a monkey node
        
        if self.is_highlight_material(_node.CurrentColor.value): # monkey node in close proximity
         
          _node.CurrentColor.value = avango.gua.Vec4(1.0, 0.0, 0.0, 1.0)
          _node.Material.value.set_uniform("Color", _node.CurrentColor.value) # switch to dragging material

          self.dragged_objects.append(_node) # add node for dragging
          
          ## dragging without snapping
          _dragging_offset_mat = avango.gua.make_inverse_mat(_hand_mat) * _node.Transform.value # object transformation in hand coordinate system
          _node.DraggingOffsetMatrix.value = _dragging_offset_mat # here you can store node dependent dragging transformations 
         
  

  def object_dragging(self):

    # apply hand movement to (all) dragged objects
    for _node in self.dragged_objects:

      _node.Transform.value = self.sf_hand_mat.value * _node.DraggingOffsetMatrix.value


  
  def stop_dragging(self):
  
    # travers all dragged objects
    for _node in self.dragged_objects:
      
      _node.CurrentColor.value = avango.gua.Vec4(0.0, 1.0, 0.0, 1.0)
      _node.Material.value.set_uniform("Color", _node.CurrentColor.value) # switch to highlight material

    
    self.dragged_objects = [] # clear list


  def is_default_material(self, VEC4):
    return VEC4.x == 1.0 and VEC4.y == 1.0 and VEC4.z == 1.0 and VEC4.w == 1.0


  def is_highlight_material(self, VEC4):
    return VEC4.x == 0.0 and VEC4.y == 1.0 and VEC4.z == 0.0 and VEC4.w == 1.0


  def is_dragging_material(self, VEC4):
    return VEC4.x == 1.0 and VEC4.y == 0.0 and VEC4.z == 0.0 and VEC4.w == 1.0
  
    
  ### callbacks
  @field_has_changed(sf_key_1)
  def sf_key_1_changed(self):
    
    if self.sf_key_1.value == True: # key is pressed

      self.set_manipulation_technique(0) # switch to isotonic position control


  @field_has_changed(sf_key_2)
  def sf_key_2_changed(self):
    
    if self.sf_key_2.value == True: # key is pressed
      
      self.set_manipulation_technique(1) # switch to elastic position control
      

  @field_has_changed(sf_key_3)
  def sf_key_3_changed(self):
    
    if self.sf_key_3.value == True: # key is pressed
      
      self.set_manipulation_technique(2) # switch to isotonic rate control
            

  @field_has_changed(sf_key_4)
  def sf_key_4_changed(self):
    
    if self.sf_key_4.value == True: # key is pressed

      self.set_manipulation_technique(3) # switch to elastic rate control
                  

  @field_has_changed(sf_key_5)
  def sf_key_5_changed(self):
    
    if self.sf_key_5.value == True: # key is pressed
      
      self.set_manipulation_technique(4) # switch to isotonic acceleration control
      

  @field_has_changed(sf_key_6)
  def sf_key_6_changed(self):
    
    if self.sf_key_6.value == True: # key is pressed
      
       self.set_manipulation_technique(5) # switch to elastic acceleration control
      

  @field_has_changed(mf_buttons)
  def mf_buttons_changed(self):

    _left_button = self.mf_buttons.value[0]
    _right_button = self.mf_buttons.value[1]
    
    _button = _left_button ^ _right_button # button left XOR button right

    if _button == True: # key is pressed
      self.start_dragging()
      
    else:      
      self.stop_dragging()
      
          
  @field_has_changed(sf_hand_mat)
  def sf_hand_mat_changed(self):

    _hand_pos = self.sf_hand_mat.value.get_translate()

    # travers all scenegraph nodes
    for _node in self.scene_root.Children.value:

      _name = _node.Name.value

      if _name.count("monkey") > 0: # identify a monkey node

        _pos = _node.Transform.value.get_translate() # a monkey position

        _dist = (_hand_pos - _pos).length() # hand-object distance
        _node_col = _node.CurrentColor.value

        ### toggle object highlight
        if _dist < 0.02 and self.is_default_material(_node_col):
          _node.CurrentColor.value = avango.gua.Vec4(0.0, 1.0, 0.0, 1.0)
          _node.Material.value.set_uniform("Color", _node.CurrentColor.value) # switch to highlight material
        
        elif _dist > 0.025 and self.is_highlight_material(_node_col):
          _node.CurrentColor.value = avango.gua.Vec4(1.0, 1.0, 1.0, 1.0)
          _node.Material.value.set_uniform("Color", _node.CurrentColor.value) # switch to default material


    self.object_dragging() # evtl. drag object with hand input

    # calc hand velocity
    _distance = (_hand_pos - self.lf_hand_mat.get_translate()).length()
    _velocity = _distance * 60.0 # application loop runs with 60Hz
    _velocity = round(_velocity, 2) # round to 2nd decimal place
    print(_velocity, "m/s")
    self.lf_hand_mat = self.sf_hand_mat.value



class Manipulation(avango.script.Script):

  ### input fields
  mf_dof = avango.MFFloat()
  mf_dof.value = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # init 7 channels

  ### output_fields
  sf_mat = avango.gua.SFMatrix4()
  sf_mat.value = avango.gua.make_identity_mat()

  ### constructor
  def __init__(self):
    self.super(Manipulation).__init__()

    # variables
    self.type         = ""
    self.enable_flag  = False

    
  ### callbacks
  @field_has_changed(mf_dof)
  def mf_dof_changed(self):
    
    if self.enable_flag == True:

      self.manipulate()

        
  ### functions
  def enable_manipulation(self, FLAG):
    
    self.enable_flag = FLAG
    
    if self.enable_flag == True:
      print(self.type + " enabled")
    
      self.reset()
      
   
  def manipulate(self):
    raise NotImplementedError("To be implemented by a subclass.")


  def reset(self):
    raise NotImplementedError("To be implemented by a subclass.")
    
    
  def clamp_matrix(self, MATRIX):
    
    # clamp translation to certain range
    _x_range = 0.3 # in meter
    _y_range = 0.15 # in meter
    _z_range = 0.15 # in meter    
    
    MATRIX.set_element(0,3, min(_x_range, max(-_x_range, MATRIX.get_element(0,3)))) # clamp x-axis
    MATRIX.set_element(1,3, min(_y_range, max(-_y_range, MATRIX.get_element(1,3)))) # clamp y-axis
    MATRIX.set_element(2,3, min(_z_range, max(-_z_range, MATRIX.get_element(2,3)))) # clamp z-axis
     
    return MATRIX


  def set_matrix(self, MATRIX):
 
    if MATRIX != self.sf_mat.value: # check for valid input --> hand was moves
    
      MATRIX = self.clamp_matrix(MATRIX) # clamp to translation range
    
      self.sf_mat.value = MATRIX # apply new hand matrix


### ISOTONIC DEVICE MAPPINGS ##

class IsotonicPositionControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):
    
    self.type = "isotonic-position-control"
    
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
  
    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]
      
    _x *= 0.1
    _y *= 0.1
    _z *= 0.1
   
    _new_mat = self.sf_mat.value * avango.gua.make_trans_mat(_x, _y, _z)
<<<<<<< HEAD

    self.set_matrix(_new_mat) # apply new input matrix
    
    
=======
    print(self.sf_mat.value)

    self.set_matrix(_new_mat) # apply new input matrix
    
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
  # override base class function
  def reset(self):
  
    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center


class IsotonicRateControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):

    self.type = "isotonic-rate-control"

<<<<<<< HEAD
    # further variables if needed
    # ...
=======
    self.velocity_vec = avango.gua.Vec3(0,0,0)
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
      
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
<<<<<<< HEAD
  
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)
    pass
    
=======

    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]

    _x *= 0.006
    _y *= 0.006
    _z *= 0.006

    _vec = avango.gua.Vec3(_x, _y, _z)
    self.velocity_vec += _vec

    _current_trans = self.sf_mat.value.get_translate()
    _rate = _current_trans + self.velocity_vec
    _new_mat = avango.gua.make_trans_mat(_rate)

    self.set_matrix(_new_mat)
  
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88

  # override base class function
  def reset(self):

    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center
<<<<<<< HEAD

    # implement further reset functionality here if needed
    # ...
    
=======
    self.velocity_vec = avango.gua.Vec3(0,0,0)

>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88

class IsotonicAccelerationControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):

    self.type = "isotonic-acceleration-control"

<<<<<<< HEAD
    # further variables if needed
    # ...

      
=======
    self.velocity_vec = avango.gua.Vec3(0,0,0)
    self.acceleration_vec = avango.gua.Vec3(0,0,0)

>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
  
<<<<<<< HEAD
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)
    pass

=======
    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]

    _x *= 0.0001
    _y *= 0.0001
    _z *= 0.0001

    _vec = avango.gua.Vec3(_x, _y, _z)
    self.acceleration_vec += _vec
    self.velocity_vec += self.acceleration_vec

    _current_trans = self.sf_mat.value.get_translate()
    _rate = _current_trans + self.velocity_vec
    _new_mat = avango.gua.make_trans_mat(_rate)

    self.set_matrix(_new_mat)
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88

  # override base class function
  def reset(self):

    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center
<<<<<<< HEAD

    # implement further reset functionality here if needed
    # ...

=======
    self.velocity_vec = avango.gua.Vec3(0,0,0)
    self.acceleration_vec = avango.gua.Vec3(0,0,0)
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88


### ELASTIC DEVICE MAPPINGS ###

class ElasticPositionControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):
    
    self.type = "elastic-position-control"

    # further variables if needed
    # ...
    
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
<<<<<<< HEAD
  
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)
    pass

=======
    
    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]
      
    _new_mat = avango.gua.make_trans_mat(_x, _y, _z)
    self.set_matrix(_new_mat)
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88

  # override base class function
  def reset(self):
  
    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center

    # implement further reset functionality here if needed
    # ...


class ElasticRateControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):

    self.type = "elastic-rate-control"

    # further variables if needed
    # ...
      
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
<<<<<<< HEAD
  
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)
    pass
         

  # override base class function
  def reset(self):
  
    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center
  
    # implement further reset functionality here if needed
    # ...
=======

    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]
      
    _x *= 0.01
    _y *= 0.01
    _z *= 0.01
    
    _new_mat = self.sf_mat.value * avango.gua.make_trans_mat(_x, _y, _z)
    print(self.sf_mat.value)

    self.set_matrix(_new_mat) # apply new input matrix
  
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)       

  # override base class function
  def reset(self):
      self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88


class ElasticAccelerationControlManipulation(Manipulation):

  def my_constructor(self, SF_MATRIX, MF_DOF):

    self.type = "elastic-acceleration-control"
<<<<<<< HEAD

    # further variables if needed
    # ...
      
=======
   
    self.velocity_vec = avango.gua.Vec3(0,0,0)
    self.acceleration_vec = avango.gua.Vec3(0,0,0)

>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
    # init field connections
    self.mf_dof.connect_from(MF_DOF)
    
    SF_MATRIX.connect_from(self.sf_mat)


  # override base class function
  def manipulate(self):
<<<<<<< HEAD
    
    # implement functionality here
    # apply new matrix with self.set_matrix(MATRIX)
    pass
    
=======

    _x = self.mf_dof.value[0]
    _y = self.mf_dof.value[1]
    _z = self.mf_dof.value[2]

    _x *= 0.000001
    _y *= 0.000001
    _z *= 0.000001

    _vec = avango.gua.Vec3(_x, _y, _z)
    self.acceleration_vec += _vec
    self.velocity_vec += self.acceleration_vec

    _current_trans = self.sf_mat.value.get_translate()
    _rate = _current_trans + self.velocity_vec
    _new_mat = avango.gua.make_trans_mat(_rate)

    self.set_matrix(_new_mat)
>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88

  # override base class function
  def reset(self):

    self.sf_mat.value = avango.gua.make_identity_mat() # snap hand to center
<<<<<<< HEAD
 
=======
    self.velocity_vec = avango.gua.Vec3(0,0,0)
    self.acceleration_vec = avango.gua.Vec3(0,0,0)

>>>>>>> f99fb1ded9da219bd2122829e56a3cf1d4e57c88
    # implement further reset functionality here if needed
    # ...