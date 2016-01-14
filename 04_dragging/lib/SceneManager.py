#!/usr/bin/python

# import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon

# import python libraries
import random


class SceneManager:

  # constructor
  def __init__(self, PARENT_NODE):
    
    ### init scene lights
    self.top_light = avango.gua.nodes.LightNode(Name = "top_light")
    PARENT_NODE.Children.value.append(self.top_light)
    self.top_light.Color.value = avango.gua.Color(0.9, 0.9, 0.9)
    self.top_light.EnableSpecularShading.value = True
    self.top_light.Softness.value = 2.0 # exponent
    self.top_light.Falloff.value = 0.0 # exponent
    self.top_light.EnableShadows.value = True
    self.top_light.ShadowMapSize.value = 2048
    self.top_light.ShadowOffset.value = 0.001
    self.top_light.Transform.value = avango.gua.make_trans_mat(0.0, 1.0, 0.0) * \
                                        avango.gua.make_rot_mat(-90.0,1,0,0) * \
                                        avango.gua.make_scale_mat(2.0)

    
    self.front_light = avango.gua.nodes.LightNode(Name = "front_light")
    PARENT_NODE.Children.value.append(self.front_light)
    self.front_light.Color.value = avango.gua.Color(0.35, 0.35, 0.35)
    self.front_light.EnableSpecularShading.value = False
    self.front_light.EnableShadows.value = False
    self.front_light.Transform.value = avango.gua.make_trans_mat(0.0, 0.15, 1.0) * \
                                        avango.gua.make_scale_mat(2.0)


    ### init scene geometry
    _loader = avango.gua.nodes.TriMeshLoader() # init trimesh loader to load external meshes

    # init ground plane
    self.plane_geometry = _loader.create_geometry_from_file("plane", "data/objects/plane.obj", avango.gua.LoaderFlags.DEFAULTS)
    self.plane_geometry.Transform.value = avango.gua.make_trans_mat(0.0, -0.15, 0.0)
    self.plane_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 0.5, 0.0, 1.0))
    PARENT_NODE.Children.value.append(self.plane_geometry)

    # init manipulation geometry
    _number = 25
    
    for _i in range(_number):
   
      _x_range = 280
      _y_range = 130
   
      _rand_pos_x = random.randrange(-_x_range, _x_range) * 0.001
      _rand_pos_y = random.randrange(-_y_range, _y_range) * 0.001
      #_rand_pos_z = random.randrange(-150, 150) * 0.001
      _rand_pos_z = 0.0

      _rand_angle = random.randrange(-180,180)
      _rand_axis_x = random.randrange(0,100) * 0.01
      _rand_axis_y = random.randrange(0,100) * 0.01
      _rand_axis_z = random.randrange(0,100) * 0.01
           
      _geometry = _loader.create_geometry_from_file("monkey" + str(_i), "data/objects/monkey.obj", avango.gua.LoaderFlags.DEFAULTS)
      _geometry.Transform.value = avango.gua.make_trans_mat(_rand_pos_x, _rand_pos_y, _rand_pos_z) * \
                                  avango.gua.make_rot_mat(_rand_angle,_rand_axis_x,_rand_axis_y,_rand_axis_z) * \
                                  avango.gua.make_scale_mat(0.017)
      
      _geometry.add_field(avango.gua.SFMatrix4(), "DraggingOffsetMatrix")
      _geometry.add_field(avango.gua.SFVec4(), "CurrentColor")
      _geometry.CurrentColor.value = avango.gua.Vec4(1.0, 1.0, 1.0, 1.0)
      _geometry.Material.value.set_uniform("Color", _geometry.CurrentColor.value)
      PARENT_NODE.Children.value.append(_geometry)
      
      
