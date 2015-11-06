#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

### import python libraries
import time
import math


class Scene:

    ## constructor
    def __init__( self
                , PARENT_NODE = None
                ):

        ### resources ###

        ## init scene light
        self.scene_light = avango.gua.nodes.LightNode(Name = "scene_light", Type = avango.gua.LightType.POINT)
        self.scene_light.Color.value = avango.gua.Color(0.8, 0.8, 0.8)
        self.scene_light.Brightness.value = 25.0
        self.scene_light.Falloff.value = 1.0 # exponent
        self.scene_light.EnableShadows.value = True
        self.scene_light.ShadowMapSize.value = 1024
        self.scene_light.ShadowOffset.value = 0.001
        '''
        self.scene_light.EnableDiffuseShading.value = True
        self.scene_light.EnableSpecularShading.value = True
        '''
        self.scene_light.Transform.value = avango.gua.make_trans_mat(0.0, 1.0, 0.0) * \
                                           avango.gua.make_rot_mat(-90.0, 1, 0, 0) * \
                                           avango.gua.make_scale_mat(2.0)
        PARENT_NODE.Children.value.append(self.scene_light)


        ## init scene geometries
        
        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external meshes

        self.parcours0_geometry = _loader.create_geometry_from_file("parcours0_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours0_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, 0.0) * \
                                                  avango.gua.make_scale_mat(0.5, 0.01, 0.05)
        self.parcours0_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours0_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours0_geometry)
        

        self.parcours1_geometry = _loader.create_geometry_from_file("parcours1_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours1_geometry.Transform.value = avango.gua.make_trans_mat(-0.6, 0.0, 0.0) * \
                                                  avango.gua.make_scale_mat(0.5, 0.01, 0.05)
        self.parcours1_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours1_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours1_geometry)


        self.parcours2_geometry = _loader.create_geometry_from_file("parcours2_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours2_geometry.Transform.value = avango.gua.make_trans_mat(0.1, 0.1, 0.0) * \
                                                  avango.gua.make_scale_mat(0.1, 0.01, 0.05)
        self.parcours2_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours2_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours2_geometry)
                                                  

        self.parcours3_geometry = _loader.create_geometry_from_file("parcours3_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours3_geometry.Transform.value = avango.gua.make_trans_mat(-0.15, 0.025, 0.0) * \
                                                  avango.gua.make_scale_mat(0.1, 0.05, 0.05)
        self.parcours3_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours3_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours3_geometry)
                                                  

        self.parcours4_geometry = _loader.create_geometry_from_file("parcours4_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours4_geometry.Transform.value = avango.gua.make_trans_mat(-0.3, -0.1 ,0.0) * \
                                                  avango.gua.make_scale_mat(0.15, 0.01, 0.05)
        self.parcours4_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours4_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours4_geometry)
                                                  

        self.parcours5_geometry = _loader.create_geometry_from_file("parcours5_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours5_geometry.Transform.value = avango.gua.make_trans_mat(-0.05, 0.175, 0.0) * \
                                                  avango.gua.make_scale_mat(0.15, 0.01, 0.05)
        self.parcours5_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours5_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours5_geometry)


        self.parcours6_geometry = _loader.create_geometry_from_file("parcours7_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours6_geometry.Transform.value = avango.gua.make_trans_mat(0.1, 0.25, 0.0) * \
                                                  avango.gua.make_scale_mat(0.1, 0.01, 0.05)
        self.parcours6_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours6_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours6_geometry)
                                                  
        
        self.parcours7_geometry = _loader.create_geometry_from_file("parcours8_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.parcours7_geometry.Transform.value = avango.gua.make_trans_mat(-0.5, 0.04, 0.0) * \
                                                  avango.gua.make_scale_mat(0.01, 0.08, 0.05)
        self.parcours7_geometry.Material.value.set_uniform("ColorMap", "data/textures/bricks_diffuse.jpg")
        self.parcours7_geometry.Material.value.set_uniform("NormalMap", "data/textures/bricks_normal.jpg")
        PARENT_NODE.Children.value.append(self.parcours7_geometry)


