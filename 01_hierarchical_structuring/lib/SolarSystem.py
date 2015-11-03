#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed
import avango.daemon

### import python libraries
import math
import random as rand

class OrbitVisualization:

    ### constructor
    def __init__(self, PARENT_NODE = None , ORBIT_RADIUS = 1.0, ORBIT_INCLINATION = avango.gua.make_rot_mat(0.0, 0, 0, 1)):

        ### parameters ###
        self.number_of_segments = 50
        self.thickness = 0.001
        self.color = avango.gua.Color(1.0,1.0,1.0)
        
        ### resources ###
        for _i in range(self.number_of_segments):
            _segment_angle  = 360.0 / self.number_of_segments
            _segment_length = (math.pi * 2.0 * ORBIT_RADIUS) / self.number_of_segments
      
            _loader = avango.gua.nodes.TriMeshLoader() # init trimesh loader

            _geometry = _loader.create_geometry_from_file("orbit_segment_{0}".format(str(_i)), "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS)        
            _geometry.Transform.value = avango.gua.make_rot_mat(_i * _segment_angle, 0.0, 1.0, 0.0) * \
                                        avango.gua.make_trans_mat(ORBIT_RADIUS, 0.0, 0.0) * \
                                        avango.gua.make_scale_mat(self.thickness, self.thickness, _segment_length)
            _geometry.Material.value.set_uniform("Color", avango.gua.Vec4(self.color.r, self.color.g, self.color.b, 1.0))
            _geometry.Material.value.set_uniform("Emissivity", 0.5)
    
            if PARENT_NODE is not None:
                PARENT_NODE.Children.value.append(_geometry)
    

class SolarSystem(avango.script.Script):

    ### fields ###

    ## input fields
    sf_key0 = avango.SFFloat()
    sf_key1 = avango.SFFloat()
  
    ## output_fields
    sf_time_scale_factor = avango.SFFloat()
    sf_time_scale_factor.value = 1.0


    ### constructor
    def __init__(self):
        self.super(SolarSystem).__init__()
        
        
        ### resources ###

        ## init device sensors
        self.keyboard_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.keyboard_sensor.Station.value = "gua-device-keyboard0"

        self.sf_key0.connect_from(self.keyboard_sensor.Button12)
        self.sf_key1.connect_from(self.keyboard_sensor.Button13)


    def my_constructor(self, PARENT_NODE):

        ### init further resources ###
       
        ## init Sun
        self.sun = SolarObject( NAME = "sun"
                              , TEXTURE_PATH = "data/textures/Sun.jpg"
                              , PARENT_NODE = PARENT_NODE
                              , SF_TIME_SCALE = self.sf_time_scale_factor
                              , DIAMETER = 1392000.0 * 0.05 # downscale sun geometry relative to planet sizes
                              , ORBIT_RADIUS = 0.0 # in km
                              , ORBIT_INCLINATION = 0.0 # in degrees
                              , ORBIT_DURATION = 0.0 # in days
                              , ROTATION_INCLINATION = 7.0 # in degrees
                              , ROTATION_DURATION = 0.0 # in days
                              )
                                                                            
        # init lightsource (only for sun)
        self.sun_light = avango.gua.nodes.LightNode(Name = "sun_light", Type = avango.gua.LightType.POINT)
        self.sun_light.Color.value = avango.gua.Color(1.0, 1.0, 1.0)
        self.sun_light.Brightness.value = 25.0
        self.sun_light.Transform.value = avango.gua.make_scale_mat(25.0) # light volume defined by scale

        _sun_node = self.sun.get_orbit_node()
        _sun_node.Children.value.append(self.sun_light)

        ## init Mercury
        self.mercury = SolarObject(NAME = "mercury",
                                   TEXTURE_PATH = "data/textures/mercury.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 4878.0,
                                   ORBIT_RADIUS = 57900000.0, # in km
                                   ORBIT_INCLINATION = 40.0, # in degrees
                                   ORBIT_DURATION = 28.0, # in days
                                   ROTATION_INCLINATION = 7.0, # in degrees
                                   ROTATION_DURATION = 87.5) # in days)
        ## init Venus
        self.venus = SolarObject(NAME = "venus",
                                   TEXTURE_PATH = "data/textures/venus.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 12104.0,
                                   ORBIT_RADIUS = 108200000.0, # in km
                                   ORBIT_INCLINATION = 3.93, # in degrees
                                   ORBIT_DURATION = 121.5, # in days
                                   ROTATION_INCLINATION = 0.0, # in degrees
                                   ROTATION_DURATION = 224.7) # in days)

        ## init Earth
        self.earth = SolarObject(NAME = "earth",
                                   TEXTURE_PATH = "data/textures/Earth.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 12756.0,
                                   ORBIT_RADIUS = 149600000.0, # in km
                                   ORBIT_INCLINATION = 10.0, # in degrees
                                   ORBIT_DURATION = 365.0, # in days
                                   ROTATION_INCLINATION = 23.4, # in degrees
                                   ROTATION_DURATION = 1.0) # in days)


        ## init Earth-Moon
        self.earth_moon = SolarObject(NAME = "earth_moon",
                                   TEXTURE_PATH = "data/textures/Moon.jpg",
                                   PARENT_NODE = self.earth.get_orbit_node(), 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 3476.0,
                                   ORBIT_RADIUS = 384400.0 * 50, # in km
                                   ORBIT_INCLINATION = 0.0, # in degrees
                                   ORBIT_DURATION = 27.0, # in days
                                   ROTATION_INCLINATION = 6.6, # in degrees
                                   ROTATION_DURATION = 27.0) # in days)

        ## init Mars
        self.mars = SolarObject(NAME = "mars",
                                   TEXTURE_PATH = "data/textures/Mars.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 6787.0,
                                   ORBIT_RADIUS = 227900000.0, # in km
                                   ORBIT_INCLINATION = 1.85, # in degrees
                                   ORBIT_DURATION = 686.98, # in days
                                   ROTATION_INCLINATION = 23.98, # in degrees
                                   ROTATION_DURATION = 1.01) # in days)

        ## init Jupiter
        self.jupiter = SolarObject(NAME = "jupiter",
                                   TEXTURE_PATH = "data/textures/jupiter.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 142754.0,
                                   ORBIT_RADIUS = 778300000.0, # in km
                                   ORBIT_INCLINATION = 1.304, # in degrees
                                   ORBIT_DURATION = (11.86 * 365), # in days
                                   ROTATION_INCLINATION = 3.1, # in degrees
                                   ROTATION_DURATION = (9.8 / 24) ) # in days)
        ## init Jupiter-Moons
        for i in range(0,16):
            _random_orbit_radius = rand.randrange(1289800.0, 23700000.0)
            _jupiter_moon = SolarObject(NAME = "jupiter_moon01",
                                   TEXTURE_PATH = "data/textures/venus.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 12104.0,
                                   ORBIT_RADIUS = _random_orbit_radius, # in km
                                   ORBIT_INCLINATION = 10.0, # in degrees
                                   ORBIT_DURATION = 121.5, # in days
                                   ROTATION_INCLINATION = 0.0, # in degrees
                                   ROTATION_DURATION = 2) # in / 24 days)
        
        self.saturn = SolarObject(NAME = "saturn",
                                   TEXTURE_PATH = "data/textures/saturn.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 120057.0,
                                   ORBIT_RADIUS = 1427000000.0, # in km
                                   ORBIT_INCLINATION = 2.48, # in degrees
                                   ORBIT_DURATION = (29.46 * 365), # in days
                                   ROTATION_INCLINATION = 26.7, # in degrees
                                   ROTATION_DURATION = (10.53 / 24) ) # in days)

        self.uranus = SolarObject(NAME = "uranus",
                                   TEXTURE_PATH = "data/textures/Uranus.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 51177.0,
                                   ORBIT_RADIUS = 2869600000.0, # in km
                                   ORBIT_INCLINATION = 0.772, # in degrees
                                   ORBIT_DURATION = (84 * 365), # in days
                                   ROTATION_INCLINATION = 97.9, # in degrees
                                   ROTATION_DURATION = (17.0 / 24) ) # in days)

        self.neptune = SolarObject(NAME = "neptune",
                                   TEXTURE_PATH = "data/textures/Neptun.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 49520.0,
                                   ORBIT_RADIUS = 4496600000.0, # in km
                                   ORBIT_INCLINATION = 1.7, # in degrees
                                   ORBIT_DURATION = (164.79 * 365), # in days
                                   ROTATION_INCLINATION = 29.6, # in degrees
                                   ROTATION_DURATION = (16.04 / 24) ) # in days)

        self.pluto = SolarObject(NAME = "pluto",
                                   TEXTURE_PATH = "data/textures/Moon.jpg",
                                   PARENT_NODE = _sun_node, 
                                   SF_TIME_SCALE = self.sf_time_scale_factor,
                                   DIAMETER = 2345.0,
                                   ORBIT_RADIUS = 5899900000.0, # in km
                                   ORBIT_INCLINATION = 0.24, # in degrees
                                   ORBIT_DURATION = (247.69 * 365), # in days
                                   ROTATION_INCLINATION = 94.0, # in degrees
                                   ROTATION_DURATION = (6.39 / 24) ) # in days)

    ### callback functions ###

    ## Evaluated when value changes.
    @field_has_changed(sf_key0)
    def sf_key0_changed(self):
        if self.sf_key0.value == True: # button pressed
            _new_factor = self.sf_time_scale_factor.value * 1.5 # increase factor about 50% 

            self.set_time_scale_factor(_new_factor)
      

    ## Evaluated when value changes.
    @field_has_changed(sf_key1)
    def sf_key1_changed(self): 
        if self.sf_key1.value == True: # button pressed
            _new_factor = self.sf_time_scale_factor.value * 0.5 # decrease factor about 50% 

            self.set_time_scale_factor(_new_factor)
      


    ### functions ###
    def set_time_scale_factor(self, FLOAT): 
        self.sf_time_scale_factor.value = min(10000.0, max(1.0, FLOAT)) # clamp float to reasonable intervall



class SolarObject:

    ### constructor
    def __init__( self
                , NAME = ""
                , TEXTURE_PATH = ""
                , PARENT_NODE = None
                , SF_TIME_SCALE = None
                , DIAMETER = 1.0
                , ORBIT_RADIUS = 1.0
                , ORBIT_INCLINATION = 0.0 # in degrees
                , ORBIT_DURATION = 0.0
                , ROTATION_INCLINATION = 0.0 # in degrees
                , ROTATION_DURATION = 0.0
                ):

        ### parameters ###
        self.sf_time_scale_factor = SF_TIME_SCALE        

        self.diameter = DIAMETER * 0.000001
        self.orbit_radius = ORBIT_RADIUS * 0.000000002

        if ORBIT_DURATION > 0.0:
          self.orbit_velocity = 1.0 / ORBIT_DURATION
        else:
          self.orbit_velocity = 0.0

        if ROTATION_DURATION > 0.0:
            self.rotation_velocity = 1.0 / ROTATION_DURATION # get velocity
        else:
            self.rotation_velocity = 0.0

        self.rotation_inclination = ROTATION_INCLINATION
        self.orbit_inclination_mat = avango.gua.make_rot_mat(ORBIT_INCLINATION, 0, 0, 1)


        ### resources ###
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)

        _loader = avango.gua.nodes.TriMeshLoader() # init trimesh loader to load external meshes

        # init geometries of solar object
        self.object_geometry = _loader.create_geometry_from_file(NAME + "_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.object_geometry.Transform.value = avango.gua.make_scale_mat(self.diameter, self.diameter, self.diameter)

        self.object_geometry.Material.value.set_uniform("ColorMap", TEXTURE_PATH)
        self.object_geometry.Material.value.set_uniform("Roughness", 0.2)        
        #self.object_geometry.Material.value.set_uniform("Emissivity", 0.2)
        self.object_geometry.Material.value.EnableBackfaceCulling.value = False

        self.axis1_geometry = _loader.create_geometry_from_file("axis1", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.axis1_geometry.Transform.value = avango.gua.make_scale_mat(0.001,self.diameter*2.5,0.001)
        self.axis1_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.axis1_geometry.Material.value.set_uniform("Emissivity", 1.0) # no shading --> render color

        self.axis2_geometry = _loader.create_geometry_from_file("axis2", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.axis2_geometry.Transform.value = avango.gua.make_scale_mat(0.001,self.diameter*2.5,0.001)
        self.axis2_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 1.0, 0.0, 1.0))
        self.axis2_geometry.Material.value.set_uniform("Emissivity", 1.0) # no shading --> render color

        # init transformation node for rotatning the orbit along the inclination angle
        self.orbit_inclination_node = avango.gua.nodes.TransformNode(Name = NAME + "_orbit_inclination_node")
        self.orbit_inclination_node.Transform.value = self.orbit_inclination_mat
        if PARENT_NODE is not None:
            PARENT_NODE.Children.value.append(self.orbit_inclination_node)

        # init transformation nodes for specific solar object aspects                
        self.orbit_radius_node = avango.gua.nodes.TransformNode(Name = NAME + "_orbit_radius_node")
        self.orbit_radius_node.Transform.value = avango.gua.make_trans_mat(self.orbit_radius, 0.0, 0.0)

        # init planet roation node
        self.rotation_inclination_node = avango.gua.nodes.TransformNode(Name = NAME + "_orbit_radius_node")
        self.rotation_inclination_node.Transform.value = avango.gua.make_rot_mat(self.rotation_inclination, 0, 0, 1)

        #append nodes to each other ...
        self.orbit_inclination_node.Children.value.append(self.orbit_radius_node)
        self.orbit_radius_node.Children.value = [self.rotation_inclination_node, self.axis2_geometry]
        self.rotation_inclination_node.Children.value = [self.object_geometry, self.axis1_geometry]

        ## init sub classes
        # init orbit visualization here ...
        _orbit_visu = OrbitVisualization(PARENT_NODE = self.orbit_inclination_node, ORBIT_RADIUS = self.orbit_radius, ORBIT_INCLINATION = self.orbit_inclination_mat)



    ### functions
    def get_orbit_node(self):
        return self.orbit_radius_node


    def update_orbit(self):
        self.orbit_radius_node.Transform.value = avango.gua.make_rot_mat(self.orbit_velocity * self.sf_time_scale_factor.value, 0.0, 1.0, 0.0) * \
                                                 self.orbit_radius_node.Transform.value


    def update_rotation(self):
        # pass
        self.object_geometry.Transform.value = avango.gua.make_rot_mat(self.rotation_velocity * self.sf_time_scale_factor.value, 0.0, 1.0, 0.0) * \
                                               self.object_geometry.Transform.value

    ### callbacks
    def frame_callback(self): # evaluated once per frame
        self.update_orbit()
        self.update_rotation()


