#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

### import python libraries
# ...


class SceneManager(avango.script.Script):

    ## input fields
    sf_reset_button = avango.SFBool()

    sf_scene1_button = avango.SFBool()
    sf_scene2_button = avango.SFBool()


    ## Default constructor.
    def __init__(self):
        self.super(SceneManager).__init__()
        
        ### resources ###
        self.keyboard_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.keyboard_device_sensor.Station.value = "device-keyboard"

        ## init field connections
        self.sf_reset_button.connect_from(self.keyboard_device_sensor.Button18) # spacebar key
        self.sf_scene1_button.connect_from(self.keyboard_device_sensor.Button12) # key 1
        self.sf_scene2_button.connect_from(self.keyboard_device_sensor.Button13) # key 2


    def my_constructor( self
                      , PARENT_NODE = None
                      ):


        ## init scenes
        self.scene1 = Scene1(PARENT_NODE = PARENT_NODE)        
        self.scene2 = Scene2(PARENT_NODE = PARENT_NODE)

        self.scene1.enable(True)



    ### callback functions ###
    @field_has_changed(sf_scene1_button)
    def sf_scene1_button_changed(self):
        if self.sf_scene1_button.value == True: # button pressed
            self.scene1.enable(True)
            self.scene2.enable(False)


    @field_has_changed(sf_scene2_button)
    def sf_scene2_button_changed(self):
        if self.sf_scene2_button.value == True: # button pressed
            self.scene1.enable(False)
            self.scene2.enable(True)

    '''
    @field_has_changed(sf_reset_button)
    def sf_reset_button_changed(self):
        if self.sf_reset_button.value == True: # button pressed
            self.scene1.reset()
            self.scene2.reset()
    '''


class Scene:

    ## constructor
    def __init__( self
                , NAME = "Town"
                , PARENT_NODE = None
                ):

        ### resources ###
        self.scene_root = avango.gua.nodes.TransformNode(Name = NAME)
        PARENT_NODE.Children.value.append(self.scene_root)
        

        ### set initial states ###
        self.enable(False)
        
        
    ### functions ###
    def enable(self, BOOL):
        if BOOL == True:
            self.scene_root.Tags.value = [] # visible
        else:
            self.scene_root.Tags.value = ["invisible"] # invisible
            
        
    '''
    def reset(self):
        print("reset")
        for _node in self.scene_root.Children.value:
            if _node.has_field("HomeMatrix") == True:
                _node.Transform.value = _node.HomeMatrix.value
    '''


class Scene1(Scene):

    ## constructor
    def __init__( self
                , NAME = ""
                , PARENT_NODE = None
                ):


        # call base class constructor
        Scene.__init__( self
                      , NAME = NAME
                      , PARENT_NODE = PARENT_NODE
                      )


        ### further resources ###
        
        ## init scene geometries        
        _loader = avango.gua.nodes.TriMeshLoader() # get trimesh loader to load external meshes

        # water
        self.water_geometry = _loader.create_geometry_from_file("water_geometry", "data/objects/plane.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.water_geometry.Transform.value = avango.gua.make_trans_mat(0.0,-0.35,0.0) * avango.gua.make_scale_mat(150.0,1.0,150.0)
        self.water_geometry.Material.value.set_uniform("Roughness", 0.4)
        self.scene_root.Children.value.append(self.water_geometry)
        self.water_geometry.Tags.value = ["town"]
        # town
        self.town = _loader.create_geometry_from_file("town", "/opt/3d_models/architecture/medieval_harbour/town.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.town.Transform.value = avango.gua.make_scale_mat(7.5)
        self.scene_root.Children.value.append(self.town)
        self.town.Tags.value = ["town"]

        for _node in self.town.Children.value:
            _node.Material.value.EnableBackfaceCulling.value = False
            _node.Material.value.set_uniform("Emissivity", 0.0)
            _node.Material.value.set_uniform("Metalness", 0.1)
            _node.Material.value.set_uniform("Roughness", 0.5)


        ## init scene light
        _light_dimensions = avango.gua.Vec3(120.0, 120.0, 120.0)

        self.scene_light = avango.gua.nodes.LightNode(Name = "scene_light", Type = avango.gua.LightType.SPOT)
        self.scene_light.Color.value = avango.gua.Color(0.5, 0.5, 0.5)
        self.scene_light.Brightness.value = 50.0
        self.scene_light.Softness.value = 0.5 # exponent
        self.scene_light.Falloff.value = 0.5 # exponent
        #self.scene_light.EnableGodrays.value = True
        self.scene_light.EnableShadows.value = True
        self.scene_light.ShadowMapSize.value = 2048
        #self.scene_light.ShadowOffset.value = 0.01
        self.scene_light.ShadowMaxDistance.value = 120.0
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.2
        self.scene_light.Transform.value = avango.gua.make_trans_mat(0.0, 45.0, 55.0) * \
                                           avango.gua.make_rot_mat(-45.0,1,0,0) * \
                                           avango.gua.make_scale_mat(_light_dimensions)
        self.scene_root.Children.value.append(self.scene_light)



class Scene2(Scene):

    ## constructor
    def __init__( self
                , NAME = "Car"
                , PARENT_NODE = None
                ):


        # call base class constructor
        Scene.__init__( self
                      , NAME = NAME
                      , PARENT_NODE = PARENT_NODE
                      )

        ## init scene geometries  
        _loader = avango.gua.nodes.TriMeshLoader()

        self.passat_geometry = _loader.create_geometry_from_file("passat_geometry", "/opt/3d_models/cars/passat/passat.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.passat_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,0.0) * \
                                               avango.gua.make_rot_mat(-90.0,1,0,0) * \
                                               avango.gua.make_rot_mat(-90.0,0,0,1) * \
                                               avango.gua.make_scale_mat(0.035)
        self.scene_root.Children.value.append(self.passat_geometry)
        self.passat_geometry.Tags.value = ["car"]
        

        # ground
        self.ground_geometry = _loader.create_geometry_from_file("ground_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.LOAD_MATERIALS)
        self.ground_geometry.Transform.value = avango.gua.make_scale_mat(12.0,0.01,12.0)
        self.ground_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,1.0,0.85,1.0))
        self.ground_geometry.Material.value.set_uniform("Emissivity", 0.5)
        self.ground_geometry.Material.value.set_uniform("Metalness", 0.1)
        self.scene_root.Children.value.append(self.ground_geometry)
        self.ground_geometry.Tags.value = ["car"]


        ## init scene light
        _light_dimensions = avango.gua.Vec3(30.0, 30.0, 30.0)

        self.scene_light = avango.gua.nodes.LightNode(Name = "scene_light", Type = avango.gua.LightType.SPOT)
        self.scene_light.Color.value = avango.gua.Color(0.5, 0.5, 0.5)
        self.scene_light.Brightness.value = 30.0
        #self.scene_light.Softness.value = 0.5 # exponent
        #self.scene_light.Falloff.value = 0.5 # exponent
        #self.scene_light.EnableGodrays.value = True
        self.scene_light.EnableShadows.value = True
        self.scene_light.ShadowMapSize.value = 2048
        self.scene_light.ShadowOffset.value = 0.005
        self.scene_light.ShadowMaxDistance.value = 120.0
        self.scene_light.ShadowNearClippingInSunDirection.value = 0.1
        self.scene_light.Transform.value = avango.gua.make_trans_mat(0.0, 10.0, 3.0) * \
                                           avango.gua.make_rot_mat(-80.0,1,0,0) * \
                                           avango.gua.make_scale_mat(_light_dimensions)
        self.scene_root.Children.value.append(self.scene_light)
        
