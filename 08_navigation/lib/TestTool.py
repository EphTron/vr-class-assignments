#!/usr/bin/python

## @file
# Contains class LightTool.

### import avango-guacamole libraries
import avango
import avango.gua

import avango.script
from avango.script import field_has_changed

### import framework libraries
from lib.tools.Tool2 import Tool, ToolResource, ResourceSelector
from lib.Intersection import *

import lib.Utilities

### import python libraries
import builtins


class LightTool(Tool):


    ### constructor
    def __init__( self
                , INITIATOR = None
                , INPUT = None
                , RAY_LENGHT = 10.0
                , TOOL_THICKNESS = 0.01 # in meter
                , LIGHT_COLOR = avango.gua.Color(1.0,1.0,1.0)
                , LIGHT_BRIGHTNESS = 10.0
                , LIGHT_DIMENSIONS = avango.gua.Vec3(1.0,1.0,1.0)
                , SHADOW_FLAG = False
                , SHADOW_MAP_SIZE = 512
                , VIEW_CONTEXT_LIST = []
                , VISIBILITY_DICTIONARY = {}
                ):


        ## init base class
        Tool.__init__( self
                     , INITIATOR = INITIATOR
                     , INPUT = INPUT
                     , VIEW_CONTEXT_LIST = VIEW_CONTEXT_LIST
                     , VISIBILITY_DICTIONARY = VISIBILITY_DICTIONARY
                     , RESOURCE_SELECTOR = LightToolResourceSelector(TOOL = self, RAY_LENGTH = RAY_LENGHT)
                     )


        ### parameters ###        
        self.tool_thickness = 0.01
        
        self.light_geometry_thickness = 0.02
        self.light_color = avango.gua.Color(1.0,1.0,1.0)
        self.light_brightness = 10.0
        self.light_dimensions = avango.gua.Vec3(1.0,1.0,1.0)
        self.shadow_flag = False
        self.shadow_map_size = 512


        ### variables ###
        self.active_resource = None
        self.action_flag = False
       
        ### resources ###

        ## init script
        self.script = LightToolScript()
        self.script.my_constructor(self)

        ## init intersection
        self.default_intersection = Intersection( PICK_LENGTH = RAY_LENGHT
                                                , BLACK_LIST = ["invisible"]
                                                )

                                                

        ## init intersection node
        self.intersection_node = avango.gua.nodes.TransformNode(Name = "light_intersection_node")
        builtins.APPLICATION.append_node_to_root(self.intersection_node)

        self.intersection_geometry = builtins.TRIMESH_LOADER.create_geometry_from_file("intersection_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.intersection_geometry.Material.value.set_uniform("Emissivity", 0.5)
        self.intersection_geometry.ShadowMode.value = 0 # disable for shadowing
        self.intersection_geometry.Transform.value = avango.gua.make_scale_mat(0.03)
        self.intersection_node.Children.value.append(self.intersection_geometry)
        
        self.offset_node = avango.gua.nodes.TransformNode(Name = "offset_node")
        self.offset_node.Transform.value = avango.gua.make_trans_mat(0.0,0.0,0.5)
        self.intersection_node.Children.value.append(self.offset_node)


        ## init flashlight geometries
        self.cube_geometry = builtins.TRIMESH_LOADER.create_geometry_from_file("flashlight_resource_cube_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.cube_geometry.Material.value.set_uniform("Emissivity", 0.5)
        self.cube_geometry.ShadowMode.value = 0 # disable for shadowing
        self.cube_geometry.Transform.value = avango.gua.make_scale_mat(self.light_geometry_thickness)
        self.offset_node.Children.value.append(self.cube_geometry)

      
        self.cone_geometry = builtins.TRIMESH_LOADER.create_geometry_from_file("flashlight_resource_cone_geometry", "data/objects/cone.obj", avango.gua.LoaderFlags.DEFAULTS)        
        self.cone_geometry.Material.value.set_uniform("Emissivity", 1.0) # pass through
        self.cone_geometry.ShadowMode.value = 0 # disable for shadowing      
        self.cone_geometry.Transform.value = avango.gua.make_trans_mat(0.0, 0.0, -self.light_geometry_thickness) * \
                                             avango.gua.make_scale_mat(self.light_geometry_thickness * 2.0)
        self.offset_node.Children.value.append(self.cone_geometry)

        ## init light node
        self.light_node = avango.gua.nodes.LightNode(Name = "light_node")
        self.light_node.Type.value = avango.gua.LightType.SPOT
        #self.light_node.ShadowNearClippingInSunDirection.value = 0.01
        #self.light_node.ShadowNearClippingInSunDirection.value = 0.1 * (1.0/LIGHT_DIMENSIONS.z)
        self.light_node.ShadowFarClippingInSunDirection.value = 1.1
        self.light_node.Falloff.value = 0.5
        self.offset_node.Children.value.append(self.light_node)
             
        
        ## trigger callbacks
        ## @var frame_trigger
        # Triggers framewise evaluation of respective callback method
        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = False)

        # Triggers framewise evaluation of respective callback method
        self.update_trigger = avango.script.nodes.Update(Callback = self.update_callback, Active = False)


        ### set initial states ###
        ## set tool properties
        self.set_thickness(TOOL_THICKNESS)
        
        ## set light properties
        self.set_light_color(LIGHT_COLOR)
        self.set_light_brightness(LIGHT_BRIGHTNESS)
        self.set_light_dimensions(LIGHT_DIMENSIONS)
        self.set_shadow_flag(SHADOW_FLAG)
        self.set_shadow_map_size(SHADOW_MAP_SIZE)        
        

        print("LightTool initialized:", Tool.get_id(self))



    ### functions ###

    ## extend respective base-class function
    def init(self):
        Tool.init(self) # call base-class function
            
        self.enable(True)
        
        self.connect_input()


    ## implement respective base-class function
    def init_resource(self, PARENT_NODE, VIEW_CONTEXT):    
        _resource = LightToolResource( TOOL = self
                                     , PARENT_NODE = PARENT_NODE
                                     , VIEW_CONTEXT = VIEW_CONTEXT
                                     , TOOL_THICKNESS = self.tool_thickness
                                     )


    ## extend respective base-class function
    def enable(self, BOOL):
        Tool.enable(self, BOOL) # call base-class function
    
        self.frame_trigger.Active.value = self.enable_flag # enable/disable frame callback
   
   

    ## INPUT CONNECTIONS
    def connect_input(self):
        if self.INPUT is not None: # guard
            if self.INPUT.get_matrix_field() is not None:
                self.connect_matrix(self.INPUT.get_matrix_field())

            if self.INPUT.get_button0_field() is not None:
                self.script.sf_button0.disconnect()
                self.script.sf_button0.connect_from(self.INPUT.get_button0_field())
                 

    def connect_matrix(self, SF_MAT4):
        for _resource in Tool.get_resource_list(self):
            _resource.connect_matrix(SF_MAT4)
    

    def set_thickness(self, FLOAT):
        self.tool_thickness = FLOAT
        
        for _resource in Tool.get_resource_list(self):
            _resource.set_thickness(self.tool_thickness)


    def set_light_color(self, COLOR):
        self.light_color = COLOR

        self.light_node.Color.value = COLOR
        self.cone_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(COLOR.r, COLOR.g, COLOR.b, 1.0))
            

    def set_light_brightness(self, FLOAT):
        self.light_brightness = FLOAT

        self.light_node.Brightness.value = FLOAT
        

    def set_light_dimensions(self, VEC3):
        self.light_dimensions = VEC3

        self.light_node.Transform.value = avango.gua.make_scale_mat(VEC3)

        self.light_node.ShadowNearClippingInSunDirection.value = 0.1 * (1.0/VEC3.z) # shadow near cliiping at 10cm (independant of light dimensions)
        self.light_node.ShadowMaxDistance.value = VEC3.z + 5.0


    def set_shadow_flag(self, FLAG):
        self.shadow_flag = FLAG

        self.light_node.EnableShadows.value = FLAG
        

    def set_shadow_map_size(self, UINT):
        self.shadow_map_size = UINT

        self.light_node.ShadowMapSize.value = UINT
        

    def select_active_resource(self):
        _list = self.RESOURCE_SELECTOR.get_candidate_resources()

        if len(_list) == 0:
            self.active_resource = None
        elif len(_list) > 0:
            self.active_resource = _list[0]


    def process_button0(self, BOOL):
        if BOOL == True:
            if self.active_resource is not None:
                _world_mat = lib.Utilities.get_world_transform(self.active_resource.get_node())
            
                self.default_intersection.set_pick_mat(_world_mat)
                self.default_intersection.compute_intersection()

                _pick_result = self.default_intersection.get_first_pick_result()
                
                if _pick_result is not None:
                    self.action_flag = True

                    self.update_trigger.Active.value = True
                
                    _pick_world_pos = _pick_result[3]
                    
                    _platform_node = self.active_resource.get_node().Parent.value
                    _mat = _platform_node.Transform.value
                    
                    self.intersection_node.Transform.value = avango.gua.make_trans_mat(_pick_world_pos) * \
                                                             avango.gua.make_rot_mat(_mat.get_rotate_scale_corrected())
                                        
                    self.intersection_node.Tags.value = [] # set visible
                
        else:
            if self.action_flag == True:
                self.action_flag = False
                
                self.update_trigger.Active.value = False
                
                self.intersection_node.Tags.value = ["invisible"] # set invisible
        


    ### callback functions ###
    def frame_callback(self):
        #print("frame")
        
        if self.action_flag == False:
            Tool.update_assigned_user(self)
            
            self.RESOURCE_SELECTOR.update_candidate_resources()

            self.select_active_resource()

            
    def update_callback(self):       
        _vec = self.INPUT.get_inputs()
        
        _platform_node = self.active_resource.get_node().Parent.value
        _mat = _platform_node.Transform.value

        _vec = avango.gua.make_rot_mat(_mat.get_rotate_scale_corrected()) * avango.gua.Vec3(_vec.x, _vec.y, 0.0) # express in platform orientation
        
        self.intersection_node.Transform.value = avango.gua.make_trans_mat(self.intersection_node.Transform.value.get_translate()) * \
                                                 avango.gua.make_rot_mat(_vec.x * builtins.APPLICATION.get_application_fps_scale_factor() * 0.5,0,1,0) * \
                                                 avango.gua.make_rot_mat(_vec.y * builtins.APPLICATION.get_application_fps_scale_factor() * 0.5,1,0,0) * \
                                                 avango.gua.make_rot_mat(_vec.z * builtins.APPLICATION.get_application_fps_scale_factor() * 0.5,0,0,1) * \
                                                 avango.gua.make_rot_mat(self.intersection_node.Transform.value.get_rotate_scale_corrected())


class LightToolScript(avango.script.Script):
    
    ### internal fields ###
    sf_button0 = avango.SFBool()
    sf_button1 = avango.SFBool()

    ### Default constructor.
    def __init__(self):
        self.super(LightToolScript).__init__()

        self.CLASS = None
        
      
    def my_constructor(self, CLASS):
        ### references ###
        self.CLASS = CLASS
        

    ## Called whenever sf_button0 changes.
    @field_has_changed(sf_button0)
    def sf_button0_changed(self):
        if self.CLASS is not None:
            self.CLASS.process_button0(self.sf_button0.value)


        
class LightToolResource(ToolResource):

    def __init__( self
                , TOOL = None
                , PARENT_NODE = None
                , VIEW_CONTEXT = None
                , TOOL_THICKNESS = 0.01
                ):


        # call base-class constructor
        ToolResource.__init__( self 
                             , TOOL = TOOL
                             , PARENT_NODE = PARENT_NODE
                             , VIEW_CONTEXT = VIEW_CONTEXT
                             )

               
        ### further resources ###

        ## init flashlight geometries
        self.cursor_geometry = builtins.TRIMESH_LOADER.create_geometry_from_file("light_cursor_geometry", "data/objects/cube.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.cursor_geometry.Material.value.set_uniform("Emissivity", 0.5)
        self.cursor_geometry.ShadowMode.value = 0 # disable for shadowing
        ToolResource.get_node(self).Children.value.append(self.cursor_geometry)
       
        ### set further initial states ###
        self.set_color(COLOR = self.VIEW_CONTEXT.get_color())
        self.set_thickness(TOOL_THICKNESS)



    ### further functions ###
    
    ## extend respective base-class function
    def set_color(self, COLOR = avango.gua.Color(), VIEW_ID = None):       
        if VIEW_ID is None:
            ToolResource.set_color(self, COLOR) # call base-class function
            
            self.cursor_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(self.color.r, self.color.g, self.color.b, 1.0))

        else: # set view dependant color
            self.cursor_geometry.Material.value.set_view_uniform("Color", avango.gua.Vec4(COLOR.r, COLOR.g, COLOR.b, 1.0), VIEW_ID)
                

    def set_thickness(self, FLOAT):
        self.cursor_geometry.Transform.value = avango.gua.make_scale_mat(FLOAT)
        


class LightToolResourceSelector(ResourceSelector):

    ### constructor
    def __init__( self
                , TOOL = None
                , RAY_LENGTH = 1.0 # in meter
                ):


        ## init base class
        ResourceSelector.__init__( self
                                 , TOOL = TOOL
                                 )
    

        ### further resources ###
    
        ## init intersection
        self.default_intersection = Intersection( PICK_LENGTH = RAY_LENGTH
                                                #, BLACK_LIST = ["invisible", "unpickable", "man_unpickable"]
                                                , BLACK_LIST = ["invisible"]                                                
                                                )


    ### further functions ###

    ## implement base-class function
    def update_candidate_resources(self):
        self.candidate_resource_list = [] # clear
            
        if self.TOOL.assigned_user is not None: # guard
            _candidate_resource_list = []
            _candidate_resource_tupel_list = []
                        
            ## check if ray representations intersect viewing frusta of their context
            for _resource in self.TOOL.get_resource_list():
                _world_mat = lib.Utilities.get_world_transform(_resource.get_node())
                _pos = _world_mat.get_translate()
                #print(_resource, _resource.VIEW_CONTEXT, _pos)
               
                _frusta_list = _resource.VIEW_CONTEXT.get_valid_frusta_of_user(self.TOOL.assigned_user)

                for _frustum in _frusta_list:           
                    if _frustum.contains(_pos) == True: # tool resource is inside this frustum --> resource is candidate for interaction
                        _candidate_resource_list.append(_resource)

                        break # break smallest enclosing loop

                    else:
                        self.default_intersection.set_pick_mat(_world_mat)           
                        _ray = self.default_intersection.get_ray()

                        _mf_pick_result = _frustum.ray_test(_ray)

                        if len(_mf_pick_result.value) > 0: # tool resource intersects this frustum --> resource is candidate for interaction
                            _candidate_resource_list.append(_resource)

                            break # break smallest enclosing loop

            #print("1", _candidate_resource_list)


            ## check if candidate ray representations intersect scene geometry
            for _resource in _candidate_resource_list:
                _pick_result = self.calc_intersection(_resource.get_node())

                if _pick_result is not None:
                    _pick_world_pos = _pick_result[3]
                    
                    _frusta_list = _resource.VIEW_CONTEXT.get_valid_frusta_of_user(self.TOOL.assigned_user)
                
                    for _frustum in _frusta_list:           
                        if _frustum.contains(_pick_world_pos) == True: # pick position of this resource is inside frusta of the respective view-context --> pick position visible for assigned user
                            _candidate_resource_tupel_list.append( (_resource, _pick_result) )

                            break # break smallest enclosing loop

            #print("2", _candidate_resource_tupel_list)
            
            ## select one ray representation as interaction candidate
            if len(_candidate_resource_tupel_list) == 1:
                _tupel = _candidate_resource_tupel_list[0]
                self.candidate_resource_list.append(_tupel[0]) # get first (and only) active resource
                
            elif len(_candidate_resource_tupel_list) > 1:                            
                # if multiple intersecting resources exist, take the one with shortest pick distance
                _nearest_distance = sys.maxsize # start with max distance
           
                for _tupel in _candidate_resource_tupel_list:
                    _resource = _tupel[0]
                    _pick_result = _tupel[1]
                    _pick_distance = _pick_result[5]
                
                    if len(self.candidate_resource_list) == 0:
                        self.candidate_resource_list.append(_resource)
                        _nearest_distance = _pick_distance
                        
                    elif len(self.candidate_resource_list) > 0:
                        if _pick_distance < _nearest_distance:
                            self.candidate_resource_list[0] = _resource
                            _nearest_distance = _pick_distance


            # print("3", self.candidate_resource_list)

    

    def calc_intersection(self, NODE):
        _world_mat = lib.Utilities.get_world_transform(NODE)
        
        self.default_intersection.set_pick_mat(_world_mat)
        self.default_intersection.compute_intersection()

        return self.default_intersection.get_first_pick_result()
                
