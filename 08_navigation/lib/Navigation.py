#!/usr/bin/python

### import guacamole libraries
import avango
import avango.gua
import avango.script
from avango.script import field_has_changed

### import framework libraries
from lib.Intersection import Intersection

### import Python libraries
import math
import time
   
class SteeringNavigation(avango.script.Script):

    ### fields ###

    ## input fields
    mf_dof = avango.MFFloat()
    mf_dof.value = [0.0,0.0,0.0,0.0,0.0,0.0,0.0] # init 7 channels

    sf_pointer_button0 = avango.SFBool()
    sf_pointer_button1 = avango.SFBool()

    sf_head_mat = avango.gua.SFMatrix4()
    sf_head_mat.value = avango.gua.make_trans_mat(0.0,0.0,1.0)

    ## internal fields
    mf_pointer_pick_result = avango.gua.MFPickResult()

    ## output fields
    sf_nav_mat = avango.gua.SFMatrix4()
    sf_nav_mat.value = avango.gua.make_identity_mat()


    ### constructor
    def __init__(self):
        self.super(SteeringNavigation).__init__()

    def my_constructor(self,
                       SCENEGRAPH,
                       PARENT_NODE,
                       MF_NAV_DOF, 
                       POINTER_STATION_NAME, 
                       POINTER_TRACKING_NAME, 
                       TRACKING_TRANSMITTER_OFFSET,
                       ):

        self.scenegraph = SCENEGRAPH
        self.parent_node = PARENT_NODE

        ### parameters ###
        self.ray_length = 100.0 # in meters
        self.ray_thickness = 0.01 # in meters

        self.intersection_point_size = 0.025 # in meters

        self.navidget_duration = 10.0 # in seconds
        self.navidget_sphere_size = 0.5 # in meters


        ### variables ###
        self.navigation_mode = 0 # 0 = steering mode, 1 = maneuvering mode, 3 = Navidget target-mode, 4 = Navidget animation-mode

        self.navidget_start_pos = None
        self.navidget_target_pos = None
        self.navidget_start_quat = None
        self.navidget_target_quat = None
        self.navidget_start_time = None
 
        ### navigation variables
        self.rotation_center = None
        self.first_pick_result = None
        self.rot_x_accum_value = 0.0
        self.rot_y_accum_value = 0.0
        self.manu_distance = 0.0

        self.offset_trans_mat = avango.gua.make_identity_mat()
        self.offset_rot_mat = avango.gua.make_identity_mat()

        self.navidget_trans_mat = avango.gua.make_identity_mat()
        self.navidget_rot_mat = avango.gua.make_identity_mat()
        self.animation_state = "start"


        
        ### resources ###

        # init pointer sensors
        self.pointer_tracking_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.pointer_tracking_sensor.Station.value = POINTER_TRACKING_NAME
        self.pointer_tracking_sensor.ReceiverOffset.value = avango.gua.make_identity_mat()
        self.pointer_tracking_sensor.TransmitterOffset.value = TRACKING_TRANSMITTER_OFFSET

        self.pointer_device_sensor = avango.daemon.nodes.DeviceSensor(DeviceService = avango.daemon.DeviceService())
        self.pointer_device_sensor.Station.value = POINTER_STATION_NAME

        # init nodes and geometries
        _loader = avango.gua.nodes.TriMeshLoader() 

        self.offset_node = avango.gua.nodes.TransformNode(Name = "offset_node")
        self.offset_node.Transform.value = avango.gua.make_trans_mat(0.0,0.0,0.0)
        SCENEGRAPH.Root.value.Children.value.append(self.offset_node)

        self.proxy_geo = _loader.create_geometry_from_file("proxy_geo", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.proxy_geo.Transform.value = avango.gua.make_scale_mat(0.2)
        SCENEGRAPH.Root.value.Children.value.append(self.proxy_geo)
        self.proxy_geo.Material.value.set_uniform("Color", avango.gua.Vec4(0.0, 0.5, 1.0, 1.0))

        self.rot_helper = avango.gua.nodes.TransformNode(Name = "rot_helper")
        self.offset_node.Children.value.append(self.rot_helper)

        #self.proxy_geo.Tags.value = [] # set geometry invisible


        self.ray_transform = avango.gua.nodes.TransformNode(Name = "ray_transform")
        PARENT_NODE.Children.value.append(self.ray_transform)

        self.ray_geometry = _loader.create_geometry_from_file("ray_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.ray_transform.Children.value.append(self.ray_geometry)
        self.ray_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 1.0))
        self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                                            avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,self.ray_length)

        self.intersection_point_geometry = _loader.create_geometry_from_file("intersection_point_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        SCENEGRAPH.Root.value.Children.value.append(self.intersection_point_geometry)
        self.intersection_point_geometry.Tags.value = ["invisible"] # set geometry invisible

        self.maneuvering_point_geometry = _loader.create_geometry_from_file("maneuvering_point_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS)
        SCENEGRAPH.Root.value.Children.value.append(self.maneuvering_point_geometry)
        self.maneuvering_point_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0, 0.0, 0.0, 0.25))
        self.maneuvering_point_geometry.Tags.value = ["invisible"] # set geometry invisible

        ## init Navidget nodes
        self.navidget_node = avango.gua.nodes.TransformNode(Name = "navidget_node")
        SCENEGRAPH.Root.value.Children.value.append(self.navidget_node)
        self.navidget_node.Tags.value = ["invisible"]

        self.navidget_sphere = avango.gua.nodes.TransformNode(Name = "navidget_sphere")
        self.navidget_node.Children.value.append(self.navidget_sphere)


        self.navidget_sphere_geometry = _loader.create_geometry_from_file("navidget_sphere_geometry", "data/objects/sphere.obj", avango.gua.LoaderFlags.DEFAULTS | avango.gua.LoaderFlags.MAKE_PICKABLE)
        self.navidget_node.Children.value.append(self.navidget_sphere_geometry)
        self.navidget_sphere_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(0.0,1.0,1.0,0.25))
        self.navidget_sphere_geometry.Transform.value = avango.gua.make_scale_mat(self.navidget_sphere_size)

        self.navidget_stick = avango.gua.nodes.TransformNode(Name = "navidget_stick")
        self.navidget_node.Children.value.append(self.navidget_stick)

        self.navidget_stick_geometry = _loader.create_geometry_from_file("navidget_stick_geometry", "data/objects/cylinder.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.navidget_stick.Children.value.append(self.navidget_stick_geometry)
        self.navidget_stick_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,1.0,1.0))
        self.navidget_stick_geometry.Transform.value = avango.gua.make_scale_mat(0.015,0.015,self.navidget_sphere_size * 2.0)

        self.navidget_camera_geometry = _loader.create_geometry_from_file("navidget_camera_geometry", "data/objects/cam.obj", avango.gua.LoaderFlags.DEFAULTS)
        self.navidget_node.Children.value.append(self.navidget_camera_geometry)
        #self.navidget_camera_geometry.Material.value.set_uniform("Color", avango.gua.Vec4(1.0,0.0,0.0,1.0))
        self.navidget_camera_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size * 2.0) * \
                                                        avango.gua.make_scale_mat(3.0)

        # init ray intersection for target identification
        self.intersection = Intersection(SCENEGRAPH = SCENEGRAPH)
        self.navidget_intersection = Intersection(SCENEGRAPH = SCENEGRAPH)
        self.navidget_intersection.white_list = ["navidget_sphere"]
        self.navidget_intersection.black_list = [""]

        self.frame_trigger = avango.script.nodes.Update(Callback = self.frame_callback, Active = True)

        # init field connections
        self.mf_dof.connect_from(MF_NAV_DOF)
        self.sf_pointer_button0.connect_from(self.pointer_device_sensor.Button0)
        self.sf_pointer_button1.connect_from(self.pointer_device_sensor.Button1)
        self.ray_transform.Transform.connect_from(self.pointer_tracking_sensor.Matrix)

    
    ### callbacks ###
    @field_has_changed(mf_dof)
    def mf_dof_changed(self):                     

        ## deffine minimum translation and roattion to trigger the navigation
        _min_translation = 0.01
        _min_rotation    = 0.01

        ## handle translation input
        _x = self.mf_dof.value[0]
        _y = self.mf_dof.value[1]
        _z = self.mf_dof.value[2]

        _movement_vector = avango.gua.Vec3(_x, _y, _z)

        ## handle rotation input
        _rx = self.mf_dof.value[3]
        _ry = self.mf_dof.value[4]
        _rz = self.mf_dof.value[5]

        _rotation_vector = avango.gua.Vec3(_rx, _ry, _rz)
        # print(_rotation_vector)

        if self.navigation_mode == 0: # steering mode
            ## implement steering input

            ### translation
            # check for each axis wheather the input value is beyond the minimum defined translation
            # if the translation is bigger than the minimum translation
            # add the min translation with the sign of current translation to the input translation
            if(abs(_movement_vector.x) < _min_translation):
                _movement_vector.x = 0
            else:
                _movement_vector.x -= math.copysign(_min_translation, _movement_vector.x)
            if(abs(_movement_vector.y) < _min_translation):
                _movement_vector.y = 0
            else:
                _movement_vector.y -= math.copysign(_min_translation, _movement_vector.y)
            if(abs(_movement_vector.z) < _min_translation):
                _movement_vector.z = 0
            else:
                _movement_vector.z -= math.copysign(_min_translation, _movement_vector.z)

            # update the _movement_vector by setting it to the 3rd power of the previous movement version
            _movement_vector.x = pow(_movement_vector.x, 3) * 2
            _movement_vector.y = pow(_movement_vector.y, 3) * 2
            _movement_vector.z = pow(_movement_vector.z, 3) * 2
         
            # print(_rotation_vector.x)

            ### rotation
            # same procedure as translation
            if(abs(_rotation_vector.x) < _min_rotation):
                _rotation_vector.x = 0
            else:
                _rotation_vector.x -= math.copysign(_min_rotation, _rotation_vector.x)
            if(abs(_rotation_vector.y) < _min_rotation):
                _rotation_vector.y = 0
            else:
                _rotation_vector.y -= math.copysign(_min_rotation, _rotation_vector.y)
            if(abs(_rotation_vector.z) < _min_rotation):
                _rotation_vector.z = 0
            else:
                _rotation_vector.z -= math.copysign(_min_rotation, _rotation_vector.z)

            _rotation_vector.x = pow(_rotation_vector.x, 3) * 4
            _rotation_vector.y = pow(_rotation_vector.y, 3) * 4
            _rotation_vector.z = pow(_rotation_vector.z, 3) * 4
            # print(_rotation_vector.x)

            # get a quaternion for the current rotation
            _current_rotation = self.sf_nav_mat.value.get_rotate()
            # print(type(self.sf_nav_mat.value))
            # print(type(self.sf_nav_mat.value.get_rotate()))

            # create rotation matrix by getting the values of the quaternion
            _current_rotation_mat = avango.gua.make_rot_mat( _current_rotation.get_angle(), _current_rotation.get_axis() )

            # rotation matrix
            _rotation_matrix = avango.gua.make_rot_mat(_rotation_vector.x,1,0,0) * \
                               avango.gua.make_rot_mat(_rotation_vector.y,0,1,0) * \
                               avango.gua.make_rot_mat(_rotation_vector.z,0,0,1)

            # create new movement matrix containing the 
            _movement_matrix   = _current_rotation_mat * avango.gua.make_trans_mat(_movement_vector)
            _final_movement = avango.gua.make_trans_mat( self.sf_nav_mat.value.get_translate() + \
                                                         _movement_matrix  .get_translate())

            # apply final movement to the navigation
            self.sf_nav_mat.value = _final_movement * _current_rotation_mat * _rotation_matrix


        elif self.navigation_mode == 1: # maneuvering mode

            # _trans_world_vec = self.rotation_center - self.sf_nav_mat.value.get_translate()

            # _quat = avango.gua.make_inverse_mat(self.sf_nav_mat.value).get_rotate()


            # _adjusted_trans = avango.gua.make_rot_mat(_quat.get_angle(), _quat.get_axis()) * _trans_world_vec
            # #_adjusted_trans.z -= _movement_vector.z
            # self.manu_distance = _movement_vector.z
            # _zoom_mat =  avango.gua.make_trans_mat(0.0,0.0,self.manu_distance )
            # _zoom_mat_inv = avango.gua.make_inverse_mat(_zoom_mat)

            # _adjusted_trans_vec = avango.gua.Vec3(_adjusted_trans.x, _adjusted_trans.y, _adjusted_trans.z)

            # _adjusted_trans_mat = avango.gua.make_trans_mat(_adjusted_trans_vec)
            # _adjusted_trans_mat_inv =  avango.gua.make_inverse_mat(_adjusted_trans_mat)

            # ## rotation
            # _rotation_matrix = avango.gua.make_rot_mat(_rotation_vector.x,1,0,0) * \
            #                    avango.gua.make_rot_mat(_rotation_vector.y,0,1,0)

            # self.sf_nav_mat.value = self.sf_nav_mat.value * _adjusted_trans_mat *  _rotation_matrix * _adjusted_trans_mat_inv * _zoom_mat

            self.rot_x_accum_value -= _rotation_vector.x
            self.rot_y_accum_value -= _rotation_vector.y
            self.manu_distance += _movement_vector.z

            #_rot_mat = avango.gua.make_rot_mat(self.rot_accum_value, 0,1,0)

            _rot_mat = avango.gua.make_rot_mat(self.rot_x_accum_value,1,0,0) * \
                       avango.gua.make_rot_mat(self.rot_y_accum_value,0,1,0)


            self.rot_helper.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.manu_distance)
                               

            self.offset_node.Transform.value = self.offset_trans_mat * self.offset_rot_mat * _rot_mat
            _manu_pos = self.rot_helper.WorldTransform.value.get_translate()
            #self.sf_nav_mat.value = avango.gua.make_trans_mat(_manu_pos)
            self.sf_nav_mat.value = self.rot_helper.WorldTransform.value


            

    @field_has_changed(sf_pointer_button0)
    def sf_pointer_button0_changed(self):
        if self.sf_pointer_button0.value == True:
    
            self.set_navigation_mode(1)

            # if there are objects hit by the ray
            if len(self.mf_pointer_pick_result.value) > 0:
                self.first_pick_result = self.mf_pointer_pick_result.value[0]
            else:
                self.first_pick_result = None


            if self.first_pick_result is not None:
                


                _obj_pos = self.first_pick_result.WorldPosition.value# world position of selected object            
                # self.rotation_center = _obj_pos

                # _nav_rot_q = self.sf_nav_mat.value.get_rotate()
                # self.nav_rot = avango.gua.make_rot_mat(_nav_rot_q.get_angle(), _nav_rot_q.get_axis())

                # self.rot_accum_value = avango.gua.make_identity_mat()

                # self.last_rotation = avango.gua.make_identity_mat()

                # self.maneuvering_point_geometry.Tags.value = []
                # self.maneuvering_point_geometry.Transform.value = avango.gua.make_trans_mat(self.rotation_center)


                _obj_mat = avango.gua.make_trans_mat(_obj_pos)
                p2 = self.parent_node.WorldTransform.value.get_translate()

                
                _head_distance = self.sf_head_mat.value.get_translate()
                self.manu_distance = (_obj_pos - p2).length() #-0.6# +_head_distance.z 
                self.rot_helper.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.manu_distance)

                print("_head_distance",_head_distance.z)
                print("abstand obj - prox",self.rot_helper.Transform.value.get_translate().length())
                print("abstand obj - wir",(_obj_pos - p2).length())# + _head_distance.z)
                #print("unserer pos",p2)
                #print("prox pos",self.rot_helper.WorldTransform.value.get_translate())
                           
                # translate the indicator and make it visible
                self.maneuvering_point_geometry.Transform.value = _obj_mat
                self.maneuvering_point_geometry.Tags.value = []


                self.offset_node.Transform.value = _obj_mat
                p3 = self.offset_node.WorldTransform.value.get_translate()
                #print("abstand zwischen rothelper und off",(p3-p1).length())
                #print("abstand zwischen uns und off",(p3-p2).length())
                #print("abstand zwischen uns und rot",(p1-p2).length())
                vec1 = avango.gua.Vec3  (0.0,0.0,-1.0)
                vec2 = p3-p2

                self.offset_trans_mat = _obj_mat
                self.offset_rot_mat = self.get_rotation_matrix_between_vectors(vec1, vec2 )
                self.offset_node.Transform.value = self.offset_trans_mat * self.offset_rot_mat
                #print("prox pos",self.rot_helper.WorldTransform.value.get_translate())

                self.rot_x_accum_value = 0.0
                self.rot_y_accum_value = 0.0
                #self.sf_nav_mat.value = self.rot_helper.Transform.value
                
            # if there is no selected node hide the grab indicator
            else: 
                self.maneuvering_point_geometry.Tags.value = ["invisible"]
                self.rotation_center = avango.gua.make_identity_mat()
                self.manu_distance = 0

        else:
            self.set_navigation_mode(0)

    @field_has_changed(sf_pointer_button1)
    def sf_pointer_button1_changed(self):

        if self.sf_pointer_button1.value == True:
            print("hallo")
            self.set_navigation_mode(3)
            """
            if len(self.mf_pointer_pick_result.value) > 0: # intersection found
                _pick_result = self.mf_pointer_pick_result.value[0] # get first intersection target

                ## set initial Navidget GUI position and orientation
                # if there are objects hit by the ray

            else:
                _pick_result = None


            if _pick_result is not None:
                _obj_pos = _pick_result.WorldPosition.value # world position of selected object            
                _obj_mat = avango.gua.make_trans_mat(_obj_pos)
                self.navidget_node.Transform.value = _obj_mat
                
                self.navidget_stick.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size) #* \
                                                               #avango.gua.make_scale_mat(0.015,0.015,self.navidget_sphere_size * 2.0)

                self.navidget_camera_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size * 2.0) * \
                                                                avango.gua.make_scale_mat(3.0)

                self.navidget_trans_mat = _obj_mat
            """

        else:
            ## start Navidget animation-mode on button release
            if self.navigation_mode == 3:
                self.set_navigation_mode(4)
    
    

    def frame_callback(self): # executed every frame when active
        ## calc intersection
        _mf_pick_result = self.intersection.calc_pick_result(PICK_MAT = self.ray_transform.WorldTransform.value, PICK_LENGTH = self.ray_length)
        self.mf_pointer_pick_result.value = _mf_pick_result.value

        self.update_ray_parameters()

        if self.navigation_mode == 3: # Navidget target-mode
            self.update_navidget_parameters()

        elif self.navigation_mode == 4: # Navidget animation-mode
            self.sf_nav_mat.value = self.animate_navidget()
            
                          

    ### functions ###
    def set_navigation_mode(self, MODE):
        self.navigation_mode = MODE
        if self.navigation_mode == 0: # switch to Steering mode
            # set other modes geometry invisible
            self.maneuvering_point_geometry.Tags.value = ["invisible"]
            self.navidget_node.Tags.value = ["invisible"]

            print("SWITCH TO STEERING MODE")

        elif self.navigation_mode == 1: # switch to maneuvering mode
            self.maneuvering_point_geometry.Tags.value = []
            self.navidget_node.Tags.value = ["invisible"]
            print("SWITCH TO MANEUVERING MODE")
        
        elif self.navigation_mode == 3: # switch to Navidget target mode
                    
                self.navidget_sphere_geometry.Tags.value = []
                self.navidget_sphere_geometry.Tags.value = ["navidget_sphere"]
                self.navidget_node.Tags.value = []

                if len(self.mf_pointer_pick_result.value) > 0: # intersection found
                    _pick_result = self.mf_pointer_pick_result.value[0] # get first intersection target

                    ## set initial Navidget GUI position and orientation
                    # if there are objects hit by the ray

                else:
                    _pick_result = None


                if _pick_result is not None:
                    _obj_pos = _pick_result.WorldPosition.value # world position of selected object            
                    _obj_mat = avango.gua.make_trans_mat(_obj_pos)

                    self.navidget_node.Transform.value = _obj_mat
                    
                    self.navidget_stick.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size) #* \
                                                                   #avango.gua.make_scale_mat(0.015,0.015,self.navidget_sphere_size * 2.0)

                    self.navidget_camera_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.navidget_sphere_size * 2.0) * \
                                                                    avango.gua.make_scale_mat(3.0)

                    self.navidget_trans_mat = _obj_mat


                print("SWITCH TO NAVIDGET TARGET-MODE")

        elif self.navigation_mode == 4:        
            print("try animation")
            ## define start and target parameters for Navidget animation

            self.navidget_start_pos = self.parent_node.WorldTransform.value.get_translate()
            self.navidget_target_pos = self.navidget_camera_geometry.WorldTransform.value.get_translate()
            self.navidget_start_quat = self.parent_node.WorldTransform.value.get_rotate()
            self.navidget_target_quat = self.navidget_camera_geometry.WorldTransform.value.get_rotate()
            self.navidget_start_time = time.time()

            print("SWITCH TO NAVIDGET ANIMATION-MODE")


    def update_ray_parameters(self):
        if len(self.mf_pointer_pick_result.value) > 0: # intersection found
            _pick_result = self.mf_pointer_pick_result.value[0] # get first intersection target

            _point = _pick_result.WorldPosition.value # intersection point in world coordinate system
            _distance = (_point - self.ray_transform.WorldTransform.value.get_translate()).length()

            # update intersection point visualization
            self.intersection_point_geometry.Tags.value = [] # set geometry visible

            self.intersection_point_geometry.Transform.value = avango.gua.make_trans_mat(_point) * \
                                                               avango.gua.make_scale_mat(self.intersection_point_size)


            # update ray visualization (ray length)
            self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,_distance * -0.5) * \
                                                avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,_distance)

        else: # no intersection found
            # update intersection point visualization        
            self.intersection_point_geometry.Tags.value = ["invisible"] # set geometry invisible

            # update ray visualization (ray length)            
            self.ray_geometry.Transform.value = avango.gua.make_trans_mat(0.0,0.0,self.ray_length * -0.5) * \
                                                avango.gua.make_scale_mat(self.ray_thickness,self.ray_thickness,self.ray_length)
                                            
                                            
    def update_navidget_parameters(self):       
        if self.navigation_mode == 3: # maneuvering mode
            _mf_pick_result = self.navidget_intersection.calc_pick_result(PICK_MAT = self.ray_transform.WorldTransform.value, PICK_LENGTH = self.ray_length)
             # if there are objects hit by the ray
            if len(_mf_pick_result.value) > 0:
                _sphere_pick = _mf_pick_result.value[0]

            else:
                _sphere_pick = None


            if _sphere_pick is not None:
                _sphere_pick_pos = _sphere_pick.WorldPosition.value # world position of selected object
                _sphere_pick_mat = avango.gua.make_trans_mat(_sphere_pick_pos)
                self.proxy_geo.Transform.value =_sphere_pick_mat*  avango.gua.make_scale_mat(0.08)

                ######
                _origin_axis = avango.gua.Vec3(0.0,0.0,-1.0) #self.navidget_trans_mat.get_translate() + avango.gua.Vec3(0.0,0.0,1.0) # original rot
                #_our_pos = self.parent_node.WorldTransform.value.get_translate()
                _center = self.navidget_trans_mat.get_translate() 
                

                #vec1 = _hit_point-_origin_axis
                #vec2 = _hit_point-_our_pos

                #_origin_rot_mat = self.get_rotation_matrix_between_vectors(vec1, vec2)

                #_hit_point = self.navidget_node.WorldTransform.value.get_translate()

                #vec1 = _hit_point-_our_pos
                vec1 = _center - _sphere_pick_pos
                vec2 = _origin_axis
                
                self.navidget_rot_mat = self.get_rotation_matrix_between_vectors(vec2, vec1) 
                self.navidget_node.Transform.value = self.navidget_trans_mat * self.navidget_rot_mat
                #print("prox pos",self.rot_helper.WorldTransform.value.get_translate())


    
    def animate_navidget(self):
        print("animphase 1")
        _current_time = time.time()

        _slerp_ratio = (_current_time - self.navidget_start_time) / self.navidget_duration
        

        # last animatuion step - finish animation afterwards
        if _slerp_ratio >= 1:
            self.set_navigation_mode(0)
            print("slerp done")

            _new_io_mat = avango.gua.make_trans_mat(navidget_target_pos) *\
                          avango.gua.make_rot_mat(navidget_target_quat)
            return _new_io_mat

        # move sun 
        else:           
            print("animating", _slerp_ratio)
            _factor = _slerp_ratio

            _new_pos =  (self.navidget_start_pos * (1-_factor)) + ( self.navidget_target_pos * _factor)
            _new_quat = self.slerp(self.navidget_start_quat, self.navidget_target_quat, _slerp_ratio)
            print("animating4", _new_pos)

            _new_io_mat = avango.gua.make_trans_mat(_new_pos) *\
                          avango.gua.make_rot_mat(_new_quat)

            print ( "gogogo")
            return _new_io_mat



    def slerp(self, qa , qb, SLERP_RATIO):
        _quat = qa.slerp_to(qb, SLERP_RATIO)
        return _quat



    def get_rotation_matrix_between_vectors(self, VEC1, VEC2):
        VEC1.normalize()
        VEC2.normalize()
        _angle = math.degrees(math.acos(VEC1.dot(VEC2)))
        _axis = VEC1.cross(VEC2)

        return avango.gua.make_rot_mat(_angle, _axis)
        
