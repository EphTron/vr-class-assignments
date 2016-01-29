#!/usr/bin/python

### import guacamole libraries
import avango
import avango.daemon
import os


## Initializes AR Track
def init_art_tracking():

    # create instance of DTrack
    _dtrack = avango.daemon.DTrack()
    _dtrack.port = "5000" # ART port

    #_dtrack.stations[2] = avango.daemon.Station('tracking-art-glasses-1') # 3D-TV wireless shutter glasses
    _dtrack.stations[10] = avango.daemon.Station('tracking-art-glasses-1') # 3D-TV wireless shutter glasses
    _dtrack.stations[8] = avango.daemon.Station('tracking-art-glasses-2') # 3D-TV wired shutter glasses  
    #_dtrack.stations[5] = avango.daemon.Station('tracking-art-glasses-3') # passiv circular glasses

    _dtrack.stations[1] = avango.daemon.Station('tracking-art-pointer-1') # August
    _dtrack.stations[11] = avango.daemon.Station('tracking-art-pointer-2') # ednet

    device_list.append(_dtrack)
    print("ART Tracking started")


def init_pst_tracking():

    # create instance of DTrack
    _pst = avango.daemon.DTrack()
    _pst.port = "5020" # PST port

    _pst.stations[1] = avango.daemon.Station('tracking-pst-glasses-1')
    _pst.stations[2] = avango.daemon.Station('tracking-pst-pointer-1') # August

    device_list.append(_pst)

    print("PST Tracking started!")



### device functions
def init_spacemouse():

	_string = os.popen("/opt/avango/vr_application_lib/tools/list-ev -s | grep \"3Dconnexion SpaceNavigator\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

	if len(_string) == 0:
		_string = os.popen("/opt/avango/vr_application_lib/tools/list-ev -s | grep \"3Dconnexion SpaceTraveler USB\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

	_string = _string.split()
	if len(_string) > 0:

		_string = _string[0]
	
		_spacemouse = avango.daemon.HIDInput()
		_spacemouse.station = avango.daemon.Station('device-old-spacemouse') # create a station to propagate the input events
		_spacemouse.device = _string

		# map incoming spacemouse events to station values
		_spacemouse.values[0] = "EV_ABS::ABS_X"   # trans X
		_spacemouse.values[1] = "EV_ABS::ABS_Z"   # trans Y
		_spacemouse.values[2] = "EV_ABS::ABS_Y"   # trans Z
		_spacemouse.values[3] = "EV_ABS::ABS_RX"  # rotate X
		_spacemouse.values[4] = "EV_ABS::ABS_RZ"  # rotate Y
		_spacemouse.values[5] = "EV_ABS::ABS_RY"  # rotate Z

		# buttons
		_spacemouse.buttons[0] = "EV_KEY::BTN_0" # left button
		_spacemouse.buttons[1] = "EV_KEY::BTN_1" # right button

		device_list.append(_spacemouse)
		print("SpaceMouse started at:", _string)

	else:
		print("SpaceMouse NOT found!")


def init_new_spacemouse():

  _string = os.popen("python find_device.py 1 3Dconnexion SpaceNavigator").read()

  _string = _string.split()
  if len(_string) > 0:  
    _string = _string[0]
  
    # create a station to propagate the input events
    _spacemouse = avango.daemon.HIDInput()
    _spacemouse.station = avango.daemon.Station('device-new-spacemouse')
    _spacemouse.device = _string
    _spacemouse.timeout = '10'

    # map incoming spacemouse events to station values
    _spacemouse.values[0] = "EV_REL::REL_X"   # trans X
    _spacemouse.values[1] = "EV_REL::REL_Z"   # trans Y
    _spacemouse.values[2] = "EV_REL::REL_Y"   # trans Z
    _spacemouse.values[3] = "EV_REL::REL_RX"  # rotate X
    _spacemouse.values[4] = "EV_REL::REL_RZ"  # rotate Y
    _spacemouse.values[5] = "EV_REL::REL_RY"  # rotate Z

    # buttons
    _spacemouse.buttons[0] = "EV_KEY::BTN_0"  # left button
    _spacemouse.buttons[1] = "EV_KEY::BTN_1"  # right button

    device_list.append(_spacemouse)
    print("NewSpaceMouse started at:", _string)

  else:
    print("NewSpaceMouse NOT found !")



def init_keyboard():

    _string = os.popen("ls /dev/input/by-id | grep \"-event-kbd\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()  
    _string = _string.split()
  
    if len(_string) > 0:
        _string = _string[0]
        
        _keyboard = avango.daemon.HIDInput()
        _keyboard.station = avango.daemon.Station('device-keyboard')
        _keyboard.device = "/dev/input/by-id/" + _string
        
        _keyboard.buttons[0] = "EV_KEY::KEY_KPPLUS"
        _keyboard.buttons[1] = "EV_KEY::KEY_KPMINUS"
        _keyboard.buttons[2] = "EV_KEY::KEY_W"
        _keyboard.buttons[3] = "EV_KEY::KEY_A"
        _keyboard.buttons[4] = "EV_KEY::KEY_S"
        _keyboard.buttons[5] = "EV_KEY::KEY_D"
        _keyboard.buttons[6] = "EV_KEY::KEY_PAGEUP"
        _keyboard.buttons[7] = "EV_KEY::KEY_PAGEDOWN"
        _keyboard.buttons[8] = "EV_KEY::KEY_LEFT"
        _keyboard.buttons[9] = "EV_KEY::KEY_RIGHT"
        _keyboard.buttons[10] = "EV_KEY::KEY_UP"
        _keyboard.buttons[11] = "EV_KEY::KEY_DOWN"
        
        _keyboard.buttons[12] = "EV_KEY::KEY_1"
        _keyboard.buttons[13] = "EV_KEY::KEY_2"
        _keyboard.buttons[14] = "EV_KEY::KEY_3"     
        _keyboard.buttons[15] = "EV_KEY::KEY_4"
        _keyboard.buttons[16] = "EV_KEY::KEY_5"
        _keyboard.buttons[17] = "EV_KEY::KEY_6"
        _keyboard.buttons[18] = "EV_KEY::KEY_SPACE"
        _keyboard.buttons[19] = "EV_KEY::KEY_LEFTCTRL"
        
        device_list.append(_keyboard)
        
        print("Keyboard started at:", _string)
      
    else:
        print("Keyboard NOT found !")
		


def init_mouse():

	_string = os.popen("ls /dev/input/by-id | grep \"-event-mouse\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

	_string = _string.split()
	if len(_string) > 0:

		_string = _string[0]

		_mouse = avango.daemon.HIDInput()
		_mouse.station = avango.daemon.Station('device-mouse')
		_mouse.device = "/dev/input/by-id/" + _string
		_mouse.timeout = '14' # better !

		_mouse.values[0] = "EV_REL::REL_X"
		_mouse.values[1] = "EV_REL::REL_Y"

		_mouse.buttons[0] = "EV_KEY::BTN_LEFT"
		_mouse.buttons[1] = "EV_KEY::BTN_RIGHT"

		device_list.append(_mouse)
		print("Mouse started at:", _string)

	else:
		print("Mouse NOT found !")


## Initializes the August pointing device.
def init_august_pointer(DEVICE_STATION_STRING):

    _string = os.popen("python find_device.py 1 MOUSE USB MOUSE").read()        
    _string = _string.split()

    if len(_string) > 0:
        _string = _string[0]

        _pointer = avango.daemon.HIDInput()
        _pointer.station = avango.daemon.Station(DEVICE_STATION_STRING) # create a station to propagate the input events
        _pointer.device = _string
        #_pointer.timeout = '15'

        # map incoming events to station values
        _pointer.buttons[0] = "EV_KEY::KEY_F5" # front button
        #_pointer.buttons[0] = "EV_KEY::KEY_ESC" # front button
        _pointer.buttons[1] = "EV_KEY::KEY_PAGEUP" # center button
        _pointer.buttons[2] = "EV_KEY::KEY_PAGEDOWN" # back button

        device_list.append(_pointer)
        print('August Pointer found at:', _string)
    
    else:
        print("August Pointer NOT found !")


def init_ednet_pointer(DEVICE_STATION_STRING):

    _string = os.popen("python find_device.py 1 MOSART Semi. Input Device").read()               
    _string = _string.split()

    if len(_string) > 0:
        _string = _string[0]

        _pointer = avango.daemon.HIDInput()
        _pointer.station = avango.daemon.Station(DEVICE_STATION_STRING) # create a station to propagate the input events
        _pointer.device = _string
        #_pointer.timeout = '15'

        # map incoming events to station values
        _pointer.buttons[0] = "EV_KEY::KEY_PAGEUP" # front button
        _pointer.buttons[1] = "EV_KEY::KEY_B" # back button

        device_list.append(_pointer)
        print('Ednet Pointer found at:', _string)

    else:
        print("Ednet Pointer NOT found !")

def init_gyromouse():

	_string = os.popen("./list-ev -s | grep \"Gyration Gyration RF Technology Receiver\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()
	
	if len(_string) == 0:
	  _string = os.popen("./list-ev -s | grep \"Gyration GyroPoint RF Technology Receiver\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

	_string = _string.split()
	if len(_string) > 0:

		_string = _string[1]

		_gyromouse = avango.daemon.HIDInput()
		_gyromouse.station = avango.daemon.Station('device-gyromouse')
		_gyromouse.device = _string
		_gyromouse.timeout = '14' # better !

		_gyromouse.values[0] = "EV_REL::REL_X"
		_gyromouse.values[1] = "EV_REL::REL_Y"

		_gyromouse.buttons[0] = "EV_KEY::BTN_LEFT"
		_gyromouse.buttons[1] = "EV_KEY::BTN_RIGHT"

		device_list.append(_gyromouse)
		print("GyroMouse started at:", _string)

	else:
		print("GyroMouse NOT found !")


device_list = []

init_spacemouse()
init_new_spacemouse()
init_keyboard()
#init_mouse()
init_art_tracking()
init_pst_tracking()
init_august_pointer("device-pointer-1")
init_ednet_pointer("device-pointer-2")
init_gyromouse()


avango.daemon.run(device_list)


