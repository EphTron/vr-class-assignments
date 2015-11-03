#!/usr/bin/python

import avango.daemon
import os
import sys

# functions
def init_spacemouse():

    new_spacemouse_string = os.popen("/opt/avango/vr_application_lib/tools/list-ev -s | grep \"3Dconnexion SpaceNavigator for Notebooks\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()
    old_spacemouse_string = os.popen("/opt/avango/vr_application_lib/tools/list-ev -s | grep \"3Dconnexion SpaceNavigator\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

    new_spacemouse_string = new_spacemouse_string.split()
    old_spacemouse_string = old_spacemouse_string.split()

    if len(new_spacemouse_string) > 0:

        ## new spacemouse was found ##
        
        string = new_spacemouse_string[0]

        spacemouse = avango.daemon.HIDInput()
        spacemouse.station = avango.daemon.Station('gua-device-spacemouse') # create a station to propagate the input events
        spacemouse.device = string
        spacemouse.timeout = '14' # better !
        spacemouse.norm_abs = 'True'

        # map incoming spacemouse events to station values
        spacemouse.values[0] = "EV_REL::REL_X"   # trans X
        spacemouse.values[1] = "EV_REL::REL_Z"   # trans Y
        spacemouse.values[2] = "EV_REL::REL_Y"   # trans Z
        spacemouse.values[3] = "EV_REL::REL_RX"  # rotate X
        spacemouse.values[4] = "EV_REL::REL_RZ"  # rotate Y
        spacemouse.values[5] = "EV_REL::REL_RY"  # rotate Z

        # buttons
        spacemouse.buttons[0] = "EV_KEY::BTN_0" # left button
        spacemouse.buttons[1] = "EV_KEY::BTN_1" # right button

        device_list.append(spacemouse)
        print("New SpaceMouse started at:", string)


    elif len(old_spacemouse_string) > 0:

        ## old spacemouse was found ##

        string = old_spacemouse_string[0]
    
        spacemouse = avango.daemon.HIDInput()
        spacemouse.station = avango.daemon.Station('gua-device-spacemouse') # create a station to propagate the input events
        spacemouse.device = string

        # map incoming spacemouse events to station values
        spacemouse.values[0] = "EV_ABS::ABS_X"   # trans X
        spacemouse.values[1] = "EV_ABS::ABS_Z"   # trans Y
        spacemouse.values[2] = "EV_ABS::ABS_Y"   # trans Z
        spacemouse.values[3] = "EV_ABS::ABS_RX"  # rotate X
        spacemouse.values[4] = "EV_ABS::ABS_RZ"  # rotate Y
        spacemouse.values[5] = "EV_ABS::ABS_RY"  # rotate Z

        # buttons
        spacemouse.buttons[0] = "EV_KEY::BTN_0" # left button
        spacemouse.buttons[1] = "EV_KEY::BTN_1" # right button

        device_list.append(spacemouse)
        print("SpaceMouse started at:", string)

    else:
        print("SpaceMouse NOT found!")


def init_keyboard():

    keyboard_name = os.popen("ls /dev/input/by-id | grep \"-event-kbd\" | sed -e \'s/\"//g\'  | cut -d\" \" -f4").read()

    keyboard_name = keyboard_name.split()

    for i, name in enumerate(keyboard_name):

        keyboard = avango.daemon.HIDInput()
        keyboard.station = avango.daemon.Station('gua-device-keyboard' + str(i))
        keyboard.device = "/dev/input/by-id/" + name


        keyboard.buttons[0] = "EV_KEY::KEY_W"
        keyboard.buttons[1] = "EV_KEY::KEY_A"
        keyboard.buttons[2] = "EV_KEY::KEY_S"
        keyboard.buttons[3] = "EV_KEY::KEY_D"
        keyboard.buttons[4] = "EV_KEY::KEY_LEFT"
        keyboard.buttons[5] = "EV_KEY::KEY_RIGHT"
        keyboard.buttons[6] = "EV_KEY::KEY_UP"
        keyboard.buttons[7] = "EV_KEY::KEY_DOWN"
        keyboard.buttons[8] = "EV_KEY::KEY_Q"
        keyboard.buttons[9] = "EV_KEY::KEY_E"
        keyboard.buttons[10] = "EV_KEY::KEY_PAGEUP"
        keyboard.buttons[11] = "EV_KEY::KEY_PAGEDOWN"
        keyboard.buttons[12] = "EV_KEY::KEY_KPPLUS"
        keyboard.buttons[13] = "EV_KEY::KEY_KPMINUS"
        

        device_list.append(keyboard)
        print("Keyboard " + str(i) + " started at:", keyboard_name)



device_list = []

init_spacemouse()
init_keyboard()

avango.daemon.run(device_list)
