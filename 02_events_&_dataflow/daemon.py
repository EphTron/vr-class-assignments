#!/usr/bin/python

### import guacamole libraries
import avango
import avango.daemon
import os


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
		


device_list = []

init_keyboard()

avango.daemon.run(device_list)
