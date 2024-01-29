try:
  import usocket as socket
except:
  import socket

from machine import Pin
import network

import esp
esp.osdebug(None)

import gc
gc.collect()

import micropython
micropython.alloc_emergency_exception_buf(100)

ssid = ''
password = ''

#station = network.WLAN(network.STA_IF)

#station.active(True)
#station.connect(ssid, password)

#while station.isconnected() == False:
#  pass

#print('Connection successful!')
#print(station.ifconfig())


