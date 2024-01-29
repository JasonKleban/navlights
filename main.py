import gc
import uasyncio
import utime
import machine
import math
import neopixel
import sys
from bno055 import BNO055
import ubinascii, uhashlib
import os
from perlin import Perlin2D

boottime = utime.time()
print("HELLO")

pixelCount = 64 + 11
frameCount = 512
forward = 32

cellsHigh = 16
cellsWide = 4
fps = 24

brightness = .05

navLightWidth = 8

def dull(r, g, b):
    return (int(brightness * r), int(brightness * g), int(brightness * b)) 

palette = [  dull(*(0,0,36))
            ,dull(*(4,2,57))
            ,dull(*(6,5,80))
            ,dull(*(8,7,104))
            ,dull(*(9,9,121))
            ,dull(*(53,58,148))
            ,dull(*(97,107,175))
            ,dull(*(136,151,200))
            ,dull(*(180,201,228))
            ,dull(*(224,250,255))
            ,dull(*(0,0,0))]

def lineReader(uart, buffersize=240):
    buf = memoryview(bytearray(buffersize))
    e = 0
    while True:
        notyielded = True
        s, e = e, e + (uart.readinto(buf[e:]) or 0)
        while s < e:
            s += 1
            if buf[s-1] == ord('\n'):
                yield bytes(buf[:s])
                buf[:e-s] = buf[s:e]
                s, e = 0, e-s
                notyielded = False
        if e == len(buf):
            raise BufferError('Overflow')
            e = 0
        elif notyielded:
            yield None

def getReadLoop():
    # adc = machine.ADC(2)
    # adc.atten(machine.ADC.ATTN_2_5DB)
    # adc.width(machine.ADC.WIDTH_12BIT)
    
    np = neopixel.NeoPixel(machine.Pin(3), pixelCount)
    
    i2c = machine.SoftI2C(scl=Pin(7), sda=Pin(6), freq=400000)
    
    calibrated = False
    imu = BNO055(i2c)
    imu.set_offsets(bytearray(ubinascii.unhexlify('f3ff1a00deffc3ff9dfde3fd03000100ffffe8031e04')))
    
    gpsModule = lineReader(machine.UART(1, baudrate=9600, timeout=0, tx=21, rx=20))

    perlin = Perlin2D(cellsHigh, cellsWide)

    async def readLoop():
        nonlocal np, imu, calibrated, gpsModule, perlin
        
        lastMS = utime.ticks_ms()
        time = status = lat = ns = lng = ew = None
        speed = trackTrue = date = mag = mew = None
        alt = au = None
        buff = None
        datetime = None
        headed = None

        while True:            
            try:                
                if not calibrated:
                    nextCalibrated = imu.calibrated()
                    
                    if calibrated != nextCalibrated:
                        calibrated = nextCalibrated
                        print(hex(int(ubinascii.hexlify(imu.sensor_offsets()).decode(), 16)))
                
                while True:
                    
                    orientationVec = imu.euler()
                    gravityVec = imu.gravity()
                    accelVec = imu.lin_acc()
                    compassVec = imu.mag()
                    deltaMS = utime.ticks_diff(utime.ticks_ms(), lastMS); lastMS = utime.ticks_ms()
                    buff = next(gpsModule)
                    
                    gravityMag = math.sqrt(gravityVec[0] * gravityVec[0] + gravityVec[1] * gravityVec[1] + gravityVec[2] * gravityVec[2]) or None
                    gravityNormalized = (gravityVec[0] / gravityMag, gravityVec[1] / gravityMag, gravityVec[2] / gravityMag) if gravityMag is not None else None
                                        
                    compassMag = math.sqrt(compassVec[0] * compassVec[0] + compassVec[1] * compassVec[1] + compassVec[2] * compassVec[2]) or None
                    northNormalized = (compassVec[0] / compassMag, compassVec[1] / compassMag, compassVec[2] / compassMag) if compassMag is not None else None
                    
                    eastNormalized = ((gravityNormalized[1] * northNormalized[2] - gravityNormalized[2] * northNormalized[1], 
                        gravityNormalized[2] * northNormalized[0] - gravityNormalized[0] * northNormalized[0], 
                        gravityNormalized[0] * northNormalized[1] - gravityNormalized[1] * northNormalized[0]) 
                        if gravityNormalized is not None and northNormalized is not None else None)

                    # TODO
                    
                    accelUpMs2 = (-(gravityNormalized[0] * accelVec[0] + gravityNormalized[1] * accelVec[1] + gravityNormalized[2] * accelVec[2])
                        if gravityNormalized is not None else None)
                    accelNorthMs2 = ((northNormalized[0] * accelVec[0] + northNormalized[1] * accelVec[1] + northNormalized[2] * accelVec[2])
                        if northNormalized is not None else None)
                    accelEastMs2 = ((eastNormalized[0] * accelVec[0] + eastNormalized[1] * accelVec[1] + eastNormalized[2] * accelVec[2])
                        if eastNormalized is not None else None)

                    if accelUpMs2 is not None and accelNorthMs2 is not None and accelEastMs2 is not None:
                        #print('{:+0.1f}m/s² Up, {:+0.1f}m/s² North {:+0.1f}m/s² East'.format(
                        #   accelUpMs2, accelNorthMs2, accelEastMs2))
                        print('{:+0.1f}m/s² Up'.format(accelUpMs2))
                        
                    
                    # Length in km of 1° of latitude = always 111.32 km
                    # Length in km of 1° of longitude = 40075 km * cos( latitude ) / 360
                    
                    # Predict Kalman Filter
                    
                    # TODO
                    
                    # Update Kalman Filter
                    
                    if buff is None: break
                    
                    if buff.startswith('$GPRMC,'):
                    
                        parts = buff.decode('ascii').strip().split(',')
                        
                        _, time, status, lat, ns, lng, ew, speed, trackTrue, date, mag, mew, *_ = parts
                        
                        if status != 'A': continue
                                        
                        datetime = '20{}-{}-{} {}:{}:{} UTC'.format(date[4:], date[2:4], date[:2], time[:2], time[2:4], time[4:6])
                        trackTrue = 0.0 if trackTrue is not None and trackTrue.strip() == '' else float(trackTrue)
                        mag = 0.0 if mag is not None and mag.strip() == '' else float(mag)
                        speed = 0.0 if speed is not None and speed.strip() == '' else float(speed)
                        
                    elif buff.startswith('$GPGGA,'):
                        parts = buff.decode('ascii').strip().split(',')
                        
                        _, _, _, _, _, _, _, _, _, alt, au, *_ = parts
                    else:
                        continue
                    
                heading = round(orientationVec[0])
                heading = 0 if heading == 360 else heading
                
                headed = (forward - round((heading / 360.0) * pixelCount)) % pixelCount
                
                #print('{:03}º pixel # {}'.format(heading, headed))
                    
#                     print('{}: {}º{}{} {}º{}{} Tracking {:4.1f}º {} off {:05.1f}º at {:3.1f} kts, Heading {:03}º'
#                           .format(
#                               datetime,
#                               lat[:-8], lat[-8:], ns, lng[:-8], lng[-8:], ew,
#                               mag, mew,
#                               trackTrue,
#                               speed,
#                               heading))

                f = (lastMS / 24) % frameCount

                for l in range(pixelCount):
                    np[l % pixelCount] = (
                        (0,255,0) if headed in ((l + x) % pixelCount for x in range(0, -navLightWidth, -1)) else
                        (255,0,0) if headed in ((l + x) % pixelCount for x in range(1, 1 + navLightWidth)) else
                        (0,0,0) if headed in (((l - navLightWidth) % pixelCount), ((l + 1 + navLightWidth) % pixelCount)) else
                            palette[
                                math.ceil(
                                ((perlin.getValue(
                                    (f / frameCount) * cellsHigh,
                                    (l / pixelCount) * cellsWide
                                ) + 1) / 2) * len(palette)) ])
                
                np.write()

            except OSError as e:
                print(e)

            await uasyncio.sleep_ms(0)

    return readLoop

try:   
    loop = uasyncio.get_event_loop()

    def _handle_exception(loop, context):
        sys.print_exception(context["exception"])
        loop.stop()

    loop.set_exception_handler(_handle_exception)
    
    readLoop = getReadLoop()
    loop.create_task(readLoop())
    
    loop.run_forever()
    loop.close()
finally:
    print("GOODBYE")
    # uart = machine.UART(0, 115200)
    # os.dupterm(uart, 1)
    uasyncio.new_event_loop()  # Clear uasyncio stored state