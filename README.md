
# Nav Lights Hat

(An amatuerish) hardware project incorporating an ESP32S3, a NEO-6M GPS Flight Controller Module, a BNO055 9-axis absolute orientation sensor and a strip of neopixels arranged in sequence in a single loop on the circumerence of a hat brim.

The intention of this project is to visually indicate the speed and direction of travel of the device, displayed at the direction of the inferred course-over-ground angle and with a marine-style red/green/white port/starboard/aft indicators with a width proportional to the inferred speed-over-ground.  When the speed is near zero and no significant course can be determined, the width of the indicators shrinks to zero.  A "background" animation seamless 2D tile is generated as perlin noise, animated as 1D pixel frame over 1D time with the animation rotated on the neopixel loop dynamically to maintain an orientation fixed relative to magnetic north.  As the speed-over-ground increases, the background animation should dull to zero brightness.

The silly application of this as a wearable device is to serve as nav lights when operating a dinghy/tender boat at night, a low-maintenance and self-contained unit that at least approximates the required equipment and allowing the wearer to swivel their head to maintain full 360-degree awareness.

The Orientation Sensor provides compass and IMU readings.  The GPS sensor provides absolute position and velocity.  Compass readings allow the device orientation to be inferred.  IMU readings are supposed to refine position and orientation inference through dead-reckoning. 

I believe there are a still a few bugs in the orientation tracking.  The directional indicators sometimes appear when the device is at rest and do not consistently point "forward" during motion tests.