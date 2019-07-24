Raspberry Pi Stills Node
------------------------

This node was created to be an alternative to [raspicam_node](https://github.com/UbiquityRobotics/raspicam_node)
which works for video, but therefore has faster framerates and lower resolutions than the pi camera is capable of in still mode.
Therefore I adapted the code of raspistill to send out higher res (2592x1944) at a slow rate.

This is very much hacked together for a specific use (finding QR codes for landmarks for a robot) so as such it is not 
flexible at all. It also has not been tested on many different Pi's. YMMV.
