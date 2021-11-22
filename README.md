# ESP 8266 heat camera (with AMG8833)

This code extends the [Thermal Cam Interpolate](https://github.com/adafruit/Adafruit_AMG88xx/tree/master/examples/thermal_cam_interpolate) example and adds temporal smoothing and auto scaling of min-max values.

We measure every 200ms, and smooth the interpolated image by taking the average of the last 4 frames. This reduces the jumpyness a bit.

It also keeps track of the minimal and maximal values for the past 2 seconds, and auto scales the image based on those ranges (instead of a fixed range between 20 and 30).
