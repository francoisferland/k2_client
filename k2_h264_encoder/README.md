h264_encoder
------------

This project is not meant as a ROS package but a standalone executable that
listens to the output of k2_server, encode the images as a H.264 stream, and
sends them over a new TCP socket.
It is meant to run on the same machine as k2_server.
