#!/usr/bin/env python

import sys
import os
import BaseHTTPServer
from SimpleHTTPServer import SimpleHTTPRequestHandler
import rospkg
import rospy


def server():
    rospy.init_node('rws_pr2_navigation_http_server')

    rospack = rospkg.RosPack()

    os.chdir(os.path.join(rospack.get_path('rws_pr2_navigation'), "www"))

    HandlerClass = SimpleHTTPRequestHandler
    ServerClass = BaseHTTPServer.HTTPServer
    Protocol = "HTTP/1.0"

    port = 8000
    server_address = ('0.0.0.0', port)

    HandlerClass.protocol_version = Protocol
    httpd = ServerClass(server_address, HandlerClass)

    sa = httpd.socket.getsockname()
    print "Serving HTTP on", sa[0], "port", sa[1], "..."
    httpd.serve_forever()

if __name__ == '__main__':
    try:
        server()
    except rospy.ROSInterruptException: pass