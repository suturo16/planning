#!/usr/bin/env python
     
import xmlrpclib

"""
Saving the server for later use.
"""
server = xmlrpclib.Server('http://134.102.162.240:7080/')


"""
The main class. This will run as soon as the script is used, on pepper you
want to pack the server commands on the onStart method, not here.
"""
if __name__ == '__main__':          
  server.echo("right")
  pass
