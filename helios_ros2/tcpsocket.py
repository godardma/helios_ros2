#-*- coding: utf-8 -*-
import socket
import time

class TcpSocket(object):
    """
    """
    def __init__(self,ip,port,server = False):
        """
        """
        self.ip = str(ip)
        self.port = int(port)
        self.server = bool(server)
        #
        self.socket = None
        self.commSocket = None
        self.commAddr = None
        #
        self.serverSocket = None
        #
        self.connected = False

    def initSocket(self):
        sock = socket.socket(socket.AF_INET,socket.SOCK_STREAM)
        sock.settimeout(0.001)
        sock.setsockopt(socket.SOL_SOCKET,socket.SO_REUSEADDR,1)
        if self.server is True:
            #print (self.ip,self.port)
            sock.bind((self.ip,self.port))
            sock.listen(0)
            self.serverSocket = sock
        else:
            self.socket = sock

    def connect(self):
        """
        """
        if self.socket is None:
            self.initSocket()
        if self.connected is False:
            try:
                self.socket.connect((self.ip,self.port))
            except socket.error:
                self.connected = False
            else:
                self.connected = True

    def accept(self):
        """
        """
        if self.serverSocket is None:
            self.initSocket()
        if self.connected is False:
            try:
                self.socket,self.commAddr = self.serverSocket.accept()
            except socket.timeout:
                self.connected = False
            else:
                self.socket.settimeout(0.001)
                self.connected = True

    def keep(self):
        """
        """
        if not self.connected:
            if self.server:
                self.accept()
            else:
                self.connect()

    def send(self,msg):
        if not self.connected:
            return
        #
        buf = msg
        while len(buf) != 0:
            try:
                n = self.socket.send(buf)
                buf = buf[n:]
            except socket.error:
                self.socket.close()
                self.socket = None
                self.connected = False
                break

    def receive(self):
        if not self.connected:
            return
        #
        buf = None
        try:
            buf = self.socket.recv(4096)
        except socket.error:
            pass
        finally:
            if buf == b'':
                print('buf is empty')
                self.socket.close()
                self.socket = None
                self.connected = False
            elif buf is None:
                buf = ''
        #
        return buf

    def isConnected(self):
        return self.connected

if __name__ == '__main__':
    #def __init__(self,ip,port,server = False):
    server = TcpSocket('localhost',5000,server = True)
    while True:
        server.keep()
        if server.isConnected():
            buf = server.receive()
            if buf is not None and buf != '':
                print(buf)
        time.sleep(0.1)
