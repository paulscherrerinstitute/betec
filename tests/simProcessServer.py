from __future__ import print_function

import random
import re
import math
import socket
import sys
import threading
import time
import traceback

# end of line
IOS = '\r\n'

class Axis(object):
    def __init__(self, number):
        self.number = number
        self.type = 0
        self.scale = 1000
        self.velocity = 1
        self.unit = 'deg'
        self.steps = 0
        self.position = 0
        self.moving = 0
        self.low_limit = 0
        self.high_limit = 0
        self.stabilized = 1
        self.aux1 = 0
        self.aux2 = 0
        self.tid = None
        self.stopping = False
        self.callbacks = []

    def move(self, target, velocity):
        speed = 1 * velocity
        total_steps = max(abs(target - self.position) / speed, 1)
        delta = (target - self.position) / total_steps
        self.moving = 1

        def simulate_move():
            while not self.stopping:
                self.position += delta
                if (delta >= 0 and self.position >= target) or \
                   (delta < 0 and self.position < target):
                    self.position = target
                    break
                time.sleep(1)
            self.moving = 0
            self.tid = None
            if self.stopping:
                self.notify('AXIS MOTION STOPPED:%d' % (self.number+1))
            else:
                self.notify('AXIS MOTION FINISHED:%d' % (self.number+1))

        self.tid = threading.Thread(target=simulate_move)
        self.stopping = False
        self.tid.start()

    def stop(self):
        self.stopping = True

    def subscribe(self, callback):
        self.callbacks.append(callback)

    def unsubscribe(self, callback):
        self.callbacks.remove(callback)
    
    def notify(self, message):
        for callback in self.callbacks:
            callback(message)

    def getPosition(self):
        return self.position + random.random()/1000.

    def getState(self):
        return '%d %d %f %d %d %d %d %d %d' % (
                self.number+1,
                self.steps + int(random.random() * 10),
                self.getPosition(),
                self.moving,
                self.low_limit,
                self.high_limit,
                self.stabilized,
                self.aux1,
                self.aux2)

class Mono(object):
    def __init__(self):
        self.energy = 100
        self.mirror = 0
        self.grating = 1
        self.grating_list = [(-10,800), (-5,1000), (0,1500), (5,2400)]
        self.monoState = 0
        self.lineDensity = 800.0
        self.diffOrder = 1
        self.cff = 2.15
        self.globalMotion = 0
        self.globalStabilization = 0
        self.globalLimitEncoderError = 0
        self.globalMotionError = 0
        self.globalMotionErrorDesc = 0

    def getPose(self):
        energy = self.getEnergy(axes[1].getPosition(), axes[0].getPosition())
        return '%f %f %d %f %d' % (
                1239.8 / energy,
                energy,
                self.grating,
                self.mirror,
                any(axis.moving for axis in axes))

    def getState(self):
        return '%d %d %d %d %d' % (
                any(axis.moving for axis in axes),
                self.globalStabilization,
                self.globalLimitEncoderError,
                self.globalMotionError,
                self.globalMotionErrorDesc)

    def setEnergy(self, energy):
        la = 2.9979e8 / (energy * 1.602176634) * 6.6261e-15
        lq = self.lineDensity * 1e3 * self.diffOrder * la
        B = lq * self.cff / (1 - self.cff**2)
        cosa = B/self.cff + math.sqrt(B**2 + 1)
        cosb = B*self.cff + math.sqrt(B**2 + 1)
        alpha = math.acos(cosa) / math.pi * 180
        beta = math.acos(cosb) / math.pi * 180
        theta = (alpha + beta) / 2

        return beta, theta

    def getEnergy(self, beta, theta):
        alpha = 2 * theta - beta
        lq = math.cos(alpha/180.*math.pi) - math.cos(beta/180.*math.pi)
        la = lq / (self.lineDensity * 1e3 * self.diffOrder)
        energy = 2.9979e8 / (la * 1.602176634) * 6.6261e-15
        return energy

class NotifyThread(threading.Thread):
    def __init__(self):
        threading.Thread.__init__(self)
        self.client = None
        self.daemon = True

    def run(self):
        while True:
            if self.client is None or \
                not self.client.enable_notify:
                time.sleep(1)
                continue

            try:
                self.client.notify()
            except:
                traceback.print_exc()
                self.client = None
            else:
                time.sleep(self.client.notify_interval)

mono = Mono()
axes = [Axis(0), Axis(1), Axis(2), Axis(3)]

notification = NotifyThread()
notification.start()

class ClientThread(threading.Thread):
    def __init__(self, client):
        threading.Thread.__init__(self)
        self.client = client
        self.daemon = True
        for axis in axes:
            axis.subscribe(self.callback)
        self.lock = threading.Lock()
        self.enable_notify = 0
        self.notify_interval = 0.1

    def send(self, msg):
        self.client.send(msg.encode('ascii'))

    def notify(self):
        self.lock.acquire()
        self.send('AUTO POSE:M %s%s' % (mono.getPose(), IOS))
        for axis in axes:
            self.send('AXISSTATE:%s%s' % (axis.getState(), IOS))
        self.send('AUTO STATE:%s%s' % (mono.getState(), IOS))
        self.lock.release()

    def callback(self, message):
        self.lock.acquire()
        self.send(message + IOS)
        self.lock.release()

    def sendLater(self, message, axes):
        def send():
            while True:
                time.sleep(1)
                if all(not axis.moving for axis in axes):
                    self.send(message + IOS)
                    break
        t = threading.Thread(target=send)
        t.setDaemon(True)
        t.start()

    def run(self):
        f = self.client.makefile()

        while True: 
            try:
                line = f.readline().rstrip(IOS)
            except:
                break
            if not line:
                break

            parts = line.split(':')
            if len(parts) == 2:
                command, param = parts
            else:
                command = parts[0]
                param = ''
            print('')
            print(time.time(),'INP', line)
            reply = None
            if command == 'AXIS STOP':
                axis = int(param)
                axes[axis-1].stop()
                reply = command + ':y ' + str(axis) + IOS
            elif command == 'AXIS MOVEUNITSABS':
                parts = param.split()
                axis = int(parts[0])
                target = float(parts[1])
                if len(parts) == 3:
                    velocity = float(parts[2])
                else:
                    velocity = 1
                reply = command + ':y ' + str(axis) + IOS
                axes[axis-1].move(target, velocity)
            elif command == 'AXIS MOVEUNITSDELTA':
                parts = param.split()
                axis = int(parts[0])
                delta = float(parts[1])
                if len(parts) == 3:
                    velocity = float(parts[2])
                else:
                    velocity = 1
                reply = command + ':y ' + str(axis) + IOS
                axes[axis-1].move(axes[axis-1].position + delta, velocity)
            elif command == 'SET ENERGY':
                mono.energy = float(param)
                reply = command + ':y ' + param + IOS
                f.flush()
                self.sendLater('ENERGY:' + param, (axes[0], axes[1]))
                beta, theta = mono.setEnergy(mono.energy)
                axes[0].move(theta, 1)
                axes[1].move(beta, 1)
            elif command == 'SET GRATINGNR':
                parts = param.split()
                mono.grating = int(parts[0])
                reply = command + ':y ' + param + IOS
                f.flush()
                pos, density = mono.grating_list[mono.grating-1]
                axes[3].move(pos, 1)
                self.sendLater('GRATINGNR:' + param, (axes[3],))
                mono.lineDensity = density
            elif command == 'GET GRATINGNR':
                reply = 'GRATINGNR:%d%s' % (mono.grating, IOS)
                f.flush()
            elif command == 'SET MTPOS':
                mono.mirror = float(param)
                reply = command + ':y ' + param + IOS
                axes[2].move(mono.mirror, 1)
                self.sendLater('MTPOS:' + param, (axes[2],))
            elif command == 'SET STABSTATE':
                stabState = int(param)
                reply = ommand + ':y ' + param + IOS
            elif command == 'SET GRIDPARAMETERS':
                parts = param.split()
                mono.lineDensity = float(parts[0])
                mono.diffOrder = int(parts[1])
                mono.cff = float(parts[2])
                reply = command + ':y ' + param + IOS
            elif command == 'GET GRIDPARAMETERS':
                reply = 'GRIDPARAMETERS:%f %d %f%s' %\
                        (mono.lineDensity, mono.diffOrder, mono.cff, IOS)
            elif command == 'GET AXISPARAMS':
                axis = int(param)
                reply = 'AXISPARAMS:%d %d %d %d %s%s' % (axis,
                        axes[axis-1].type, axes[axis-1].velocity,
                        axes[axis-1].scale, axes[axis-1].unit, IOS)
            elif command == 'GET AXISSTATE':
                axis = int(param)
                reply = 'AXISSTATE:' + axes[axis-1].getState() + IOS
            elif command == 'GET STATE':
                reply = 'STATE ' + mono.getState() + IOS
            elif command == 'GET POSE':
                if param == 'M':
                    reply = 'POSE:M ' + mono.getPose() + IOS
            elif command == 'SET REQUESTINTERVAL':
                self.notify_interval = float(param)/1000.
                reply = command + ':y ' + param + IOS
            elif command == 'GET REQUESTINTERVAL':
                reply = 'REQUESTINTERVAL:%d' % int(self.notify_interval * 1000) + IOS
            elif command == 'ENABLE NOTIFY':
                self.enable_notify = int(param)
                reply = 'ENABLE NOTIFY:y' + IOS
                if self.enable_notify:
                    notification.client = self
                else:
                    notification.client = None
            elif command == 'USER LOG IN':
                reply = 'USER LOG IN:y USERSTATE:1 EXPERT' + IOS
            elif command == 'USER LOG OUT':
                'USE LOG OUT:y USERSTATE:5 VIEWER' + IOS
            elif command == 'SYSTEM STOP':
                reply = 'SYSTEM IS STOPPED' + IOS
            else:
                print('Unknow command %s' % command)
                continue
            self.lock.acquire()
            print(time.time(),'OUT', reply)
            self.send(reply)
            self.lock.release()

        f.close() # close socket


if __name__ == '__main__':
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
    server.bind(('localhost', 45401))
    server.listen(5)
    while True:
        # wait for next client to connect
        client, address = server.accept()
        print('new client from', address)
        thread = ClientThread(client)
        thread.start()

