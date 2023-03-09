#!/usr/bin/env python3

import urllib.request
import time
import datetime
import math
import calendar

class Axis:
    def __init__(self,server,uname=None,pwd=None):
        if uname is not None and pwd is not None:
            pass
            # pwd_manager = urllib2.HTTPPasswordMgrWithDefaultRealm()
            # pwd_manager.add_password(None,server,uname,pwd)

            # auth_handler = urllib2.HTTPDigestAuthHandler(pwd_manager)

            # opener = urllib2.build_opener(auth_handler)
            # urllib2.install_opener(opener)

        self.params = Params(server)
        self.ptz = PTZ(server)
        self.snapshot = Snapshot(server)
        self.temp = TempControl(server)

class APIGroup:
    def __init__(self,server,cmd_path):
        self.server = server
        self.cmd_path = cmd_path

    def cmd(self,params=[]):
        if type(params) == type(str()):
            p = params
        else:
            p = '&'.join(params)

        return urllib.request.urlopen(self.server+self.cmd_path+p)
        try:
            return urllib2.urlopen(self.server+self.cmd_path+p)
        except:
            return None

class Params(APIGroup):
    def __init__(self,server):
        APIGroup.__init__(self,server,'axis-cgi/param.cgi?')

    def definitions(self):
        return self.cmd(('action=listdefinitions','listformat=xmlschema')).read()

class TempControl(APIGroup):
    def __init__(self,server):
        APIGroup.__init__(self,server,'axis-cgi/temperaturecontrol.cgi?')

    def action(self,device,id,a,ts=None):
        params = ['device='+device,'id='+str(id),'action='+a]
        if ts is not None:
            params.append('timestamp='+str(ts))
        return self.cmd(params).read()
        
    def start(self,device,id):
        ts = calendar.timegm(datetime.datetime.utcnow().timetuple())
        return self.action(device,id,'start',ts)

    def stop(self,device,id):
        ts = calendar.timegm(datetime.datetime.utcnow().timetuple())
        return self.action(device,id,'stop',ts)
    

    def info(self):
        return self.cmd().read()

class Snapshot(APIGroup):
    def __init__(self,server):
        APIGroup.__init__(self,server,'axis-cgi/jpg/image.cgi?')
        
    def get(self,compression=None):
        if compression is not None:
            ret = self.cmd('compression='+str(compression))
        else:
            ret = self.cmd()
        if ret is not None:
            return ret.read()

    def save(self,fn,compression=None):
        data = self.get(compression)
        if data is None:
            return False
        out = open(fn,'wb')
        out.write(data)
        out.close()
        return True

class Position:
    def __init__(self,lines,ptz):
        self.timestamp = datetime.datetime.now()
        self.pan = None
        self.tilt = None
        self.zoom = None
        self.ptz = ptz

        for l in lines:
            l = l.decode('utf-8')
            parts = l.split('=',1)
            if parts[0] == 'pan':
                self.pan = float(parts[1])
            if parts[0] == 'tilt':
                self.tilt = float(parts[1])
            if parts[0] == 'zoom':
                self.zoom = ptz.coordToZoom(int(parts[1]))

    def __str__(self):
        return 'pan: {:.2f} tilt: {:.2f} zoom: {:.1f} ts: {} '.format(self.pan,self.tilt,self.zoom,self.timestamp.isoformat())

    def ete(self,pan,tilt,zoom,speed):
        time_left = 0.0
        
        pd = self.ptz.degreeDistance(pan, self.pan)
        if pd is not None:
            time_left = max(time_left,pd/speed)
        
        td = self.ptz.degreeDistance(tilt, self.tilt)
        if td is not None:
            time_left = max(time_left,td/speed)

        if self.zoom is not None and zoom is not None:
            zspeed = 0.35
            z1 = math.log10(self.zoom)
            z2 = math.log10(zoom)
            zd = abs(z1-z2)
            ztime = zd/zspeed
            time_left = max(time_left,ztime)

        return time_left

    def fov(self):
        max_fov = 55.2
        return max_fov/self.zoom,(max_fov*(720.0/1280.0))/self.zoom

class PTZ(APIGroup):
    def __init__(self,server):
        APIGroup.__init__(self,server,'axis-cgi/com/ptz.cgi?')
        self.minZoom = 1.0
        self.maxZoom = 18.0
        self.minZoomCoord = 1
        self.maxZoomCoord = 9999

        if server is not None:
            print(self.getPosition())
            limits = self.cmd('query=limits')
            for l in limits.read().decode('utf-8').split():
                print(l)



    def info(self):
        return self.cmd('info=1').read()
        ret = []
        for l in self.cmd('info=1').readlines():
            ret.append(l.strip())
        return ret

    def zoomToCoord(self, zoom):
        return self.minZoomCoord+int((self.maxZoomCoord-self.minZoomCoord)*
        (zoom - self.minZoom)/(self.maxZoom-self.minZoom))

    def coordToZoom(self, coord):
        return int(0.5+(self.minZoom+((self.maxZoom-self.minZoom)*(coord-self.minZoomCoord))/float(self.maxZoomCoord-self.minZoomCoord))*100)/100.0

    def speedToValue(self, speed):
        if speed <= 0.05:
            return 1
        if speed >= 450.0:
            return 100
        if speed <= 1.5:
            return int(pow(1340*(speed-0.045),0.366))
        return int(pow(speed/450.0,1/3.0)*100)

    def valueToSpeed(self, value):
        if value < 2:
            return 0.05
        elif value >= 100:
            return 450.0
        elif value > 16:
            return 450.0*pow(value/100.0,3)
        return pow(value,2.732)/1340+0.045

    def goto(self,pan=None,tilt=None,zoom=None,speed=None):
        params = []
        if pan is not None:
            params.append('pan='+str(pan))
        if tilt is not None:
            params.append('tilt='+str(tilt))
        if zoom is not None:
            params.append('zoom='+str(self.zoomToCoord(zoom)))
        if speed is not None:
            params.append('speed='+str(self.speedToValue(speed)))

        return self.cmd(params).read()

    def autoFocus(self,af=True):
        if af:
            c = 'autofocus=on'
        else:
            c = 'autofocus=off'
        return self.cmd(c).read()

    def focus(self,f):
        return self.cmd('focus='+str(f)).read()

    def rFocus(self,f):
        return self.cmd('rfocus='+str(f)).read()

    def autoIris(self,ai=True):
        if ai:
            c = 'autoiris=on'
        else:
            c = 'autoiris=off'
        return self.cmd(c).read()
            
    def iris(self,i):
        i = max(0,min(9999,i))
        return self.cmd('iris='+str(i)).read()

    def irFilter(self,ir=True):
        if ir:
            c = 'ircutfilter=on'
        else:
            c = 'ircutfilter=off'
        return self.cmd(c).read()
                
    def stop(self):
        p = self.getPosition()
        self.goto(p[0],p[1],p[2],100)

    def normalizeDegrees(self,d):
        if d < -180 or d > 180:
            ret = d%360.0
            if ret > 180.0:
                return ret - 360.0
            return ret
        return d

    def degreeDistance(self,a,b):
        if a is None or b is None:
            return None
        return abs(self.normalizeDegrees(abs(a-b)))

    def gotoWait(self,pan=None,tilt=None,zoom=None,speed=None):
        self.goto(pan,tilt,zoom,speed)
        arrived = False

        actual_speed = self.getSpeed()

        last_ete = None
        while not arrived:
            pos = self.getPosition()

            time_left = pos.ete(pan,tilt,zoom,actual_speed)

            if time_left > 0.0:
                time.sleep(time_left)
                if time_left < 0.1:
                    arrived = True
            else:
                arrived = True
                
                
            if last_ete == time_left:
                stuck += 1
            else:
                stuck = 0
            last_ete = time_left

            if stuck > 20:
                return False
        return True

    def getPosition(self):
        ret = self.cmd('query=position')
        if ret is not None:
            self.position = Position(ret.readlines(),self)
            return self.position

    def getSpeed(self):
        k,v = self.cmd('query=speed').read().split('=',1)
        return self.valueToSpeed(int(v))
        
    def center(self, x, y):
        return self.cmd("center="+str(int(x))+','+str(int(y))).read()
