import cv2
import time
import serial
import numpy as np
import RPi.GPIO as GPIO
from gpiozero import LED
#import matplotlib.pyplot as pltfrom

class Arduino():
    def __init__(self):
        self.ser = serial.Serial('/dev/ttyUSB0')  # open serial port
        print(self.ser.name)         # check which port was really used
    
    def getLine(self):
        aux, p = '', ''
        while p != '\n':
            aux += p
            p = self.ser.read().decode()
        return aux
            
    def close(self):
        self.ser.close()

class RaspBerry():
    def __init__(self):
        GPIO.setmode(GPIO.BCM)
        GPIO.setwarnings(False)
        self.led = LED(17)
        self.act = LED(16)
        
        self.led.off()
        self.act.off()
        
        self.flag = True #Next state
        
        GPIO.setup(12, GPIO.OUT)
        GPIO.setup(18, GPIO.OUT)
        
        self.keyst = GPIO.PWM(12, 100) #7, 21
        self.lente = GPIO.PWM(18, 100) #6, 20
        
        self.k = 16
        self.l = 16
        
        self.keyst.start(self.k)
        self.lente.start(self.l)
    
    def PWMKeyst(self,x):
        if x < 2.36:
            self.k = -0.178182*x + 16.586438
        else:
            self.k = -0.173347*x + 15.006346
            #y = -0.1758242*x + 15.3461538
        print('Keyst',x,self.k)
        self.act.on()
        self.keyst.ChangeDutyCycle(self.k)
        time.sleep(1)
        self.act.off()

    def PWMLente(self,x):
        self.l = -0.00000055050*x**3 + 0.00046548998*x**2 - 0.13211757420*x + 23.45524312608
        print('Lente',x,self.l)
        self.act.on()
        self.lente.ChangeDutyCycle(self.l)
        time.sleep(1)
        self.act.off()
    
    def ChangePWM(self,A):
        if A == 'LU':
            self.l = min(21,self.l + 0.5)
            self.act.on()
            self.lente.ChangeDutyCycle(self.l)
            time.sleep(.3)
            self.act.off()
            print('Lente:', self.l)
        elif A == 'LD':
            self.l = max(10,self.l - 0.5)
            self.act.on()
            self.lente.ChangeDutyCycle(self.l)
            time.sleep(.3)
            self.act.off()
            print('Lente:', self.l)
        elif A == 'KL':
            self.k = min(22,self.k + 0.5)
            self.act.on()
            self.keyst.ChangeDutyCycle(self.k)
            time.sleep(.3)
            self.act.off()
            print('Keyst:', self.k)
        elif A == 'KR':
            self.k = max(10,self.k - 0.5)
            self.act.on()
            self.keyst.ChangeDutyCycle(self.k)
            time.sleep(.3)
            self.act.off()
            print('Keyst:', self.k)
    
    def swapHDMI(self):
        if self.flag:
            self.led.on()
        else:
            self.led.off()
        self.flag = not self.flag
    
    def putRasp(self):
        self.led.on()
        self.flag = False
    
    def putLap(self):
        self.led.off()
        self.flag = True

def Camaras():
    # Camara izquierda
    cap1 = cv2.VideoCapture(0)
    cap1.set(3, 1280)
    cap1.set(4, 960)
    cap1.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))

    # Camara derecha
    cap2 = cv2.VideoCapture(2)
    cap2.set(3, 1280)
    cap2.set(4, 960)
    cap2.set(cv2.CAP_PROP_FOURCC,cv2.VideoWriter_fourcc('M','J','P','G'))
    
    ret1, frameL = cap1.read()
    cap1.release()
    del cap1
    ret2, frameR = cap2.read()
    cap2.release()
    del cap2
    frameR = cv2.rotate(frameR, cv2.ROTATE_180)

    sclp = 0.1
    wth = int(1280* sclp)
    hht = int(960 * sclp)

    framel = cv2.resize(frameL, (wth, hht))
    framer = cv2.resize(frameR, (wth, hht))

    return frameL, frameR, framel, framer
    
def binFondos(img,b):
    R = len(img)
    C = len(img[0])
    
    out = [[False for c in range(C)] for r in range(R)]
    for r in range(R):
        for c in range(C):
            if 3*img[r][c][0] >= sum(img[r][c]) + 2*b:
                out[r][c] = True
    
    maxim1, maxim2 = 0, 0
    grid1, grid2 = [], []
    for r in range(R):
        for c in range(C):
            if out[r][c]:
                out[r][c] = False
                count = 1 
                queue = [(r,c)]
                island = [(r,c)]
                while queue:
                    follow = []
                    for (i,j) in queue:
                        for (ii,jj) in [(-1,0),(1,0),(0,-1),(0,1)]:
                            m = i + ii
                            n = j + jj
                            if 0 <= m and m < R and 0 <= n and n < C:
                                if out[m][n] == True:
                                    count += 1
                                    out[m][n] = False
                                    follow.append((m,n))
                                    island.append((m,n))
                    queue = follow
                if maxim1 < count:
                    maxim1, maxim2 = count, maxim1
                    grid1, grid2 = island, grid1
                elif maxim2 < count:
                    maxim2 = count
                    grid2 = island

    fx1 = [0 for c in range(C)]
    fy1 = [0 for r in range(R)]
    for (r,c) in grid1:
        fx1[c] += 1
        fy1[r] += 1
    x1 = [(c+1)*fx1[c] for c in range(C)]
    y1 = [(r+1)*fy1[r] for r in range(R)]
    cx1 = int(round(sum(x1)/sum(fx1),0)) # Center X 1st biggest
    cy1 = int(round(sum(y1)/sum(fy1),0)) # Center Y 1st biggest

    fx2 = [0 for c in range(C)]
    fy2 = [0 for r in range(R)]
    for (r,c) in grid2:
        fx2[c] += 1
        fy2[r] += 1
    x2 = [(c+1)*fx2[c] for c in range(C)]
    y2 = [(r+1)*fy2[r] for r in range(R)]
    cx2 = int(round(sum(x2)/sum(fx2),0)) # Center X 2nd biggest
    cy2 = int(round(sum(y2)/sum(fy2),0)) # Center Y 2nd biggest

    if cx1 <= cx2: # Circulo izquierdo - Circulo derecho
        xl,yl,xr,yr = cx1,cy1,cx2,cy2
    else:
        xl,yl,xr,yr = cx2,cy2,cx1,cy1
        
    return xl, yl, xr, yr

def binFondo(img,b):
    R = len(img)
    C = len(img[0])
    
    out = [[False for c in range(C)] for r in range(R)]
    for r in range(R):
        for c in range(C):
            if 3*img[r][c][2] >= sum(img[r][c]) + 2*b:
                out[r][c] = True
    
    maxim= 0
    grid = []
    for r in range(R):
        for c in range(C):
            if out[r][c]:
                out[r][c] = False
                count = 1 
                queue = [(r,c)]
                island = [(r,c)]
                while queue:
                    follow = []
                    for (i,j) in queue:
                        for (ii,jj) in [(-1,0),(1,0),(0,-1),(0,1)]:
                            m = i + ii
                            n = j + jj
                            if 0 <= m and m < R and 0 <= n and n < C:
                                if out[m][n] == True:
                                    count += 1
                                    out[m][n] = False
                                    follow.append((m,n))
                                    island.append((m,n))
                    queue = follow
                if maxim < count:
                    maxim= count
                    grid = island

    fx = [0 for c in range(C)]
    fy = [0 for r in range(R)]
    for (r,c) in grid:
        fx[c] += 1
        fy[r] += 1
    x = [(c+1)*fx[c] for c in range(C)]
    y = [(r+1)*fy[r] for r in range(R)]
    cx = int(round(sum(x)/sum(fx),0)) # Center X 1st biggest
    cy = int(round(sum(y)/sum(fy),0)) # Center Y 1st biggest
        
    return cx, cy

def getFondos(img,b,xl,yl,xr,yr):
    R = len(img)
    C = len(img[0])

    fx1 = [0 for c in range(C)]
    fy1 = [0 for r in range(R)]
    for r in range(10*yl-80,10*yl+80):
        for c in range(10*xl-80,10*xl+80):
            if 3*img[r][c][2] >= sum(img[r][c]) + 2*b:
                fx1[c] += 1
                fy1[r] += 1    
    x1 = [(c+1)*fx1[c] for c in range(C)]
    y1 = [(r+1)*fy1[r] for r in range(R)]
    if sum(fx1) == 0:
        nxl = 10*xl
    else:
        nxl = int(round(sum(x1)/sum(fx1),0)) #Center left X
    if sum(fy1) == 0:
        nyl = 10*yl
    else:
        nyl = int(round(sum(y1)/sum(fy1),0)) #Center left Y

    fx2 = [0 for c in range(C)]
    fy2 = [0 for r in range(R)]
    for r in range(10*yr-80,10*yr+80):
        for c in range(10*xr-80,10*xr+80):
            if 3*img[r][c][2] >= sum(img[r][c]) + 2*b:
                fx2[c] += 1
                fy2[r] += 1
    x2 = [(c+1)*fx2[c] for c in range(C)]
    y2 = [(r+1)*fy2[r] for r in range(R)]
    if sum(fx2) == 0:
        nxr = 10*xr
    else:
        nxr = int(round(sum(x2)/sum(fx2),0)) #Center right X
    if sum(fy2) == 0:
        nyr = 10*yr
    else:
        nyr = int(round(sum(y2)/sum(fy2),0)) #Center right Y

    return nxl, nyl, nxr, nyr

def getFondo(img,b,x,y):
    R = len(img)
    C = len(img[0])

    fx = [0 for c in range(C)]
    fy = [0 for r in range(R)]
    for r in range(10*y-50,10*y+50):
        for c in range(10*x-50,10*x+50):
            if 3*img[r][c][0] >= sum(img[r][c]) + 2*b:
                fx[c] += 1
                fy[r] += 1    
    x = [(c+1)*fx[c] for c in range(C)]
    y = [(r+1)*fy[r] for r in range(R)]
    nx = int(round(sum(x)/sum(fx),0)) #Center left X
    ny = int(round(sum(y)/sum(fy),0)) #Center left Y
    
    return nx, ny

def Coordenadas(lx,ly,rx,ry):
    print('[',lx,ly,'] - [',rx,ry,']')
    #   [x1,y1]   Camaras   [x2,y2]
    #   Izquierda           Derecha
    
    b = 8.04            # Distancia focal
    f = 1455            # [ { 1280/(2 * tan(47.5/2)) } + { 960/(2 * tan(36.5/2)) } ] / 2
    tl = 0.4400105309   # tan(47.5/2)
    th = 0.3297505471   # tan(36.5/2)

    dx = lx - rx
    d = b*f/dx
    er = 1.976-0.11826*d+0.00165*d*d
    zf = d+er

    l = 2*tl*zf
    nx1 = l*(lx-640)/1280
    nx2 = l*(rx-640)/1280
    xf = (nx1+nx2)/2

    h = 2*th*zf
    ny1 = h*(480-ly)/960
    ny2 = h*(480-ry)/960
    yf = (ny1+ny2)/2
    
    return (xf,yf,zf)

def Angulo(xl,yl,zl,xr,yr,zr,theta):
    al = theta + np.arctan(yl/zl)
    ar = theta + np.arctan(yr/zr)
    
    cl = np.cos(al)
    cr = np.cos(ar)
    
    dl = ( (zl*zl + yl*yl)**0.5 ) * cl
    dr = ( (zr*zr + yr*yr)**0.5 ) * cr
    
    #print(dl, dr)
    delta = np.arctan((dr-dl)/(xr-xl))
    return np.rad2deg(delta)

RB = RaspBerry()
Nano = Arduino()
print('Iniciado')

while (True):
    lec = Nano.getLine()
    print('Lectura Control Remoto:',lec)
    if lec[:4] == 'EXIT':
        RB.putRasp()
        break
    elif lec[:2] in ['LU','LD','KL','KR']:
        RB.ChangePWM(lec[:2])
        continue
    elif lec[:2] == 'SW':
        RB.swapHDMI()
        continue
    elif ' ' in lec:
        beta, alfa = map(float,lec.split())
        print(beta,alfa)
        
        RB.putRasp()
        imgL, imgR, imgl, imgr = Camaras()
        lxl,lyl,lxr,lyr = binFondos(imgl,70)
        rxl,ryl,rxr,ryr = binFondos(imgr,70)
        nlxl,nlyl,nlxr,nlyr = getFondos(imgL,70,lxl,lyl,lxr,lyr)
        nrxl,nryl,nrxr,nryr = getFondos(imgR,70,rxl,ryl,rxr,ryr)
        xl,yl,zl = Coordenadas(nlxl,nlyl,nrxl,nryl) # Circulo izquierdo
        xr,yr,zr = Coordenadas(nlxr,nlyr,nrxr,nryr) # Circulo derecho
        delta = Angulo(xl,yl,zl,xr,yr,zr,0)
        RB.PWMKeyst(delta)
        print('Circulo Izquierdo: x=',round(xl,2),', y=',round(yl,2),'z=',round(zl,2))
        print('Circulo Derecho: x=',round(xr,2),', y=',round(yr,2),', z=',round(zr,2))
        print('Angulos:',alfa,delta)
        
        imgL, imgR, imgl, imgr = Camaras()
        lxc, lyc = binFondo(imgl,70)
        rxc, ryc = binFondo(imgr,70)
        nlxc, nlyc = getFondo(imgL,70,lxc,lyc)
        nrxc, nryc = getFondo(imgR,70,rxc,ryc)
        xc,yc,zc = Coordenadas(nlxc,nlyc,nrxc,nryc) # Circulo central
        RB.PWMLente(zc)
        print('Centro: x=',round(xc,2),', y=',round(yc,2),', z=',round(zc,2))        
        RB.putLap()

Nano.close()

#cams.Cierre()
#plt.subplot(121)
#plt.imshow(imgL)
#plt.subplot(122)
#plt.imshow(imgR)
#plt.show()
