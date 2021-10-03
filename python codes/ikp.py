# -*- coding: utf-8 -*-
"""
Created on Mon Aug 10 22:38:34 2020

@author: Sina Garazhian
"""
import numpy as np
import math

pi = math.pi

class DeltaRobot:
    def __init__(self, p1, p2, p3, p4, h, l, L, 
                 alpha0 = 0, alpha1 = pi/2, alpha2 = pi, alpha3 = 3*pi/2):
        '''inputs:
        p1, p2, p3 and p4 are position of joint so they are 3*1
        
        '''
        print("*****","P and D are transpose of P and D in the book.","********")
        self.h = h
        self.P = [p1,p2,p3,p4]
        self.ls = l
        self.Ls = L
        self.alphas = [alpha0 , alpha1, alpha2, alpha3]
    def GetMotorPositions(self, x, y, z, theta):
        self.D = np.array([[x, x, x - self.h * math.sin(theta), x - self.h * math.sin(theta) ],
                            [y, y, y + self.h * math.cos(theta), y + self.h * math.cos(theta) ],
                            [z, z, z, z] ]).transpose()
        p = self.P
        L = self.Ls
        l = self.ls
        I = [None] * 4
        J = [None] * 4
        K = [None] * 4
        delta = [None] * 4
        q = [None] * 4
        theta = [0] *4
        for i in range(4):
            I[i] = (2  * self.P[i] * self.D[i] )[2]
            J[i] = -2 * self.ls * (((self.P[i] * self.D[i] )[0]) * math.cos(self.alphas[i])
            + ((self.P[i] * self.D[i])[1]) * math.sin(self.alphas[i]))
            K[i] = (np.linalg.norm(self.P[i] * self.D[i])) ** 2 -self.Ls **2 + self.ls ** 2
            delta[i] = I[i] ** 2 - K[i] ** 2 + J[i] ** 2
            if delta[i] > 0:
                q[i] = 2 * math.atan((-I[i] - math.sqrt(delta[i])) / (K[i] - J[i])) 
            else:
                #print("WARNING : delta value for i = " + str(i) + 
                 #     "is less than zero so sqrt is not defined for it so we use -delta instead of delta.Something is wrong!!!!!")
                q[i] = 2 * math.atan((-I[i] - math.sqrt(-delta[i])) / (K[i] - J[i]))
        #print("q =",q)
        alfa0 = x - p[0][0]
        beta0 = p[0][2] - z
        sai0 = math.asin((x**2 +z**2 +(y-p[0][1])**2 +p[0][0]**2 +p[0][2]**2 +L**2 -l**2 -2*z*p[0][2] -2*x*p[0][0]) /(2*L*math.sqrt(alfa0**2 + beta0**2)))
        omega0 = math.asin(beta0/math.sqrt(alfa0**2 +beta0**2))
        theta[0] = (sai0 - omega0)*180/(pi)

        alfa1 = y - p[1][1]
        beta1 = p[1][2] - z
        sai1 = math.asin((y**2 +z**2 +(x-p[1][0])**2 +p[1][1]**2 +p[1][2]**2 +L**2 -l**2 -2*z*p[1][2] -2*y*p[1][1]) /(2*L*math.sqrt(alfa1**2 + beta1**2)))
        omega1 = math.asin(beta1/math.sqrt(alfa1**2 +beta1**2))
        theta[1] = (sai1 - omega1)*180/(pi)

        alfa2 = -x + p[2][0]
        beta2 = p[2][2] - z
        sai2 = math.asin((x**2 +z**2 +(y-p[2][1])**2 +p[2][0]**2 +p[2][2]**2 +L**2 -l**2 -2*z*p[2][2] -2*x*p[2][0]) /(2*L*math.sqrt(alfa2**2 + beta2**2)))
        omega2 = math.asin(beta2/math.sqrt(alfa2**2 +beta2**2))
        theta[2] = (sai2 - omega2)*180/(pi)

        alfa3 = -y + p[3][1]
        beta3 = p[3][2] - z
        sai3 = math.asin((y**2 +z**2 +(x-p[3][0])**2 +p[3][1]**2 +p[3][2]**2 +L**2 -l**2 -2*z*p[3][2] -2*y*p[3][2]) /(2*L*math.sqrt(alfa3**2 + beta3**2)))
        omega3 = math.asin(beta3/math.sqrt(alfa3**2 +beta3**2))
        theta[3] = (sai3 - omega3)*180/(pi)
        
        print(*theta)
        return q



#test
p1 =np.array([25,0,50])
p2 =np.array([0,25,50])
p3 =np.array([-25,0,50])
p4 =np.array([0,-25,50])
l = 60
L = 30
h = 0
myrobot = DeltaRobot(p1, p2, p3, p4,h,l,L)
myrobot.GetMotorPositions(1,1,1, 0)







