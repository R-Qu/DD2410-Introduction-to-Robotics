#! /usr/bin/env python3

"""
    # {Rui Qu }
    # {rqu@kth.se }
"""
import math as m
import numpy as n
from functools import reduce


def scara_IK(point):
    
    
    x=point[0]
    y=point[1]
    z=point[2]
    
    l0=0.07
    l1=0.3
    l2=0.35    

    a=((x-l0)**2+y**2-l1**2-l2**2)/(2*l1*l2)
    b=m.sqrt(1-a**2)
    
    q1=m.atan2(y,x-l0)-m.atan2(l2*b,l1+l2*a)
    q2=m.atan2(b,a) 
    q3=z
    
    q=[q1,q2,q3]
    
    
    
    return q

      

def kuka_IK(point, R, joint_positions):

    x = point[0]
    y = point[1]
    z = point[2]
    q = joint_positions #it must contain 7 elements    
    q=n.array(joint_positions)
    E=1e-2
    X=point
    
    for t in range(100):
        X_1,R_1=kuka_DH(q)
        e_X=X_1 - X
        e_R=kuka_AD(R,R_1)
        e=n.concatenate((e_X,e_R))
        
    
        J=kuka_jacobian(q)
        e_Q=n.dot(n.linalg.pinv(J),e)
        q=q-e_Q
        
        print (n.max(n.abs(e)))
        if n.max(n.abs(e)) < E:
            break
        
    return q

def kuka_matrix(alpha,d,r,theta):    
    
    M=n.array(\
        [[m.cos(theta),-m.sin(theta)*m.cos(alpha),m.sin(theta)*m.sin(alpha),r*m.cos(theta)],
         [m.sin(theta),m.cos(theta)*m.cos(alpha),-m.cos(theta)*m.sin(alpha),r*m.sin(alpha)],
         [0,m.sin(alpha),m.cos(alpha),d],
         [0,0,0,1]
        ])
    return M

def kuka_DH(joint_positions,short=False):

    
    A=0.331
    B=0.4
    C=0.39
    D=0.078
       
    q1,q2,q3,q4,q5,q6,q7=joint_positions
    
    table=[
        [m.pi/2,0,0,q1],
        [-m.pi/2,0,0,q2],
        [-m.pi/2,B,0,q3],
        [m.pi/2,0,0,q4],
        [m.pi/2,C,0,q5],
        [-m.pi/2,0,0,q6],
        [0,0,0,q7], 
    ]
    
    T=list(map(lambda x:kuka_matrix(*x),table))
    full_T=reduce(n.dot,T)
    R=full_T[:3, :3]
    Q=n.dot(full_T,n.array([0,0,D,1]))
    Q=Q[:3]
    
    if not short: Q[2]+= A
    
    return Q, R

def kuka_AD(R1,R2):
    
    R1=n.array(R1)
    R2=n.array(R2)
    
    a1,a2,a3=R1[:,0],R1[:,1],R1[:,2]
    b1,b2,b3=R2[:,0],R2[:,1],R2[:,2]
    
    c=.5*(n.cross(a1,b1)+n.cross(a2,b2)+n.cross(a3,b3))
    
    return c        
    
def kuka_jacobian(joint_positions):

    A=0.331
    B=0.4
    C=0.39
    D=0.078
    
    q1,q2,q3,q4,q5,q6,q7=joint_positions
    
    table=[
        [m.pi/2,0,0,q1],
        [-m.pi/2,0,0,q2],
        [-m.pi/2,B,0,q3],
        [m.pi/2,0,0,q4],
        [m.pi/2,C,0,q5],
        [-m.pi/2,0,0,q6],
        [0,0,0,q7], 
    ]        
    
    P,R=kuka_DH(joint_positions,True)
    T=list(map(lambda x:kuka_matrix(*x),table))
    
    transforms=[reduce(n.dot,T[:i],n.eye(4)) for i in range(len(T))]
    rotations=list(map(lambda x:x[:3,:3],transforms))
    translations=list(map(lambda x:x[:3,3],transforms))
    J_list=[]
        
    for i in range(0,7):
       
        r=rotations[i]
        tl=translations[i]
        z_i=n.dot(r,n.array([0.,0,1]))
        Joi=z_i
        Jpi=n.cross(z_i,P-tl)
        Ji=n.concatenate((Jpi,Joi))
        J_list.append(Ji)
    J=n.stack(J_list).T
        
    return J

