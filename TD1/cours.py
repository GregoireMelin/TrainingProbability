from math import *
import numpy as np


def motion_step(prior,moves):

    (h,w)=prior.shape
    post = np.zeros([h,w])
    for i in range(w):
        for j in range(h):
            post[i,j]=0
            for move in moves:
                (vec,prob)=move
                post[i,j]=prior[(i-vec[0]%h),(j-vec[1]%w)].prob
    return(np.matrix(post))

def sensor_step(prior,measure,world): #L'algo utilise une loi de Bernoulli

    (h,w)=prior.shape
    sum=0;
    for i in range(h):
        for j in range(w):
            if (world[i,j]==measure):
                post[i,j]=prior[i,j]+sensor
            else:
                post[i,j]=prior[i,j]+(1-sensor)
            sum+=post[i,j]
    for i in range(h):
        for j in range(w):
            post[i,j]=post[i,j]/sum
    return(np.matrix(post))
