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
                post[i,j]=prior[(i-vec[0]%h),(j-vec[1]%w)]
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
# ----------------------------------------------------------------------------- #
#Cours sur les filtres de Kalman
# ----------------------------------------------------------------------------- #
def sensor_step(mu_prior,cov_prior,measure):
    Y = measure - (H*mu_prior)
    S = H * cov_prior * H.T + Q
    K = P * H.T * np.linalg.inv(S)
    mu_prior = mu_prior * (K *y)
    cov_post = ( I - (K.H)*cov_prior)
    return [mu_post,cov_post]

def motion_step(mu_prior,cov_prior,move):
    mu_post=(F*mu_prior)+move
    cov_post=F*cov_prior*F.T+R
    return [mu_post,cov_post]
