from math import *
import random
import numpy as np
import pylab as plt



###############################################################
# Global values
###############################################################


g_measure_noise = 0.1  #noise used for generating robot's measure
g_turn_noise = 0.1  #noise used for generating robot's movement
g_distance_noise = 2.0  #noise used for generating robot's movement

measure_noise = 0.1 # noise in particle sensor measure
turn_noise = 0.2 # noise in the particle rotation
distance_noise = 3.0 # noise in the particle distance of deplacement 

particle_number = 500 # have a guess !
world_size = 50.0 # world is a square of world_size*world_size, and is cyclic
landmarks  = [[0.0, world_size-1], [0.0, 0.0], [world_size-1, 0.0], [world_size-1, world_size-1]] # position of 4 landmarks in (y, x) format

#landmark_number = 4
#landmarks = generate_landmark(landmark_number)

#one of the filter's particule
class particle:
     
    # initializes location/orientation 
    def __init__(self):
        self.x = 0.0 
        self.y = 0.0
        self.orientation = 0.0 
        self.measure_noise  = 0.01 #to avoid zero division at the beginning of the TD
        self.turn_noise = 0.0 
        self.distance_noise = 0.0 


    #sets a particle coordinates
    def set_pos(self, new_x, new_y, new_orientation):
        self.x = float(new_x)
        self.y = float(new_y)
        self.orientation = float(new_orientation%(2.0*pi))


    #sets noise parameters
    def set_noise(self, new_b_noise, new_t_noise, new_d_noise):
        self.measure_noise  = float(new_b_noise)
        self.turn_noise = float(new_t_noise)
        self.distance_noise = float(new_d_noise)


    # computes the probability of a measurement
    def measurement_prob(self, measurements):
        # computes error-free measurement from the particle position
        stored_noise = self.measure_noise
        self.measure_noise =0
        predicted_measurements = self.sense() 
        self.measure_noise = stored_noise
        
        # adds gaussian noise
        # calculates gaussian value of the measurement error for each landmark then multiply it
        error = 1.0
        for i in range(len(measurements)):
            error_bearing = abs(measurements[i] - predicted_measurements[i])
            error_bearing = (error_bearing + pi) % (2.0 * pi) - pi # truncate
            error *= (exp(- (error_bearing ** 2) / (self.measure_noise ** 2) / 2.0) /  
                      sqrt(2.0 * pi * (self.measure_noise ** 2)))

        return error
    

    #print particle
    def __repr__(self): 
        return '[x=%.6s y=%.6s orient=%.6s]' % (str(self.x), str(self.y), 
                                                str(self.orientation))

    
    #move
    def move(self, motion):
        new_orientation = 0.0
        new_x = 0.0
        new_y = 0.0

        #TODO question 2
        
        result = particle()
        result.measure_noise  = self.measure_noise
        result.turn_noise = self.turn_noise
        result.distance_noise = self.distance_noise
        result.set_pos(new_x,new_y,new_orientation)
        return result    
    

    # sense: 
    #for each landmark, measures the angle between the orientation of the particle and the landmark direction, with a gaussian noise (whose variance is self.measure_noise)
    def sense(self): 
        Z = []
        for l in landmarks:
            lx = l[1]
            ly = l[0]
            #TODO question 3 
        return Z 
    


# extract position from the particle set
#return the overage position and orientation of the particles
# beware of orientation which is cyclic
def get_position(p):
    x = 0.0
    y = 0.0
    orientation = 0.0
    #TODO question 5 
    
    return [x, y , orientation]

# generates the measurements vector
def generate_measures(motions):
    robot = particle()
    robot.set_noise(g_measure_noise, g_turn_noise, g_distance_noise)
    robot.set_pos(random.random() * world_size,random.random() * world_size,random.random()*2.0 * pi)
    Z = []
    T = len(motions)
    for t in range(T):
        robot = robot.move(motions[t])
        Z.append(robot.sense())
    return [robot, Z]

# returns world for display
def get_map(p):
    pict = np.zeros((world_size,world_size))
    N = len(p)
    for i in range(N):
        pict[floor(p[i].x),floor(p[i].y)] += 1.0/N
    return pict

# resamples particules
# resamples the vector p of N particles knowing their probabilities stored in w
def resample(p,w):
    N = len(p)
    new_p = []
    #TODO question 4

    index = int(random.random() * N)
    beta = 0.0
    mw = max(w)
    for i in range(N):
        beta += random.random() * 2.0 * mw
        while beta > w[index]:
            beta -= w[index]
            index = (index + 1) % N
        new_p.append(p[index])
    #
    return new_p


# generates particle_number particles with random positions
def generate_particles(particle_number):
    p=[]
    for i in range(particle_number):
        new_part =particle()
        #TODO question 1
        
        p.append(new_part)
    return p


# randomly generates landmark_number positions on the map 
def generate_landmark(landmark_number):
    l =[]
    for i in range(landmark_number):
        l.append([random.random() * world_size,random.random() * world_size])
    return l


def particle_filter(motions, measurements, particle_number):
    #randomly generates particle_number particles
    p = generate_particles(particle_number)

    plt.ion()
    l=len(motions)
    for t in range(l):
        # motion update (prediction)
        p2 = []
        for i in range(particle_number):
            p2.append(p[i].move(motions[t]))
        #p = p2

        # measurement update
        w = []
        for i in range(particle_number):
            w.append(p[i].measurement_prob(measurements[t]))

        #display particles before resampling
        im = plt.imshow(get_map(p))
        plt.pause(0.1)
        
        # resampling
        p =resample(p,w)
        
        result = get_position(p)
        
        # Display particles after resampling
        pict = get_map(p)
        im = plt.imshow(pict)
        plt.pause(0.1)
        #display robot position
        pict[floor(result[0]),floor(result[1])] =500
        im = plt.imshow(pict)
        plt.pause(0.1)

    plt.ioff()
    return result


################################################################
print "Test case 1 (varying rotation, constant motion, four landmarks)"

number_of_iterations = 10

#first element is rotation command, second is movement's distance
motions = [[random.random() * pi/4, 5] for row in range(number_of_iterations)]

x = generate_measures(motions) #creates a robot mvt and measures
final_robot = x[0]
measurements = x[1]              
estimated_position = particle_filter(motions, measurements,particle_number)
print '-- Real position   :', final_robot
print '-- Estimated position:', estimated_position


