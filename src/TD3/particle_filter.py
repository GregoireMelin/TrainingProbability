from math import *
import random
import numpy as np
import pylab as plt


###############################################################
# Global values
###############################################################

g_measure_noise = 0.1  #noise used for generating robot s measure
g_turn_noise = 0.1  #noise used for generating robot s movement
g_distance_noise = 2.0  #noise used for generating robot s movement

measure_noise = 0.1 # noise in particle sensor measure
turn_noise = 0.2 # noise in the particle rotation
distance_noise = 3.0 # noise in the particle distance of deplacement

particle_number = 500 # have a guess !
world_size = 50.0 # world is a square of world_size*world_size, and is cyclic
landmarks  = [[0.0, world_size-1], [0.0, 0.0], [world_size-1, 0.0], [world_size-1, world_size-1]] # position of 4 landmarks in (y, x) format

#Le TP3 consiste a appliquer un filtre a particule
#dans le but d evaluer l orientation et la position
#d un robot au cours du temps. On formalise alors les coordonnees
#du robot par le vecteur [ x , y , theta ], x et y etant relatif
# a la position du robot dans la salle et theta son orientation.
#Le robot se deplace dans une salle dans laquelle il y a des
#marqueurs dont le robot peut determiner la direction.
#one of the filter s particule
def modulo(p1,p2):
    if p1<p2:
        return p1
    else:
        return p1-p2
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
        return  '[x=%.6s y=%.6s orient=%.6s]'  % (str(self.x), str(self.y), str(self.orientation))


    #move :
    #Lors d un deplacement, le robot tourne et se deplace a
    #l endroit auquel on lui demande avec une erreur definie
    #pour chaque particule, d une part par une loi normale centree
    #sur la variation d orientation souhaitee et de variance contenue
    #dans la variable turn_noise et d autre part par une loi normale
    #centree sur la distance que le robot doit parcourir et de variance distance_noise.

    def move(self, motion):
        #TODO question 2
        new_orientation = modulo(self.orientation + np.random.normal(motion[0],self.turn_noise ),2*pi)
        dist = np.random.normal(motion[1] , self.distance_noise)
        new_x = modulo(self.x + dist*cos(new_orientation),world_size)
        new_y = modulo(self.y + dist*sin(new_orientation),world_size)
        result = particle()
        result.measure_noise  = self.measure_noise
        result.turn_noise = self.turn_noise
        result.distance_noise = self.distance_noise
        result.set_pos(new_x,new_y,new_orientation)

        return result

    # sense:
    # La mesure nous donne l orientation des marqueurs mesurees par le robot.
    # On compare donc ces mesures au orientation reelle des marqueurs dans le but
    # de corriger les estimations du robot.
    # De plus, est connue la position de chaque particule ce qui permet d obtenir
    # l angle entre l orientation de la particule et le marqueur.
    #for each landmark, measures the angle between the orientation of the particle and the landmark direction, with a gaussian noise (whose variance is self.measure_noise)
    #TODO question 3
    def sense(self):
        Z = []
        for l in landmarks:

            landmark =np.array(l)
            position = np.array([self.x, self.y])
            # get direction vector : Permet de relier le robot au marqueur
            dir = landmark-position
            # normalisation du vecteur
            n = np.linalg.norm(dir)
            if n!=0:
                dir = dir/n
            # on calcule l angle entre l orientation du robot et celle du marqueur
            a = acos(np.dot(dir, [cos(self.orientation), sin(self.orientation)]))
            if self.measure_noise != 0:
                a = modulo(np.random.normal(a,self.measure_noise),2*pi)
            Z.append(a)
            # On peut donc a partir de cette angle permet d estimer la probabilite que le
            # robot se trouve a cette position (fonction : measurement_prob())
        return Z

# extract position from the particle set
#return the overage position and orientation of the particles
# beware of orientation which is cyclic
# TODO question 5
def get_position(p):
    #Calcul de la position
    X_mean = 0.0
    Y_mean = 0.0
    Theta_mean = 0.0
    for pos in p:
        Theta_mean+=pos.orientation
        X_mean += pos.x
        Y_mean += pos.y
    N = float(len(p))
    Y_mean /= N
    X_mean /= N
    Theta_mean /= N

    return [X_mean, Y_mean , Theta_mean]

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
    # Ici, on reechantillonne les particules. Les probabilites calculees dans la methode measurement_prob()
    # permettent de selectionner les meilleures particules. On a donc pour cela implementer un 'Roulette Wheel Sampling'
    # L'autre methode vue en cours serait :
    # -- SUS --
    #index = int(random.random() * N)
    #beta = 0.0
    #mw = max(w)
    #for i in range(N):
    #    beta += random.random() * 2.0 * mw
    #    while beta > w[index]:
    #        beta -= w[index]
    #        index = (index + 1) % N
    #    new_p.append(p[index])
    # -- RWS --
    s = sum(w)
    N = len(p)
    for i in range(len(w)):
        w[i]/=s
    base = np.random.random()/N
    c = w[0]
    index = 0
    for i in range(N):
        u = base + float(i)/N
        while u>c:
            index = (index+1)%len(p)
            c+=w[index]
        new_p.append(p[index])
    return new_p

# generates particle_number particles with random positions :
#Des particules sont generees aleatoirement dans la salle.
#entre 0 et la taille de la salle et l orientation entre 0 et 2*pi.
#Les coordonnees en position sont choisies aleatoirement
#Une deuxieme etape est de generer le bruit pour chaque particule :
#Ainsi on initialise les variables measure_noise, turn_noise et
#distance_noise qui correspondent respectivement aux variances
#des lois de Gauss caracterisant les erreurs de mesure de l orientation
#des marqueurs, de la variation d orientation du robot et de la
#variation de ses coordonnees lors d un deplacement.
def generate_particles(particle_number):
    p=[]
    for i in range(particle_number):
        new_part =particle()
        #TODO question 1
        new_part.set_pos(random.random()*world_size, random.random()*world_size,random.random()*pi*2)
        new_part.set_noise(measure_noise , turn_noise, distance_noise)
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
        p = p2

        # measurement update
        w = []
        for i in range(particle_number):
            w.append(p[i].measurement_prob(measurements[t]))

        #display particles before resampling
        im = plt.imshow(get_map(p))
        plt.pause(0.5)
        #plt.pause(0.1)

        # resampling
        p =resample(p,w)

        result = get_position(p)

        # Display particles after resampling
        pict = get_map(p)
        im = plt.imshow(pict)
        #plt.pause(0.1)
        plt.pause(1)
        #display robot position
        pict[floor(result[0]),floor(result[1])] =500
        im = plt.imshow(pict)
        plt.pause(0.5)

    plt.ioff()
    return result

################################################################
print "Test case 1 (varying rotation, constant motion, four landmarks)"
# TODO question 1 : voir generate_particles(particle_number)
# TODO question 2 : voir move(self, motion)
# TODO question 3 :voir sense(self)
# TODO question 4 :
# -------------------------------------------------
#Cas 1 : 1 iteration
#number_of_iterations = 1
#Cas 2 : 2 iteration
#number_of_iterations = 2

# On s interresse au moment apres la mesure du robot.
# La premiere iteration montre une
# variete de particules moins importantes que pour les autres iterations du programme.
# En effet, a la premiere iteration, les particules sont placees de maniere uniforme dans le monde.
# Les positions eloignees de la position reelle du robot sont alors remplacees par des positions plus proches
# du fait que les probabilites de ses positions eloignees d etre la position reelle du robot sont faibles (on a connaissance de la mesure)

# Les iterations suivantes montrent des particules qui se resserre autour du robot. La variete post-echantillonnage s explique alors
# car on a beaucoup de position proche de la position reelle du robot.
# -------------------------------------------------
# TODO question 5 : voir get_position(p)
# TODO question 6 :
# -------------------------------------------------
# - Cas 1 : 4 marqueurs - 3 iterations

number_of_iterations = 3
#landmark_number = 4
#landmarks = generate_landmark(landmark_number)

# - Cas 2 : 30 marqueurs - 3 iterations

#number_of_iterations = 3
#landmark_number = 30
#landmarks = generate_landmark(landmark_number)

#Cas 3 : 1 marqueur unique  - 3 iterations

#number_of_iterations = 3
#landmark_number = 30
#landmarks = generate_landmark(landmark_number)

# Le nombre de marqueur influe sur la precision du filtre.
# Plus le nombre de marqueur est important, plus la localisation estimee du robot est precise.
# Dans le cas d un marqueur unique, localiser le robot n est envisageable qu a proximite d une droite
# ayant pour extremite le marqueur ou plus precisement une demi-droite. Pour localiser le robot, soit trouver
# un point, il faut au moins 2 marqueurs.
# Placer un marqueur unique au centre du monde, localiser le robot est plus difficile puisque la variation d angle est plus rapide.
# -------------------------------------------------

#first element is rotation command, second is movement s distance
motions = [[random.random() * pi/4, 5] for row in range(number_of_iterations)]

x = generate_measures(motions) #creates a robot mvt and measures
final_robot = x[0]
measurements = x[1]
estimated_position = particle_filter(motions, measurements,particle_number)
print  '-- Real position   : ', final_robot
print  '-- Estimated position: ', estimated_position
