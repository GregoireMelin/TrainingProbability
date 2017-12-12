import numpy as np
import pylab as plt
import time
import random


#retourne une distribution de probabilite uniforme p sur une grille de taille h w
def init_positions(h,w):
    positions = np.zeros((h,w))
    for i in range(h):
        for j in range(w):
            positions[i,j]=1.0/(h*w)
    return(positions)

#met a jour les probabilites de positions du robot (positions) suivant la decision de mouvement du robot (motion) et sa chance de reussir le deplacement p_move
def move(positions,motion):
    (h,w) = positions.shape
    new_positions = np.zeros((h,w))
    for i in range(h):
        for j in range(w):

            new_positions[i,j]=positions[i,j]

    return(new_positions)


#met a jour la mesure de probabilite de position du robot (positions) suivant la valeur mesuree par les capteurs (measurement) et la fiabilite du capteur (sensor_right)
def sense(positions,measurement):
    (h,w) = positions.shape
    new_positions = np.zeros((h,w))
    for i in range(h):
        for j in range(w):
            # TODO Question 3
            new_positions[i,j]=positions[i,j]
    return(new_positions)

# applique le filtre a histogramme pour chacun des mouvements du robot
def filter(positions, motions, measurements):
    #Met en route le mode interactif
    plt.ion()

    l=len(motions)
    for i in range(l):
        #
        positions = move(positions,motions[i])
        #
        positions = sense(positions,measurements[i])

        #affiche les probabilites de position du robot a la fin de la phase de mesure
        im = plt.imshow(positions,interpolation='nearest')
        plt.pause(1)

    return(positions)

#lance le filtre sur le jeu de donnee fourni, et affiche la
#position du robot estimee par le filtre
def handle_test_case(colors, motions, measurements):
    h = len(colors)
    w = len(colors[0])

    #Initialisation de la grille selon une loi uniforme
    positions = init_positions(h,w)
    print("DEBUT : Initialisation de la grille ")
    print (positions)

    #effectue la mise a jour des probabilites de position du robot
    positions = filter(positions, motions, measurements)
    print (positions)
    ind = np.argmax(positions)
    x = ind / w
    y = ind - w*x
    print "-- Maximal probability :", positions[x,y]
    print "-- Inferred position   :", x, ",", y

def main():

    print "Test case 1"
    #couleur de chaque position de la grille
    colors = [['red', 'red', 'red', 'red' , 'red','red', 'green', 'red', 'red' , 'red'],
            ['red', 'red', 'red', 'red', 'red','red', 'red', 'green', 'orange' , 'red'],
            ['red', 'red', 'red', 'green', 'red','red', 'red', 'red', 'red', 'red'],
            ['red', 'red', 'red', 'red', 'red','red', 'red', 'red', 'red', 'red']]
    #mouvements effectues
    motions = [[0,0],[0,1],[1,0],[1,0],[0,1],[-1,0],[1,0],[0,0],[0,1]]
    #valeurs observee au cours du mouvement
    measurements = ['green', 'red', 'green' ,'green', 'orange','green','green','orange','red']
    #probabilite que la mesure du senseur soit fiable
    sensor_right = 1.0
    #probabilite que le mouvement demande soit effectue
    p_move = 1.0

    position_initiale_R=[random.randint(0,len(colors)),random.randint(0,len(colors[0]))]
    print("Position initiale du robot : ",position_initiale_R)
    handle_test_case(colors, motions, measurements)

main()

################################################################
# TESTS
################################################################
#test pour la fonction init_positions
#print(init_positions(10,10))
