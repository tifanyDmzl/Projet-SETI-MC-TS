from collections import namedtuple
from random import choice
from MCTSV2 import MCTS, Node 
import math
import matplotlib.pyplot as plt


class DroneEnvironmentState(Node):
    def __init__(environment, tup, total_dist, terminal, visited_sensors):
        environment.tup=tup
        environment.total_dist = total_dist
        environment.terminal = terminal
        environment.visited_sensors = visited_sensors


    def find_children(environment):
        # If we already visited all sensors return an empty list of children
        if environment.terminal : 
            return set()
        #Choose a next sensor which have not yet been visited (value = None) in the list of all possible children
        return {
            environment.choose_next_sensor(i) for i, value in enumerate(environment.tup) if value is None
        }
    
    def find_random_child(environment):
        if environment.terminal :
            return None
        #Utile dans MCTS heuristic (retourne aléatoirement le numéro d'un capteur qui n'a pas encore été visité !)
        empty_spots = [i for i, value in enumerate(environment.tup) if value is None]
        #print(empty_spots)
        return environment.choose_next_sensor(choice(empty_spots))
    
    def is_terminal(environment): 
        return environment.terminal
    
    def reward(environment):
        if not environment.terminal:
            raise RuntimeError(f"reward called on nonterminal environment {environment}")
        else : #Quand on est sur un noeud terminal ! 
            total_dist = calculate_total_distance_drone(points,environment.visited_sensors)
            environment.visited_sensors.clear()
            #reward = 1/(1+total_dist)
            reward = - total_dist #coef 100 pour creuser les écarts et trouver plus facilement le meilleur chemin ??
            return reward
    
    def choose_next_sensor(environment, index):
        #Add True in the tuple at the index of the capteur which have been visited ! 
        tup = environment.tup[:index] + (True,) + environment.tup[index + 1 :]
        #print("tup :",tup)
        total_dist = calculate_total_distance_drone(points,environment.visited_sensors)
        #visited_sensors = environment.get_visited_sensors()
        terminal = all(value is True for value in tup)
        #print("terminal =", terminal)
        environment.visited_sensors.append(f"C{index+1}")
        #if environment.terminal:
         #   environment.visited_sensors.clear()
        return DroneEnvironmentState(tup = tup, total_dist=total_dist, terminal=terminal, visited_sensors=environment.visited_sensors)
    
        #Fonction d'affichage
    def get_visited_sensors(environment, visited):
        # Return a list of sensor names in the order they were visited
        #sensor_names = ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8", "C9", "C10", "C11", "C12", "C13", "C14", "C15", "C16", "C17", "C18", "C19", "C20"]
        sensor_names = ["C1", "C2", "C3", "C4", "C5", "C6", "C7", "C8"]
        for i, value in enumerate(environment.tup):
            if value is True :
                flag = True
                for j in range (len(visited)):
                    if sensor_names[i] == visited[j] : 
                        flag = False 
                if flag :
                    visited.append(sensor_names[i])
        return visited


def calculate_distance(capteur1, capteur2):
    x1, y1 = capteur1['x'], capteur1['y']
    x2, y2 = capteur2['x'], capteur2['y']
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_total_distance_drone(points,visited_sensors): #Conditions des tailles de liste visited_sensors à optimiser ! 
    total_dist = 0
    #print("dist début calcul : 0?", environment.total_dist)
   
    if (len(visited_sensors)>=1):
        total_dist += calculate_distance(points["B"], points[visited_sensors[0]])
        for i in range (len(visited_sensors)-1):
            total_dist += calculate_distance(points[visited_sensors[i]], points[visited_sensors[i+1]]) 
        total_dist += calculate_distance(points[visited_sensors[len(visited_sensors)-1]], points["B"]) 
        #print ("Distance totale parcourue par le drône : ", total_dist)
    return total_dist

"""
def calculate_total_distance_drone(environment): #Conditions des tailles de liste visited_sensors à optimiser ! 
    environment.total_dist = 0
    #print("dist début calcul : 0?", environment.total_dist)
    if (len(environment.visited_sensors)>=1):
        environment.total_dist += environment.calculate_distance(environment.points["B"], environment.points[environment.visited_sensors[0]])
        for i in range (len(environment.visited_sensors)-1):
            environment.total_dist += environment.calculate_distance(environment.points[environment.visited_sensors[i]], environment.points[environment.visited_sensors[i+1]]) 
        environment.total_dist += environment.calculate_distance(environment.points[environment.visited_sensors[len(environment.visited_sensors)-1]], environment.points["B"]) 
        print ("Distance totale parcourue par le drône : ", environment.total_dist)
    return environment.total_dist
"""

    

    



def do_drone_navigation(points): 
    visited_sensors =[]
    tree = MCTS()
    print("tree : ",tree)
    environment = new_drone_environment()
    print (environment)
    while True :
        if environment.terminal : 
           break
        for _ in range(2000): #ATTENTION : Simuler assez de chemins !! Sinon c'est juste une solution trop random !!!!!!
            tree.do_rollout(environment) #Replanning tous les 1 choix de capteur ? Sûrement très précis mais utilise beaucoup de ressources CPU pour recalculer à chaque coup ! 
            #Essayer replanning tous les 4 capteurs !? 
        environment = tree.choose(environment)
        visited_sensors = environment.get_visited_sensors(visited_sensors)
        #print("tree : ",tree.Q)
        print(f"Capteurs visités : {visited_sensors}")
    # Affichage de l'ordre des capteurs visités
    #visited_sensors = environment.get_visited_sensors()

    #visited_sensors = environment.visited_sensors
    #print(f"Capteurs visités : {visited_sensors}")
    opti_total_dist = calculate_total_distance_drone(points, visited_sensors)
    print(f"Distance totale parcourue par le drône (opti) : {opti_total_dist}")
    plot_capteurs_points(points, visited_sensors)
    return visited_sensors

#Create a tuple with 4 None for each sensor not yet visited
def new_drone_environment():
    return DroneEnvironmentState(tup = (None,)*8, total_dist=0, terminal=False, visited_sensors=[])

def plot_capteurs_points(points, visited_sensors):
    # Tracer les points
    plt.figure(figsize=(10, 8))
    for name, coord in points.items():
        x, y = coord["x"], coord["y"]
        color = "red" if name == "B" else "blue"
        plt.scatter(x, y, color=color, label=name if name == "B" else "", s=100)
        offset_x = 0.3  # Décalage pour l'étiquette
        plt.text(x + offset_x, y, name, fontsize=10, ha='left', va='center')

        # Tracer le chemin emprunté par le drone
    if visited_sensors:
        # Commencer par la base
        x_coords =[]
        y_coords =[]
        x_coords = [points["B"]["x"]]
        y_coords = [points["B"]["y"]]
        # Ajouter les capteurs visités dans l'ordre
        for sensor in visited_sensors:
            x_coords.append(points[sensor]["x"])
            y_coords.append(points[sensor]["y"])
        # Revenir à la base
        x_coords.append(points["B"]["x"])
        y_coords.append(points["B"]["y"])
        # Tracer les lignes
        #print("x_coords :",x_coords)
        #print("y_coords :",y_coords)
        plt.plot(x_coords, y_coords, color="green", linestyle="--", marker="o", label="Trajet du drone")


    # Configuration de l'affichage
    plt.title("Position des capteurs et de la base")
    plt.xlabel("X")
    plt.ylabel("Y")
    plt.grid(True)
    plt.axhline(0, color="black",linewidth=0.5)
    plt.axvline(0, color="black",linewidth=0.5)
    plt.legend()
    plt.show()





if __name__ == "__main__":

    points = {
        "B": {"x": 5, "y": 7},
        "C1": {"x": 1, "y": 5},
        "C2": {"x": 2, "y": 1},
        "C3": {"x": 6, "y": 2},
        "C4": {"x": 10, "y": 6},
        "C5": {"x": 7, "y": 3},
        "C6": {"x": 4, "y": 9},
        "C7": {"x": 3, "y": 4},
        "C8": {"x": 8, "y": 1},
        }


    do_drone_navigation(points)
    


