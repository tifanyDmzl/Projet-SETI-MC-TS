from collections import namedtuple
from random import choice
from MCTSV2 import MCTS, Node 
import math
import matplotlib.pyplot as plt
from config import *



class DroneEnvironmentState(Node):
    def __init__(environment, tup, total_dist, terminal, visited_sensors, energy_remaining):
        environment.tup=tup
        environment.total_dist = total_dist
        environment.terminal = terminal
        environment.visited_sensors = visited_sensors
        environment.energy_remaining = energy_remaining

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
            consumed_energy = rapport_energy_dist*total_dist
            reward = 0
            for sensor in environment.visited_sensors :
                if (int(sensor[1:])%2): #impair
                    reward +=2/30
                else : #pair
                    reward +=1/30
            reward -= 1/110*consumed_energy/init_energy
            return reward
    
    def choose_next_sensor(environment, index):
        visited_sensors = environment.visited_sensors.copy() #Chaque état enfant a sa propre liste 

        total_dist = calculate_total_distance_drone(points,visited_sensors)

        # Calcul des distances supplémentaires
        last_sensor = visited_sensors[-1] if visited_sensors else "B"
        distance_to_next = calculate_distance(points[last_sensor], points[f"C{index+1}"])
        distance_back_to_base = calculate_distance(points[f"C{index+1}"], points["B"])
        current_to_base = calculate_distance(points[last_sensor], points["B"])

        added_distance = distance_to_next + distance_back_to_base - current_to_base

        # Mettre à jour l'énergie et la distance totale
        total_dist = total_dist + added_distance
        energy_remaining = init_energy - rapport_energy_dist * total_dist

        terminal = (len(visited_sensors)==len(points)-1) or energy_remaining < 0

        if not terminal : 
            tup = environment.tup[:index] + (True,) + environment.tup[index + 1 :]
            visited_sensors.append(f"C{index+1}")
            terminal = (len(visited_sensors)==len(points)-1) or energy_remaining < 0
        else : 
            energy_remaining = environment.energy_remaining #Décrémente pas l'énergie car si on est terminal on s'arrête là ! 
            tup = environment.tup

        return DroneEnvironmentState(tup = tup, total_dist=total_dist, terminal=terminal, visited_sensors=visited_sensors, energy_remaining = energy_remaining)

def calculate_distance(capteur1, capteur2):
    x1, y1 = capteur1['x'], capteur1['y']
    x2, y2 = capteur2['x'], capteur2['y']
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_total_distance_drone(points,visited_sensors): #Conditions des tailles de liste visited_sensors à optimiser ! 
    total_dist = 0
    if (len(visited_sensors)>=1):
        total_dist += calculate_distance(points["B"], points[visited_sensors[0]])
        for i in range (len(visited_sensors)-1):
            total_dist += calculate_distance(points[visited_sensors[i]], points[visited_sensors[i+1]]) 
        total_dist += calculate_distance(points[visited_sensors[len(visited_sensors)-1]], points["B"]) 
        #print ("Distance totale parcourue par le drône : ", total_dist)
    return total_dist

def do_drone_navigation(points, init_energy): 
    #energy_remaining = init_energy
    tree = MCTS()
    print("tree : ",tree)
    environment = new_drone_environment(points, init_energy)
    print (environment)
    while True :
        if environment.terminal : 
           break
        for _ in range(2000): #ATTENTION : Itère assez de chemins !! Sinon c'est juste une solution trop random !!!!!!
            tree.do_rollout(environment) #Replanning tous les 1 choix de capteur ? Sûrement très précis mais utilise beaucoup de ressources CPU pour recalculer à chaque coup ! 
            #Essayer replanning tous les 4 capteurs !? 
        environment = tree.choose(environment)
        print(f"Capteurs visités : {environment.visited_sensors}")
        print(f"Energy restante drône : {environment.energy_remaining}")
        

    print(f"Nombre de capteurs visités : {len(environment.visited_sensors)}")
    opti_total_dist = calculate_total_distance_drone(points, environment.visited_sensors)
    print(f"Distance totale parcourue par le drône (opti) : {opti_total_dist}")
    plot_capteurs_points(points, environment.visited_sensors)
    return environment.visited_sensors

#Create a tuple with 4 None for each sensor not yet visited
def new_drone_environment(points, init_energy):
    return DroneEnvironmentState(tup = (None,)*(len(points)-1), total_dist=0, terminal=False, visited_sensors=[], energy_remaining = init_energy)

def plot_capteurs_points(points, visited_sensors):
    # Tracer les points
    plt.figure(figsize=(10, 8))
    for name, coord in points.items():
        x, y = coord["x"], coord["y"]
        
        if name == "B" :
            color = "black" 
        elif name.startswith("C"):
            index = int(name[1:]) 
            if index % 2 : #index impair
                color = "red"
            else : 
                color = "green"
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

    do_drone_navigation(points, init_energy)
    

