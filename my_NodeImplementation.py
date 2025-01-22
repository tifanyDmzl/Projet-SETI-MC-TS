
from random import choice
from my_MCTS import MCTS, Node 
import math
import matplotlib.pyplot as plt
from config import *
#from sim_control import *
from simcontrolV5 import *
import time


class Node(Node):
    #Attributes at NODE level
    next_node_id = 0

    def __init__(self, sensor, parent, terminal):
       
        self.id = Node.next_node_id
        Node.next_node_id +=1
        self.sensor = sensor 
        self.parent = parent
        self.terminal = terminal
        self.accumulated_cost_mode_LO = 0 
        self.accumulated_cost_mode_HI = 0 
        self.compute_accumulated_cost()
        if (self.id <3):
            print(f"Node Created: ID={self.id}, Sensor={self.sensor}, Parent={self.parent.sensor if self.parent else None}, Terminal={self.terminal}")

    def compute_accumulated_cost(self) :
        if (self.parent == None) :
            self.accumulated_cost_mode_LO = 0 
            self.accumulated_cost_mode_HI = 0 
        else :
            dist_from_parent_to_current_node = calculate_distance(points[self.parent.sensor],points[self.sensor])
            self.accumulated_cost_mode_LO = self.parent.accumulated_cost_mode_LO + coef_energy_no_wind*dist_from_parent_to_current_node
            
            if (points[self.sensor]["c"] == True):
                node = self.parent
                worst_accumulated_cost = node.accumulated_cost_mode_HI
                while (node.parent != None and points[node.sensor]["c"]==False):
                    node = node.parent
                    if (node.accumulated_cost_mode_HI > worst_accumulated_cost):
                        worst_accumulated_cost = node.accumulated_cost_mode_HI

                self.accumulated_cost_mode_HI = worst_accumulated_cost + coef_energy_wind*dist_from_parent_to_current_node
            else : 
                self.accumulated_cost_mode_HI = self.parent.accumulated_cost_mode_LO + coef_energy_wind*dist_from_parent_to_current_node
        """if (self.id <3):
            print("dist_from_parent_to_current_node",dist_from_parent_to_current_node)
            print("accumulated_cost_mode_HI",self.id, " : ",self.sensor, " : ",self.accumulated_cost_mode_HI)
            print("accumulated_cost_mode_LO",self.id, " : ",self.sensor, " : ",self.accumulated_cost_mode_LO)
        """

    def find_children(self):
        # If we already visited all sensors return an empty list of children
        if self.terminal : 
            return set()
        #Choose a next sensor which have not yet been visited (value = None) in the list of all possible children
        visited_sensors = set()
        node = self
        while (node.parent != None):
            visited_sensors.add(node.sensor)
            node = node.parent 

        unvisited_sensors = [
            sensor for sensor in points.keys() if sensor not in visited_sensors and sensor != "B"
        ]

        #print("unvisited_sensors",unvisited_sensors)
        if (len(unvisited_sensors) != 0):
            return {
                    self.choose_next_sensor(sensor) for sensor in unvisited_sensors
            }
        else : #Gestion du cas où on a visité tous les capteurs 
            return {
                    self.choose_next_sensor("B") 
            }

    
    """def find_random_child(self):
        if self.terminal :
            return None
        #Utile dans MCTS heuristic (retourne aléatoirement le numéro d'un capteur qui n'a pas encore été visité !)
        empty_spots = [i for i, value in enumerate(self.tup) if value is None] 
        #print(empty_spots)
        return self.choose_next_sensor(choice(empty_spots))"""
    
    def find_closer_child(self):
        if self.terminal :
            return None
        visited_sensors = set()
        node = self
        while (node.parent != None):
            visited_sensors.add(node.sensor)
            node = node.parent

        unvisited_sensors = [
            sensor for sensor in points.keys() if sensor not in visited_sensors and sensor != "B"
        ]

        min_distance = float("inf")
        closest_sensor = "B"
        for un_sensor in unvisited_sensors:
            distance = calculate_distance(points[self.sensor],points[un_sensor])
            if distance < min_distance :
                min_distance = distance 
                closest_sensor = un_sensor
        return self.choose_next_sensor(closest_sensor)
        


    def is_terminal(self): 
        return self.terminal

    def reward(self):
        """if not self.terminal:
            raise RuntimeError(f"reward called on nonterminal self {self}")
        else : #Quand on est sur un noeud terminal ! 
        """   
        total_dist = calculate_accumulated_distance_drone(self)
        max_dist = calculate_distance_totale_max()
        
        reward = 0
        total_nber_sensors_ET_poids = (len(critical_sensors)*reward_critical_sensors + len(non_critical_sensors)*reward_non_critical_sensors)
        #for sensor in self.visited_sensors :
        node = self
        while (node.parent != None):
            sensor = node.sensor
            if sensor in critical_sensors: 
                reward +=reward_critical_sensors/total_nber_sensors_ET_poids
                reward -= (reward_critical_sensors/total_nber_sensors_ET_poids)*(total_dist/max_dist) #change
                #reward += (reward_critical_sensors/total_nber_sensors_ET_poids)*(1-(total_dist/max_dist))
            else : 
                reward +=reward_non_critical_sensors/total_nber_sensors_ET_poids
                reward -= (reward_non_critical_sensors/total_nber_sensors_ET_poids)*(total_dist/max_dist) #change
                #reward += (reward_non_critical_sensors/total_nber_sensors_ET_poids)*(1-(total_dist/max_dist))
            node = node.parent 
        #reward -= (1/total_nber_sensors_ET_poids)*(total_dist/max_dist)
        return reward
    
    def choose_next_sensor(self, sensor):
        #print("sensor",sensor)

  
        #Considère qu'on est arrivé en mode LO => va à la base en mode LO 
        #Si on est arrivé en mode HI => va à la base en mode HI 
        #Ou prendre en compte le mode actuel du système ????
        
        #Gestion terminal => A MODIF Là je check une itération trop tard = Pas assez d'énergy pour retourner à la base 

        #print("terminal",terminal)
        #print(self.sensor) #On est bloqué dans B ! On évolu pas dans l'arbre ! 

        #print("test",self.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base))
        #print("test",self.accumulated_cost_mode_LO + coef_energy_no_wind*(dist_next_to_base+dist_next_to_base))
        #A MODIF



        #Le prochain capteur à visiter : sensor => Check s'il reste assez d'énergie pour ajouter la visite de ce capteur ou non !? 
        dist_next_to_base = calculate_distance(points[sensor],points["B"])
        dist_to_next = calculate_distance(points[self.sensor],points[sensor])
        
        #Gestion du cas où on a visité tous les capteurs 
        if (sensor == "B"): 
            return Node(sensor = sensor, parent = self, terminal = True)
        #Gestion du cas où on est à cours d'énergie (noeud terminal)
        elif (self.accumulated_cost_mode_HI + coef_energy_wind*(dist_to_next+dist_next_to_base) > init_energy or self.accumulated_cost_mode_LO + coef_energy_no_wind*(dist_next_to_base+dist_next_to_base) > init_energy):      
            return Node(sensor = "B", parent = self, terminal = True)
        else :
            return Node(sensor = sensor, parent = self, terminal = False)




        

    

def calculate_distance(capteur1, capteur2):
    x1, y1 = capteur1['x'], capteur1['y']
    x2, y2 = capteur2['x'], capteur2['y']
    return math.sqrt((x2 - x1)**2 + (y2 - y1)**2)

def calculate_accumulated_distance_drone(last_node): #Conditions des tailles de liste visited_sensors à optimiser ! 
    accumulated_dist = 0 
    node = last_node
    while (node.parent != None):
        dist_to_add = calculate_distance(points[node.sensor], points[node.parent.sensor])
        accumulated_dist += dist_to_add
        node = node.parent 
    return accumulated_dist

"""
def calculate_total_distance_drone(points,visited_sensors): #Conditions des tailles de liste visited_sensors à optimiser ! 
    total_dist = 0
    if (len(visited_sensors)>=1):
        total_dist += calculate_distance(points["B"], points[visited_sensors[0]])
        for i in range (len(visited_sensors)-1):
            total_dist += calculate_distance(points[visited_sensors[i]], points[visited_sensors[i+1]]) 
        total_dist += calculate_distance(points[visited_sensors[len(visited_sensors)-1]], points["B"]) 
        #print ("Distance totale parcourue par le drône : ", total_dist)
    return total_dist
"""
def calculate_distance_totale_max():
    remaining = points.copy()
    current = remaining.pop("B")
    total = 0

    while remaining:
        next_name, next_point = max(remaining.items(), key=lambda p: calculate_distance(current, p[1])) #A revoir 
        total += calculate_distance(current, next_point)
        current = remaining.pop(next_name)
    
    return total


def do_drone_navigation(): 
    #energy_remaining = init_energy
    start_time = time.time()  # Début du chronométrage
    tree = MCTS()
    node = new_Node()
    visited_sensors = []
    real_visited_sensors =[]
    deleted_sensors = []
    sensors_since_last_replanning = 0
    is_HI_Mode = False
    visited_sensors.append(node.sensor)
    real_visited_sensors.append(node.sensor)
    while True :
        if node.terminal : 
           break
        """for _ in range(1000): #ATTENTION : Itère assez de chemins !! Sinon c'est juste une solution trop random !!!!!!
            tree.do_rollout(environment) #Replanning tous les 1 choix de capteur ? Sûrement très précis mais utilise beaucoup de ressources CPU pour recalculer à chaque coup ! 
            #Essayer replanning tous les 4 capteurs !? """
        
        # Replanning uniquement tous les 4 capteurs
        if sensors_since_last_replanning >= 3 or node.parent == None : #Réflexion tous les 4 capteurs choisi
            for _ in range(nber_of_rollout_iterations):  
                tree.do_rollout(node) #Algo = Réflexion = Mise à jour des poids de l'arbre 
            sensors_since_last_replanning = 0
            print("Do algo")


        
        #print("last_node",node.id, node.sensor, node.terminal, node.parent)
        node = tree.choose(node) #Action = choix capteur suivant => Change de noeud courrant pour aller au suivant 
        visited_sensors.append(node.sensor)
        
        sensors_since_last_replanning += 1
        print("is_HI_Mode",is_HI_Mode)
        


       
        
        if (is_HI_Mode):#Mode HI =>va que vers les capteurs HI du segment 
            if(points[node.sensor]["c"] == True):
                simulation.move_drone_to_sensor(node.sensor)
                real_visited_sensors.append(node.sensor)
            else :
                deleted_sensors.append(node.sensor)
                
        else : #Mode LO =>va que vers tous les capteurs du segment 
            simulation.move_drone_to_sensor(node.sensor)
            real_visited_sensors.append(node.sensor)



        
        real_energy_consumed = simulation.get_consumed_energy()
        dist = calculate_accumulated_distance_drone(node)
        print("energy_consumed_LO",node.accumulated_cost_mode_LO)
        print("energy_consumed_HI",node.accumulated_cost_mode_HI)
        print("accumulated_dist",dist)
        print("real_energy_consumed",real_energy_consumed)
       
        if (real_energy_consumed > node.accumulated_cost_mode_LO):
            is_HI_Mode = True
        else :
            is_HI_Mode = False 
        #print("is_HI_Mode",is_HI_Mode)
        
        energy_remaining = init_energy - real_energy_consumed
        


        
        #simulation.move_drone_to_sensor(node.sensor)

    print("node_teminal_id",node.id)

    print(f"Capteurs visités : {visited_sensors}")
    print(f"Capteurs visités (réel): {real_visited_sensors}")
    print(f"Deleted sensor :  {deleted_sensors}")
    print(f"Energy restante drône : {energy_remaining}")


    end_time = time.time()  # Fin du chronométrage
    execution_time = end_time - start_time  # Calcul du temps écoulé
    print(f"Temps d'exécution de do_drone_navigation: {execution_time:.6f} secondes")

    print(f"Nombre de capteurs visités : {len(visited_sensors)}")
    opti_total_dist = calculate_accumulated_distance_drone(node)
    print(f"Distance totale parcourue par le drône (opti) : {opti_total_dist}")
    print(f"Final Reward : ",node.reward())
    plot_capteurs_points(points, visited_sensors, real_visited_sensors, is_HI_Mode)
    #return visited_sensors


def new_Node():
    return Node(sensor = "B", parent = None, terminal = False)

def plot_capteurs_points(points, visited_sensors, real_visited_sensors, is_HI_Mode):
    # Tracer les points
    plt.figure(figsize=(10, 8))
    for name, coord in points.items():
        x, y = coord["x"], coord["y"]
        if name == "B" :
            color = "black" 
        elif name in critical_sensors:
            color = "red"
        elif name in non_critical_sensors:
            color = "green"
        plt.scatter(x, y, color=color, label=name if name == "B" else "", s=100)
        offset_x = 0.3  # Décalage pour l'étiquette
        plt.text(x + offset_x, y, name, fontsize=10, ha='left', va='center')

        # Tracer le chemin emprunté par le drone
    """if visited_sensors:
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

        plt.plot(x_coords, y_coords, color="black", linestyle="--", marker="o", label="Trajet du drone : sequence init")
    """
    if real_visited_sensors:
        # Commencer par la base
        x_coords_real =[]
        y_coords_real =[]
        """
        x_coords_real = [points["B"]["x"]]
        y_coords_real = [points["B"]["y"]]
        """
        # Ajouter les capteurs visités dans l'ordre
        for sensor in real_visited_sensors:
            x_coords_real.append(points[sensor]["x"])
            y_coords_real.append(points[sensor]["y"])
        """
        # Revenir à la base
        x_coords_real.append(points["B"]["x"])
        y_coords_real.append(points["B"]["y"])
        """
        # Tracer les lignes
        #print("x_coords :",x_coords)
        #print("y_coords :",y_coords)
        plt.plot(x_coords_real, y_coords_real, color="orange", linestyle="--", marker="o", label="Trajet du drone : real (some LO sensors deleted)")
        

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
    #create_coords(points)
    simulation = DroneSimulation(points, wind_moves)
    do_drone_navigation()
    
