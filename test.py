import math

# Définition des emplacements des points
points = {
    "B": {"x": 5, "y": 7},
    "C1": {"x": 1, "y": 5, "c": True},
    "C2": {"x": 2, "y": 1, "c": False},
    "C3": {"x": 6, "y": 2, "c": True},
    "C4": {"x": 10, "y": 6, "c": False},
    "C5": {"x": 7, "y": 3, "c": True},
    "C6": {"x": 4, "y": 9, "c": False},
    "C7": {"x": 3, "y": 4, "c": True},
    "C8": {"x": 8, "y": 1, "c": False},
    "C9": {"x": 6, "y": 6, "c": True},
    "C10": {"x": 9, "y": 5, "c": False},
    "C11": {"x": 2, "y": 8, "c": True},
    "C12": {"x": 7, "y": 8, "c": False},
    "C13": {"x": 3, "y": 7, "c": True},
    "C14": {"x": 1, "y": 3, "c": False},
    "C15": {"x": 4, "y": 2, "c": True},
    "C16": {"x": 8, "y": 4, "c": False},
    "C17": {"x": 10, "y": 2, "c": True},
    "C18": {"x": 9, "y": 8, "c": False},
    "C19": {"x": 5, "y": 5, "c": False},
    "C20": {"x": 2, "y": 2, "c": False},
}

# Fonction pour calculer la distance euclidienne entre deux points
def distance(p1, p2):
    return math.sqrt((p2["x"] - p1["x"])**2 + (p2["y"] - p1["y"])**2)

# Fonction pour calculer la distance totale maximale
def distance_totale_maximale(points):
    remaining_points = points.copy()
    current_point = remaining_points.pop("B")
    total_distance = 0
    path = ["B"]

    while remaining_points:
        # Trouver le point le plus éloigné
        next_point_name, next_point = max(
            remaining_points.items(), key=lambda item: distance(current_point, item[1])
        )
        print("next_point_name, next_point",next_point_name, next_point)
        # Ajouter la distance au total
        total_distance += distance(current_point, next_point)
        # Passer au point suivant
        current_point = next_point
        path.append(next_point_name)
        del remaining_points[next_point_name]

    return total_distance, path

# Calcul de la distance totale maximale
total_distance, path = distance_totale_maximale(points)
print(f"Distance totale maximale : {total_distance:.2f}")
print(f"Chemin suivi : {' -> '.join(path)}")
