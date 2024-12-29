"""from config import points

print(points["B"])"""


from config import *

capteurs_true = {key: value for key, value in points.items() if value.get("c") == True}
print(len(capteurs_true))

capteur1 = points["C1"]
print(capteur1["x"])
