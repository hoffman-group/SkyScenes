import os
import time

ROOT_DIR = "DATA"

"""
All the Seg/Instance/Depth Labels will be generated only in the ClearNoon variation. 
For testing, keep ClearNoon variation in the list to generate the labels.
"""

## All possible combinations:
# h_list = [15, 35, 60]
# p_list = [0, -45, -60, -90]
# weather_list = ["ClearNoon", "CloudyNoon", "MidRainyNoon", "ClearSunset", "ClearNight"]
# town_list = ["Town01", "Town02", "Town03", "Town04", "Town05", "Town06", "Town07", "Town10HD",]

## Supplementary combinations:
h_list = [15, 35,]
p_list = [0, -45,]
weather_list = ["ClearNoon", "ClearNight",]
town_list = ["Town01", "Town02",]

for h in h_list:
    for p in p_list:
        for weather in weather_list:
            for town in town_list:
                print("__"*20)
                print(f"Town: {town} | Weather: {weather}")
                print(f"Height: {h} | Pitch: {p}")
                time.sleep(5)
                os.system(f"sudo python3 loadingAttributesWeather.py --weather {weather} --height {h} --pitch {p} --town {town} --ROOT_DIR {ROOT_DIR}")
