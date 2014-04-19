import math
import matplotlib.pyplot as plt

min_angle = 300/1024.
# 45 degrees is the maximum angle we can allow for the outward rear wheel
num_steps_in_45deg = 153
distance_between_wheels = 20.0
distance_from_steering_center_to_wheel = distance_between_wheels/2

def calc_distance_from_center(step):
    return distance_between_wheels/math.tan(math.radians(min_angle*step))

x = range(1, num_steps_in_45deg)

plt.plot(x, map(calc_distance_from_center, x), 'k')
plt.title("Distance to pivot by servo steps")
plt.xlabel("servo angle step")
plt.xlabel("distance to pivot")
plt.grid(True)
plt.show()