import numpy as np

m1 = 0.015898235185335192
m2 = -0.015130770105314221

speed = 400
tau = 20
s = tau * speed / 1000


def giveTillX1(start_speed, max_speed, target):
    start_time = start_speed / m1
    start_time = np.round(start_time / tau) * tau
    
