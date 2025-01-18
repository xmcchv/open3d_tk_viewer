import numpy as np

angl = 344.38
angv = 10.99
v = 8155

alpha = float(angl)/180*np.pi
omega = float(angv)/180*np.pi
r = float(v) / 1000
# print(alpha,omega,r)
# 计算点云坐标
z = r * np.sin(omega) 
x = r * np.sin(alpha) * np.cos(omega) + 0.0635 * np.cos(alpha)
y = r * np.cos(alpha) * np.cos(omega) + 0.0635 * np.sin(alpha)


print(x,y,z)