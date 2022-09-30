import numpy as np
from matplotlib import pyplot as plt

# Rotation Matrix: 2-dimensions
def rot2D(theta: float):
    return np.array([
        [ np.cos(theta), -np.sin(theta) ],
        [ np.sin(theta), np.cos(theta)  ]
    ])

# Translation: 2-dimensions
def transl2D(initialPoint, finalPoint):
    return np.sum([initialPoint, finalPoint], axis=0)


# plt.style.use("dark_background")

# Defining the world frame {O}
O = np.array([0, 0])
xo = np.array([1, 0])
yo = np.array([0, 1])

# Plotting {O}
plt.arrow(*O, *xo, width=0.005, head_width=0, length_includes_head=True, color="blue")
plt.arrow(*O, *yo, width=0.005, head_width=0, length_includes_head=True, color="blue")

# Apllying the rotation matrix: Frame {A}
thetaA = np.radians(30)
R = rot2D(thetaA)
xA = R.dot(xo)  # R times xo
yA = R.dot(yo)  # R times yo

# Plotting {A}
plt.arrow(*O, *xA, width=0.005, head_width=0, length_includes_head=True, color="green")
plt.arrow(*O, *yA, width=0.005, head_width=0, length_includes_head=True, color="green")

# Applying translation: Frame {B} (without rotation)
B = transl2D(O, [1, 1])
xB = xo
yB = yo

# Plotting {B}
plt.arrow(*B, *xB, width=0.005, head_width=0, length_includes_head=True, color="red")
plt.arrow(*B, *yB, width=0.005, head_width=0, length_includes_head=True, color="red")
# Plotting the translation vector
plt.arrow(*O, *B, width=0.001, head_width=0.05, length_includes_head=True, color="yellow", alpha=0.5)

# Applying Homogeneous Transformation Matrix: Frame {C}
thetaC = np.radians(45)
C = transl2D(O, [-1, 1.5])
RC = rot2D(thetaC)
xC = RC.dot(xo)
yC = RC.dot(yo)

# Plotting {C}
plt.arrow(*C, *xC, width=0.005, head_width=0, length_includes_head=True, color="orange")
plt.arrow(*C, *yC, width=0.005, head_width=0, length_includes_head=True, color="orange")
# Plotting the translation vector
plt.arrow(*O, *C, width=0.001, head_width=0.05, length_includes_head=True, color="yellow", alpha=0.5)


# plt.xlim(-5, 5)
# plt.ylim(-5, 5)
plt.gca().set_aspect('equal')
plt.grid(color="black", alpha=0.3, ls="--")
plt.show()