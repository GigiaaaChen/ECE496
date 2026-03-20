import numpy as np
import matplotlib.pyplot as plt
from mpl_toolkits.mplot3d.art3d import Poly3DCollection

# ---------- Create 3D plot ----------
fig = plt.figure(figsize=(10,8))
ax = fig.add_subplot(111, projection='3d')

# ---------- Sofa ----------
sofa = np.array([[0.2,-0.3,0],[0.8,-0.3,0],[0.8,0.3,0],[0.2,0.3,0],
                 [0.2,-0.3,0.4],[0.8,-0.3,0.4],[0.8,0.3,0.4],[0.2,0.3,0.4]])
faces = [[sofa[j] for j in [0,1,2,3]],
         [sofa[j] for j in [4,5,6,7]],
         [sofa[j] for j in [0,1,5,4]],
         [sofa[j] for j in [2,3,7,6]],
         [sofa[j] for j in [1,2,6,5]],
         [sofa[j] for j in [4,7,3,0]]]
ax.add_collection3d(Poly3DCollection(faces, alpha=0.3, facecolor='brown'))

# ---------- Head ----------
u, v = np.mgrid[0:2*np.pi:20j, 0:np.pi:10j]
x = 0.1*np.cos(u)*np.sin(v) + 0.5
y = 0.1*np.sin(u)*np.sin(v) + 0
z = 0.1*np.cos(v) + 0.9
ax.plot_surface(x, y, z, color='cyan', alpha=0.5)

# ---------- Stand (thin cylinder) ----------
cyl_h = 1.0
theta = np.linspace(0, 2*np.pi, 30)
z_cyl = np.linspace(0, cyl_h, 20)
theta, z_cyl = np.meshgrid(theta, z_cyl)
x_cyl = 0.03*np.cos(theta) + 1.0
y_cyl = 0.03*np.sin(theta) + 0.0
ax.plot_surface(x_cyl, y_cyl, z_cyl, color='gray', alpha=0.8)

# ---------- Arm Parameters ----------
base_x, base_y, base_z = 1.0, 0.0, cyl_h   # base on top of stand
L2, L3 = 0.20, 0.20   # only shoulder + elbow links

# Joint ranges
t1_vals = np.linspace(-np.pi, np.pi, 12)      # yaw (360°)
t2_vals = np.linspace(-np.pi/2, np.pi/2, 9)   # shoulder (±90°)
t3_vals = np.linspace(-1.11, np.pi, 9)        # elbow -63.7° ~ +180°

# ---------- Compute reachable points ----------
pts = []
for t1 in t1_vals:
    c1, s1 = np.cos(t1), np.sin(t1)
    for t2 in t2_vals:
        for t3 in t3_vals:
            # forward kinematics (planar 2-link chain rotated by yaw)
            ct2, st2 = np.cos(t2), np.sin(t2)
            ct23, st23 = np.cos(t2+t3), np.sin(t2+t3)

            Rxy = L2*ct2 + L3*ct23
            x = base_x + Rxy * c1
            y = base_y + Rxy * s1
            z = base_z + L2*st2 + L3*st23
            pts.append([x, y, z])

pts = np.array(pts)

# ---------- Plot reachable workspace ----------
ax.scatter(pts[:,0], pts[:,1], pts[:,2], s=5, c='blue', alpha=0.5)

# ---------- Reachable workspace as a sphere ----------
u, v = np.mgrid[0:2*np.pi:40j, 0:np.pi:20j]
r = L2 + L3   # max reach for 3-DOF arm
x = r*np.cos(u)*np.sin(v) + base_x
y = r*np.sin(u)*np.sin(v) + base_y
z = r*np.cos(v) + base_z
ax.plot_surface(x, y, z, color='blue', alpha=0.2)

# ---------- Labels ----------
ax.set_xlabel("X (m)")
ax.set_ylabel("Y (m)")
ax.set_zlabel("Z (m)")
ax.set_title("3-DOF Arm Workspace with Sofa + Head + Stand")
ax.set_box_aspect([1,1,0.8])

ax.text(0.5, 0.0, 1.05, "Head", color='cyan', fontsize=12)
ax.text(0.5, 0.0, 0.0, "Sofa", color='brown', fontsize=12)
ax.text(1.05, 0.0, 1.0, "Stand", color='gray', fontsize=12)
ax.text(base_x + r, base_y, base_z + 0.2, "Arm Reachable space", color='blue', fontsize=12)

plt.show()
