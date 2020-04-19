#!/usr/bin/env python
import matplotlib.pyplot as plt
import numpy as np
import scipy.special


def plot_path(path, control_points):
	fig, ax = plt.subplots()
	ax.plot(path.T[0], path.T[1], label="Bezier Path")
	ax.plot(control_points.T[0], control_points.T[1],'--o', label="Control Points")
	ax.legend()
	ax.axis("equal")
	ax.grid(True)
	plt.show()

def curvature(dx, dy, ddx, ddy):
	return (dx * ddy - dy * ddx) / (dx ** 2 + dy ** 2) ** (3 / 2)
	
def bezier_path(sx, sy, syaw, ex, ey, eyaw, offset, n_points):

	dist = np.hypot(sx - ex, sy - ey) / offset
	control_points = np.array(
		[[sx, sy],
		 [sx + dist * np.cos(syaw), sy + dist * np.sin(syaw)],
		 [ex - dist * np.cos(eyaw), ey - dist * np.sin(eyaw)],
		 [ex, ey]])

	path = calc_bezier_path(control_points, n_points)

	return path, control_points

def calc_bezier_path(control_points, n_points=100):

	traj = []
	for t in np.linspace(0, 1, n_points):
		traj.append(bezier_point(t, control_points))

	return np.array(traj)

def bezier_point(t, control_points):

	n = len(control_points) - 1
	vals = [(scipy.special.comb(n,i)*t**i*(1-t)**(n-i)) * control_points[i] for i in range(n + 1)]
	return np.sum(vals, axis=0)

def bezier_derivatives_control_points(control_points, n_derivatives):

	w = {0: control_points}
	for i in range(n_derivatives):
		n = len(w[i])
		w[i + 1] = np.array([(n - 1) * (w[i][j + 1] - w[i][j])
							 for j in range(n - 1)])
	return w