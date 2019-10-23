import julia

j = julia.Julia()
j.include("setup.jl")
j.test("asd")


import numpy as np
w = np.array([1,2,3])
w_dot = np.array([1,2,3]) + 3
s = np.array([1,2,3]) + 6
r = np.array([[1,2,3], [4,5,6], [7,8,9]] )

y = np.empty([3*3 + 3])

for i in range(3):

    y[3*i:3*(i+1)] = np.cross(w, np.cross(w, r[:,i])) + np.cross(w_dot, r[:,i]) + s

y[9:12] = w

print j.dyncamics_wrapper(y, 1, r)
