#!/usr/bin/python

import math
import numpy

########################################
# INPUT DATA
########################################
# Position of the camera
CameraPosition = numpy.array([0.95, -0.33, 1.77])

'''
Camera ray: (0.169907, 0.083991, -1.000000)
Camera ray: (-0.223489, 0.076939, -1.000000)
Camera ray: (0.162147, 0.376809, -1.000000)
Camera ray: (-0.229128, 0.371171, -1.000000)

Marker clicked at (1.325501, -0.115945, 0.222470)
Marker clicked at (0.678368, -0.149498, 0.234969)
Marker clicked at (1.290271, 0.368814, 0.269976)
Marker clicked at (0.635118, 0.306434, 0.237471)
'''

# Rays for screen corners
CameraRays = [
    numpy.array([0.169907, 0.083991, -1.000000]),
    numpy.array([-0.223489, 0.076939, -1.000000]),
    numpy.array([0.162147, 0.376809, -1.000000]),
    numpy.array([-0.229128, 0.371171, -1.000000])]

# Real positions of the previous points
RealPositions = [
    numpy.array([1.325501, -0.115945, 0.1400000]),
    numpy.array([0.678368, -0.149498, 0.1400000]),
    numpy.array([1.290271, 0.368814, 0.1400000]),
    numpy.array([0.635118, 0.306434, 0.1400000])]
########################################

if __name__ == "__main__":
    
    if(len(CameraRays) != len(RealPositions)):
        print("Input arrays are different in size")
        quit()

    # Calculate rays from camera position
    RealRays = [(RealPositions[p_index] - CameraPosition) for p_index in xrange(len(RealPositions))]
    
    # Normalize vectors
    CameraRays = [(CameraRays[p_index] / numpy.linalg.norm(CameraRays[p_index])) for p_index in xrange(len(CameraRays))]
    RealRays = [(RealRays[p_index] / numpy.linalg.norm(RealRays[p_index])) for p_index in xrange(len(RealRays))]

    # http://en.wikipedia.org/wiki/Kabsch_algorithm

    # Covariance matrix
    C = numpy.dot(numpy.transpose(CameraRays), RealRays)

    # Singular value decomposition
    V, S, Wt = numpy.linalg.svd(C)

    # Correction of rotation matrix
    d = math.copysign(1, numpy.linalg.det(V) * numpy.linalg.det(Wt))

    # Create rotation matrix
    Corr = numpy.diag([1, 1, d])
    U0 = numpy.dot(numpy.transpose(Wt), Corr)
    U = numpy.dot(U0, numpy.transpose(V))

    # Extract angles
    Ang1 = math.atan2(U[2][1], U[2][2])
    Ang2 = math.atan2(-U[2][0], math.sqrt(math.pow(U[2][1], 2) + math.pow(U[2][2], 2)))
    Ang3 = math.atan2(U[1][0], U[0][0])

    # Print!
    print(Ang1, Ang2, Ang3)
    print(Ang2, -Ang1)
