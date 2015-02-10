###################################################
######## Programmable filter script ########
###################################################

# Author: Matthieu Heitz

# Output DataSet Type = Same as Input

import vtk
import numpy as np
from numpy import linalg as LA

# print "\n\n"
# print "****************************************************\n" \
#       "Programmable Filter: Quaternion to Orientation Frame\n" \
#       "****************************************************\n"

# Get I/O
pdi = self.GetPolyDataInput()
pdo = self.GetPolyDataOutput()


pointData = pdi.GetPointData()
# print pointData

if pointData is None:
    print "ERROR: vtkPolyData has no dataArray"


orientationArray = pointData.GetArray("orientation")
# print orientationArray

if orientationArray is None:
    print "ERROR: vtkPolyData has no dataArray called \"orientation\""


numTuples = orientationArray.GetNumberOfTuples()

# Right = X
# Up = Y
# Backward = Z

rightArray = vtk.vtkFloatArray()
rightArray.SetNumberOfComponents(3)
rightArray.SetNumberOfTuples(numTuples)
rightArray.SetName('right')

upArray = vtk.vtkFloatArray()
upArray.SetNumberOfComponents(3)
upArray.SetNumberOfTuples(numTuples)
upArray.SetName('up')

backwardArray = vtk.vtkFloatArray()
backwardArray.SetNumberOfComponents(3)
backwardArray.SetNumberOfTuples(numTuples)
backwardArray.SetName('backward')

forwardArray = vtk.vtkFloatArray()
forwardArray.SetNumberOfComponents(3)
forwardArray.SetNumberOfTuples(numTuples)
forwardArray.SetName('forward')

for i in range(0, numTuples):
    orientation = orientationArray.GetTuple(i)
    x = orientation[0]
    y = orientation[1]
    z = orientation[2]
    w = orientation[3]

    a = 1.0 - 2.0 * (y*y + z*z)
    b = 2.0 * (x*y + w*z)
    c = 2.0 * (x*z - w*y)
    rightArray.SetTuple3(i, a, b, c)

    a = 2.0 * (x*y - w*z)
    b = 1.0 - 2.0 * (x*x + z*z)
    c = 2.0 * (y*z + w*x)
    upArray.SetTuple3(i, a, b, c)

    a = 2.0 * (x*z + w*y)
    b = 2.0 * (y*z - w*x)
    c = 1.0 - 2.0 *(x*x + y*y)
    backwardArray.SetTuple3(i, a, b, c)
    forwardArray.SetTuple3(i, -a, -b, -c)


pdo.GetPointData().AddArray(orientationArray)
pdo.GetPointData().AddArray(rightArray)
pdo.GetPointData().AddArray(upArray)
pdo.GetPointData().AddArray(backwardArray)
pdo.GetPointData().AddArray(forwardArray)


# print pdo
