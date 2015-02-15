###################################################
######## Programmable filter script ########
###################################################

# Author: Matthieu Heitz

# Output DataSet Type = Same as Input


############# Properties for auto-generated XML #############

Name = 'ApplyPoseTransformToPointCloud'
Label = 'Apply Pose To PointCloud'
Help = 'This applies the device pose point cloud at each timestep'


# Still don't know if these lines are actually necessary
NumberOfInputs = 2
InputDataType1 = 'vtkPolyData'
InputDataType2 = 'vtkPolyData'
# OutputDataType = 'vtkPolyData' # omit this line to use 'same as input'

Properties = {}

def RequestData():

    ############# Initialize the filter #############

    import vtk
    import numpy as np
    from numpy import linalg as LA

    # print "\n\n"
    # print "********************************************************\n" \
    #       "Programmable Filter: Apply Pose Transform to Point Cloud\n" \
    #       "********************************************************\n"


    ############# Get I/O #############

    # Get the two inputs, and the output
    polyDataA = self.GetInputDataObject(0, 0)
    polyDataB = self.GetInputDataObject(0, 1)
    pdo = self.GetPolyDataOutput()

    # If only one input is given, raise an exception
    if polyDataA is None or polyDataB is None:
        raise Exception("\nThis filter takes 2 inputs:\n"
                        "Point Cloud Data files: pc_HHMMSSDD_NNN.vtk\n"
                        "Pose Data file: pc_HHMMSSDD_poses.vtk\n"
                        "Note that ParaView groups all the Point Cloud Data files in one\n")

    # Initialize vtkPolyData for point cloud data (PC) and pose data (P)
    polyData_PC = vtk.vtkPolyData()
    polyData_P = vtk.vtkPolyData()

    # Figure out which PolyData is which
    if polyDataA.GetFieldData().GetArray("timestamp") is not None and \
            polyDataB.GetPointData().GetArray("timestamp") is not None:
        polyData_PC = polyDataA
        polyData_P = polyDataB
    else:
        if polyDataB.GetFieldData().GetArray("timestamp") is not None and \
                polyDataA.GetPointData().GetArray("timestamp") is not None:
            polyData_PC = polyDataB
            polyData_P = polyDataA
        else:   # If none of the configuration above is met, raise an exception
            raise Exception("\nOne or both of the inputs don't have a \"timestamp\" Point/Field Data\n"
                            "Is this data coming from the \"Paraview Tango Recorder\" app ?\n"
                            "The input that ends with \'_poses.vtk\" must have a \"timestamp\" PointData\n"
                            "The input that ends with \'*.vtk\" must have a \"timestamp\" FieldData\n")

    # If the pose data doesn't contain an "orientation" PointData array, raise an exception
    if polyData_P.GetPointData().GetArray("orientation") is None:
        raise Exception("\nThe Pose file (that ends with \"_poses.vtk\") has no dataArray called \"orientation\"\n")


    ############# Find the point cloud timestamp #############

    timestamp_PC = polyData_PC.GetFieldData().GetArray("timestamp").GetTuple(0)[0]
    #print "Point cloud timestamp: " + str(timestamp_PC)


    ############# Find the closest timestamp in the poses #############

    timestampArray_P = polyData_P.GetPointData().GetArray("timestamp")

    minDiff = 1e10
    closestIndex = 0
    for i in range(0, timestampArray_P.GetNumberOfTuples()):
        diff = abs(timestampArray_P.GetTuple(i)[0]-timestamp_PC)
        if diff < minDiff:
            closestIndex = i
            minDiff = diff

    #print "Closest Pose timestamp: " + str(timestampArray_P.GetTuple(closestIndex)[0])
    #print "Index: " + str(closestIndex)


    ############# Calculate the pose transform #############

    q = polyData_P.GetPointData().GetArray("orientation").GetTuple(closestIndex)

    # Add the orientation
    # Warning: orientation gives (x, y, z, w) but vtkQuaternion takes (w, x, y, z)
    myQuaternion = vtk.vtkQuaternionf(q[3], q[0], q[1], q[2])
    rotMatrix = np.zeros((4, 4))
    rotMatrix[3, 3] = 1
    myQuaternion.ToMatrix3x3(rotMatrix[0:3, 0:3])

    # Add the translation components
    pointArray_P = polyData_P.GetPoints()
    translation = pointArray_P.GetPoint(closestIndex)
    rotMatrix[0:3, 3] = translation


    ############# Create the Camera2Device transform #############

    # Camera 2 Device extrinsic translations (Hardcoded, extracted from the device)

    Camera2DeviceTFM = np.array([[0.9999981, 0.0018835695, 4.7361093e-5, 0.061678894],
                                 [0.0018196433, -0.9719693, 0.23510009, 0.0036567878],
                                 [4.8883393e-4, -0.23509958, -0.9719711, -0.0041324375],
                                 [0.0, 0.0, 0.0, 1.0]])
    # print Camera2DeviceTFM


    ############# Apply the transforms to the point cloud #############

    vtkTFM = vtk.vtkTransform()
    vtkTFM.PostMultiply()
    vtkTFM.Identity()
    vtkTFM.Concatenate(Camera2DeviceTFM.flatten())
    vtkTFM.Concatenate(rotMatrix.flatten())

    vtkTFMFilter = vtk.vtkTransformPolyDataFilter()
    vtkTFMFilter.SetTransform(vtkTFM)
    vtkTFMFilter.SetInputData(polyData_PC)
    vtkTFMFilter.Update()

    pdo.ShallowCopy(vtkTFMFilter.GetOutput())


def RequestInformation():
    import vtk

    ############# Get I/O #############

    # Get the two inputs, and the output
    polyDataA = self.GetInputDataObject(0, 0)
    polyDataB = self.GetInputDataObject(0, 1)
    pdo = self.GetPolyDataOutput()

    # If only one input is given, raise an exception
    if polyDataA is None or polyDataB is None:
        raise Exception("\nThis filter takes 2 inputs:\n"
                        "Point Cloud Data files: pc_HHMMSSDD_NNN.vtk\n"
                        "Pose Data file: pc_HHMMSSDD_poses.vtk\n"
                        "Note that ParaView groups all the Point Cloud Data files in one\n")

    # Initialize vtkPolyData for point cloud data (PC) and pose data (P)
    polyData_PC = vtk.vtkPolyData()
    polyData_P = vtk.vtkPolyData()

    if polyDataA.GetFieldData().GetArray("timestamp") is not None and \
            polyDataB.GetPointData().GetArray("timestamp") is not None:
        pointCloudPortIndex = 0
    else:
        if polyDataB.GetFieldData().GetArray("timestamp") is not None and \
                polyDataA.GetPointData().GetArray("timestamp") is not None:
            pointCloudPortIndex = 1
        else:   # If none of the configuration above is met, raise an exception
            raise Exception("\nOne or both of the inputs don't have a \"timestamp\" Point/Field Data\n"
                            "Is this data coming from the \"Paraview Tango Recorder\" app ?\n"
                            "The input that ends with \'_poses.vtk\" must have a \"timestamp\" PointData\n"
                            "The input that ends with \'*.vtk\" must have a \"timestamp\" FieldData\n")

    def setOutputTimesteps ( algorithm , timesteps ):
        "helper routine to set timestep information"
        executive = algorithm . GetExecutive ()
        outInfo = executive . GetOutputInformation (0)
        outInfo.Remove ( executive.TIME_STEPS () )
        for timestep in timesteps :
            outInfo . Append ( executive . TIME_STEPS () , timestep )

            outInfo . Remove ( executive . TIME_RANGE () )
            outInfo . Append ( executive . TIME_RANGE () , timesteps [0])
            outInfo . Append ( executive . TIME_RANGE () , timesteps [ -1])

    def getInputTimesteps( algorithm, portindex):
        "helper routine to set timestep information"
        executive = algorithm . GetExecutive ()
        inInfo = executive . GetInputInformation (0, portindex)
        return inInfo.Get(executive.TIME_STEPS())

    myrange = getInputTimesteps(self, pointCloudPortIndex)
    setOutputTimesteps(self, myrange)
