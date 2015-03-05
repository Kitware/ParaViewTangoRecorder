###################################################
######## Programmable filter script ########
###################################################

# Author: Casey Goodlett

# Output DataSet Type = Same as Input


############# Properties for auto-generated XML #############

Name = 'AccumulatePointClouds'
Label = 'Accumulate Point Clouds over time'
Help = 'This accumulates all the point clouds into a single dataset'


# Still don't know if these lines are actually necessary
NumberOfInputs = 1
InputDataType1 = 'vtkPolyData'
# OutputDataType = 'vtkPolyData' # omit this line to use 'same as input'

Properties = {}

def RequestData():
    import numpy as np
    import vtk.util.numpy_support as vnp

    try:
        x = self.UpdateTimeIndex
    except AttributeError:
        self.UpdateTimeIndex = 0

    inp = inputs[0]
    if self.UpdateTimeIndex < len(self.TimeValues) - 1:
        #print self.UpdateTimeIndex

        # If we are not done, ask the pipeline to re-execute us.
        self.UpdateTimeIndex += 1
        request.Set(
            vtk.vtkStreamingDemandDrivenPipeline.CONTINUE_EXECUTING(), 1)
        pts = inp.Points.copy()
        try:
            self.Cache.append(pts)
        except AttributeError:
            self.Cache = []
            self.Cache.append(pts)
    else:
        request.Remove(
                vtk.vtkStreamingDemandDrivenPipeline.CONTINUE_EXECUTING())
        self.UpdateTimeIndex = 0

        pts = np.concatenate(self.Cache)
        self.Cache = []

        pd = vtk.vtkPolyData()
        vpts = vtk.vtkPoints()
        vpts.SetData(vnp.numpy_to_vtk(pts, deep=True))
        pd.SetPoints(vpts)

        m = vtk.vtkMaskPoints()
        m.SetOnRatio(1)
        m.SetGenerateVertices(True)
        m.SetInputData(pd)
        m.Update()

        output.DeepCopy(m.GetOutput())

def RequestInformation():
    import vtk

    executive = self.GetExecutive()
    info = executive.GetInputInformation(0,0)
    outInfo = executive.GetOutputInformation(0)

    # Reset values.
    self.TimeValues = info.Get(
        vtk.vtkStreamingDemandDrivenPipeline.TIME_STEPS())

    # We accumulate all particles to one dataset so we don't really
    # produce temporal data that can be separately requested.
    outInfo.Remove(
        vtk.vtkStreamingDemandDrivenPipeline.TIME_STEPS())
    outInfo.Remove(
        vtk.vtkStreamingDemandDrivenPipeline.TIME_RANGE())

    return 1

def RequestUpdateExtent():
    try:
        x = self.UpdateTimeIndex
    except AttributeError:
        self.UpdateTimeIndex = 0

    info = self.GetExecutive().GetInputInformation(0,0)
    info.Set(vtk.vtkStreamingDemandDrivenPipeline.UPDATE_TIME_STEP(),
             self.TimeValues[self.UpdateTimeIndex])
