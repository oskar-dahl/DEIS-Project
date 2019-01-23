#!/usr/bin/env python
import math

"""
Author: Oskar Dahl
FuncName: calculateUncertainty
"""
def calculateUncertainty(covariancePosPrev, deltaDistance, deltaAngle, anglePrev, errorRate, deltaDistanceRight, deltaDistanceLeft, wheelBase):

    jacobianState = jacobianStateDifferentialDrive(deltaDistance, deltaAngle, anglePrev)
    jacobianMeasure = jacobianMeasureDifferentialDrive(deltaDistance, deltaAngle, anglePrev, wheelBase)
    covarianceMesaure = covarianceMatrixMeasure(errorRate, deltaDistanceRight, deltaDistanceLeft)

    covariancePos = errorPropagationLaw(jacobianState, covariancePosPrev, jacobianMeasure, covarianceMesaure)
    return covariancePos


"""
Author: Oskar Dahl
FuncName: errorPropagationLaw
parms Matrix<double, 3,3> Js, Matrix<double, 3,3> cmXkP, Matrix<double, 3, 2> Jm, Matrix<double, 2, 2> cmVat
"""
def errorPropagationLaw(jacobianState, covariancePosPrev, jacobianMeasure, covarianceMesaure):
    covariancePos = jacobianState * covariancePosPrev * zip(*jacobianState) + jacobianMeasure * covarianceMesaure * zip(*jacobianMeasure)
    return covariancePos


"""
Author: Oskar Dahl
FuncName: jacobianStateDifferentialDrive
"""
def jacobianStateDifferentialDrive(deltaDistance, deltaAngle, anglePrev):

    Js = [[1, 0, -deltaDistance*math.sin(anglePrev + deltaAngle/2)],
          [0, 1,  deltaDistance*math.cos(anglePrev + deltaAngle/2)],
          [0, 0,                 1                                ]]

    return Js

"""
Author: Oskar Dahl
FuncName: jacobianMeasureDifferentialDrive
"""
def jacobianMeasureDifferentialDrive(deltaDistance, deltaAngle, anglePrev, wheelBase):

    jm= [[ 1/2*math.cos(anglePrev + deltaAngle/2) - 1/(2*wheelBase)*deltaDistance*math.sin(anglePrev + deltaAngle/2), 1/2*deltaDistance*math.cos(anglePrev + deltaAngle/2) + 1/(2*wheelBase)*deltaDistance*math.sin(anglePrev)],
        [  1/2*math.sin(anglePrev + deltaAngle/2) + 1/(2*wheelBase)*deltaDistance*math.cos(anglePrev + deltaAngle/2), 1/2*deltaDistance*math.cos(anglePrev + deltaAngle/2) - 1/(2*wheelBase)*deltaDistance*math.cos(anglePrev)],
        [                                       1/(2*wheelBase)  , -1/(2*wheelBase)                                                      ]]
    return jm

"""
Author: Oskar Dahl
FuncName: covarianceMatrixMeasure
"""
def covarianceMatrixMeasure(errorRate, deltaDistanceRight, deltaDistanceLeft):

    sigmaDistnace = errorRate * math.abs(deltaDistanceRight)
    sigmaAngle    = errorRate * math.abs(deltaDistanceLeft)

    covarianceMatrixMeasure = [[sigmaDistnace, 0.0],
                               [0.0,    sigmaAngle]]
    return covarianceMatrixMeasure
