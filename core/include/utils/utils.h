/* +---------------------------------------------------------------------------+
   |                     Mobile Robot Programming Toolkit (MRPT)               |
   |                          http://www.mrpt.org/                             |
   |                                                                           |
   | Copyright (c) 2005-2018, Individual contributors, see AUTHORS file        |
   | See: http://www.mrpt.org/Authors - All rights reserved.                   |
   | Released under BSD License. See details in http://www.mrpt.org/License    |
   +---------------------------------------------------------------------------+ */

#include "lines.h"
#include "planes.h"
// ...

/** \defgroup calib_core

This module encapsulates an easy-to-use GUI application for the extrinsic calibration of different types of sensors.
It includes several data classes for the calibration algorithms.
The app estimates the relative poses of a set of sensors, including 2D and 3D LiDARs, RGB-D cameras, RGB cameras, and any combination between them.
Automatic and target-less calibration algorithms based on line matching, plane matching, and trajectory matching are implemented.
The user will be able to directly visualize the calibration results and compare different algorithms.

<div align="center">
<table border="0" cellspan="4" cellspacing="4" style="border-width: 1px; border-style: solid;">
<td> \image html instrumented_vehicle.png "Figure 1. Instrumented vehicle for semi-autonomous driving, equiped with several 2D LiDARs and cameras." </td>
</table>
</div>

*/
