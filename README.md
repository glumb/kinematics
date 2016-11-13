#kinematics.js

[![GitHub license](https://img.shields.io/badge/license-MIT-blue.svg)](https://raw.githubusercontent.com/glumb/kinematics/master/LICENSE.md)
[![Travis](https://img.shields.io/travis/glumb/kinematics.svg)](https://travis-ci.org/glumb/kinematics)
[![npm](https://img.shields.io/npm/v/kinematics.svg)](https://www.npmjs.com/package/kinematics)
[![Codecov](https://img.shields.io/codecov/c/github/glumb/kinematics.svg)]()

6DOF robot kinematics in JavaScript.

##Install
```console
npm install kinematics --save
```

##Use
```js
const Kinematics = require('kinematics').default

const geometry = [
      [1,  1,  0], // V0: 1x 1y
      [0, 10,  0], // V1: 10y
      [5,  0,  0], // V2: 5x
      [3,  0,  0], // V3: 3x
      [0, -3,  0], // V4: -3y
    ]

const RobotKin = new Kinematics(geometry)

let angles = [1.57, 1.2, 0, 0.3, 2.2, 1.1]

const pose = RobotKin.forward(...angles)

angles = RobotKin.inverse(...pose)
```
##Geometry
The geometry array consists of 5 entries describing the links *V0-V5*. Each *Vn* is a tuple of 3 coordinates from *Jn* to *Jn+1*.
One constraint: The y,z of *V3* and x,z of *V4* must be 0 for the kinematics to work.
<p align="center">
  <img align="center" src="https://cloud.githubusercontent.com/assets/3062564/20245029/3a1f17b6-a997-11e6-9e19-a834144c1ea5.png" width=50%/>
</p>
<p align="center">
  <img align="center" src="https://cloud.githubusercontent.com/assets/3062564/20245030/3c43008e-a997-11e6-9228-26ea31caa072.png" width=70%/>
</p>

##API

**forward**

```js
RobotKin.forward(R0, R1, R2, R3, R4, R5)
```
returns
```
[ 
  [  0,      0,     0 ], //J0
  [  0.5,    1,  -0.8 ], //J1
  [ -0.2, -8.8,   0.3 ], //J2
  [  1.8, -5.6,  -2.8 ], //J3
  [  3.0, -3.6,  -4.7 ], //J4
  [  4.7, -1.3,  -5.5,  1,  6,  -2.8 ] //J5 + TCP Euler angles
]
```

**inverse**

*X,Y,Z* coordinates, *A,B,C* Euler angles in order 'abc'.

```js
RobotKin.inverse(X, Y, Z, A, B, C)
```
returns
```
[  2, 1.6,  2.1, -3.5,  1, -1.5 ] //array of angles
[  1, 2.3,  3.1,  NaN, NaN, NaN ] //NaN for out of reach angles
```
