// import 'babel-polyfill'
import 'mocha/mocha'
import { assert } from 'chai'
import K from '../src/index'

const geometries = [
  [
    [1, 1, 0],
    [0, 10, 0],
    [5, 0, 0],
    [3, 0, 0],
    [0, -3, 0],
  ],
  [
    [1, 1, 1],
    [0, 8, 2],
    [0, 10, 0],
    [5, 0, 0],
    [0, -6, 0],
  ],
]

const targetPoses = [
  [1, 1, 2, 1, 2, 3],
  [3, 8, 2, 4, 1, 3],
  [6, -6, -2, 0, 1, 3],
  [3, 8, 3, 4, 0, 3],
]

function eulerToVec(b, c) {
  const cb = Math.cos(b)
  const sb = Math.sin(b)
  const cc = Math.cos(c)
  const sc = Math.sin(c)

  return [
    cb * cc,
    cb * sc, -sb,
  ]
}

describe('#kinematics', () => {
  geometries.forEach((geometry) => {
    // if ik and fk break in the same way this test may not be sufficient
    // but this is very unlikely. :D
    const kin = new K(geometry)

    targetPoses.forEach((targetPose) => {
      it(`match inverse and forward kinematics ${targetPose}`, () => {
        const angles = kin.inverse(...targetPose)
        // get the TCP position [5]
        const pose = kin.forward(...angles)[5]

        const TOLERANCE = 0.000001

        // x
        assert.isBelow((pose[0] - targetPose[0]), TOLERANCE)
        // y
        assert.isBelow((pose[1] - targetPose[1]), TOLERANCE)
        // z
        assert.isBelow((pose[2] - targetPose[2]), TOLERANCE)
        // rotation a
        assert.isBelow((pose[3] - targetPose[3]), TOLERANCE)

        // direction - different Euler angles may point to the same direction
        const vecTarget = eulerToVec(targetPose[1], targetPose[2])
        const vecPose = eulerToVec(pose[1], pose[2])

        assert.isBelow((vecTarget[0] - vecPose[0]), TOLERANCE)
        assert.isBelow((vecTarget[1] - vecPose[1]), TOLERANCE)
      })
    })
  })
  it('use debug mode to cover everything', () => {
    const kin = new K(geometries[0])
    kin.debug = true
    kin.inverse(1, 1, 1, 0, 0, 0)
    kin.forward(0, 0, 0, 0, 0, 0)

    assert.equal(kin.debug, true)
  })

})
