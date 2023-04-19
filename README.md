# Collision Checking Library

This repository contains a collection of structs, functions and classes to
perform collision queries and minimum distance computations for simple geometric
shapes. The code is intended and most suitable for motion planning and control
applications (i.e,. not dynamics simulations).

## Overview

**Distance computation** The most basic functions provide distance computation
for sphere, capsule (a.k.a. sphere swept line) and box geometries. These
functions can be found in files named `distance_<object>_<object>.h`. They are
suitable for hard real-time and non real-time systems and can be compiled as
CUDA code for use on GPUs. These can be used as building blocks for higher-level
functions. Some serial CPU versions are provided in the repository (see below).

**Composite object distance computation** `compute_collisions.h` provides a free
function that performs distance computations or collision queries for
`CollisionObjects` and a `VoxelMapObject`. The `CollisionObjects` are a
collection of composite objects that are themselves composed of supported
primitive shapes. Each composite object will typically be a single robot link,
an object grasped by a robot or a static environment object with simple
geometry. The `VoxelMapObject` is a 'sphere world' intended to represent an
environment map that may contain a large number of elements and is generated
from sensor data via a voxel map, surfel map or similar representation. There
is some basic AABB-based pre-filtering of collision pairs to accelerate distance
queries. Which object pairs are considered can be configured via the
`ObjectFlags` variables, see `object_id.h`.

**Collision queries for articulated systems** The `AssemblyCollisionChecker`
template provides a higher-level interface for performing collision queries for
articulated systems with a floating base and tree topology consisting of hinge
and prismatic joints. The class template provides methods that also perform the
required forward kinematics. Most applications will want to start with this.

## Things to Tweak

If you intend to integrate this into an existing system, you will probably want
to edit the code in `collision_checking/logging.h` to use whatever the target
system uses. Likewise, you might want to replace any calls to absl logging
functions with something more appropriate.
