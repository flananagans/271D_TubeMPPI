# MAE 271D Class Project
Shirley Shah, Niki Krockenberger, Will Flanagan

## Topic
Recreating and extending the work of a Tube-MPPI controller as described by Williams et al.

* G. Williams, B. Goldfain, P. Drews, K. Saigol, J. Rehg, and E. Theodorou, “Robust Sampling Based Model Predictive Control with Sparse Objective Information,” in Robotics: Science and Systems XIV, Robotics: Science and Systems Foundation, Jun. 2018. doi:10.15607/RSS.2018.XIV.042

## File Overview
This repository is organized into one main script (located in the `main` folder) that runs the simulation using instances of classes defined in the other folders

### Relevant files:          
```ruby
├── main
    ├── hw2_main # main script to run simulation
    ├── initWorkspace.m # adds classes to path
    └── plotTrajectory.m # used to create figures
├── MPPI # class defining the MPPI controller
    └── MPPI_Controller.m
├── System # classes defining the dynamical system
    └── DiscreteLinearSystem.m # discrete double-integrator system
└── tracks # classes that define different track types
    └── OvalTrack.m # track composed of two curves and an optional straight
```

