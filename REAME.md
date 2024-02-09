Package for harmonic filter for liquid handling and relative subroutines

**Installation**
Package dependecies required:
- sudo apt update && sudo apt install ros-noetic-kdl-conversions ros-noetic-kdl-parser 

The package is running several routines for the specific tests for sloshing experiments, each class has its own state machine.
Among them some classes provide general routines: 
- *FirFilter* provides a simple implementation for a FIR filter (now some formal libraries provide the same so formally not necessary anymore)
- *SS_filter* implements the harmonic filter for antisloshing for liquids
- *ComauSSGenerator* a trajectory generator exploting filters to produce the reference trajectory for comau
- *LiquidHandler* class for evaluation of liquid properties through force sensor measurements
- *Tool_posefinder* evaluator of liquid center of mass based on force sensor readings (exploited by liquidHandler)

The main routines exploits the classes above:
- *object_locator_server*: implements service for liquid CoM evaluation (requires inizialization with nothing on the robot tool)
- *rigid_object_locator_server*: implements service for rigid object CoM evaluation (requires inizialization with nothing on the robot tool)
- *sloshing_suppression......*: several demo implementing a sloshing test