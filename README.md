# AutonomousRobot

This project simulates geometric formation-based swarm navigation for autonomous robots operating in unknown and occluded environments. The system combines sequential line-bundle escape theory, formation control, and local geometric exploration to enable robots to navigate toward target regions while escaping blind alleys and obstacle-constrained spaces under limited sensing conditions.

---

# Running the Main Swarm Simulation
## Before running
You should first change directory to cpp_core/build then cmake .. then make for creating the CMake files for the C++ part to work. 

## Usage

```bash
python Swarm_run.py -n <num_runs> -m <map_name> -s <start_x> <start_y> -g <goal_x> <goal_y> -r <vision_range> -p <picking_strategy> -open_pts_type <type>
```
## Example
```bash
python Swarm_run.py -n 0 -m _MuchMoreFun.csv -s 5 5 -g 35 50 -r 10 -p g -open_pts_type o
```
## Documentation

See [doc.md](doc.md) for detailed explanations of the swarm formation controller, geometric navigation model, and overall system architecture. This file will be change more.

## Contact 
phuc.vu030506hp@hcmut.edu.vn for more info about this code

## Note
This is not entirely my code, the base of the code is from [https://github.com/ThanhBinhTran/autonomousRobot](https://github.com/ThanhBinhTran/autonomousRobot).  
