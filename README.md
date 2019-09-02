# MultiRobotsRendezvous

Use Tverberg point (Implementation of the paper ["An efficient algorithm for fault-tolerant rendezvous of multi-robot systems with controllable sensing range"](https://ieeexplore.ieee.org/document/7487153)) and centerpoint to achieve fault-tolerant multi-robot consensus.


## Instruction
```
Tested on python 3.7.3

Four executable files:
- python main.py (general cases)
- python faultTolerantRendezvous.py
- python attackToOnePoint.py
- python resilienceSuperiority.py

Parameters:
n: number of robots
n_faulty: number of faulty robots (red)
n_fault_free: number of fault-free robots (blue)
method: "tver" for Tverberg point based concensus or "center" for centerpoint based consensus
```

## Dependencies
```
pip install shapely
```

## Comparison cases
---
### faultTolerantRendezvous.py
```
- 100 robots (fully connected graph)
- 30 of which are faulty and not moving
- fault-free robots should achieve consensus for both Tverberg point based and centerpoint based algorithms.
```

#### Tverberg point based fault-tolerant rendezvous
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/tver0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/tver2.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/tver5.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/tver10.jpg)

#### Centerpoint based fault-tolerant rendezvous
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/center0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/center2.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/center5.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/faultTolerantRendezvous/center10.jpg)

---

### attackToOnePoint.py
```
- 100 robots (fully connected graph)
- 52 faulty robots (n/2 + 2) at one point are guaranteed to make Tverberg point based fault-free robots converge to one point
- 67 faulty robots (2/3n) at one point are guaranteed to make centerpoint based fault-free robots converge to one point
- Showing centerpoint is more resilient to Tverberg point.
```

#### Tverberg point based rendezvous attacked to one point
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/tver0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/tver4.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/tver9.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/tver15.jpg)

#### Centerpoint based rendezvous attacked to one point
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/center0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/center4.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/center9.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/attackToOnePoint/center15.jpg)

---

### resilienceSuperiority.py
```
- 26 robots, 6 faulty robots in the left, 6 faulty robots in the right, 14 fault-free robots in the middle, divided into half equal clusters of 7 fault-free robots. 
- Sensing range is 1.5 and each fault-free agent is connected to all the other fault-free robots and 6 faulty robots in either left cluster or right cluster, but not both. So each fault-free robot have 20 neighbors in the beginning.
- For Tverberg point based rendezvous, fault-free robots fail to converge to one point. Because when n = 20, Tverberg point is resilient to math.ceil(20/4) - 1 = 4 faulty robots.
- However, for centerpoint based rendezvoous, fault-free robots converge to one point. Because when n = 20, centerpoint is resilient to math.ceil(20/3) - 1 = 6 faulty robots.
```

#### Tverberg point based rendezvous not resilient
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/tver0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/tver5.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/tver9.jpg)

#### Centerpoint based rendezvous resilient to more faulty points
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/center0.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/center7.jpg)
![](https://github.com/JianiLi/MultiRobotsRendezvous/blob/master/figure/resilienceSuperiority/center13.jpg)
