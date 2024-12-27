# Bachelor-Thesis
## Optimization approaches for the urban delivery problem with trucks and drones 

![Drones](/imgs/droniDHLtagliato.jpg)
 
In this study, we examine a problem characterized by the following features:

- A heterogeneous fleet composed of drones and standard vehicles such as trucks or ships.
- It is a multi-period problem involving mixed-integer linear programming: some variables are integer while others are continuous.
- The focus is not only on solving scheduling problems for material delivery but also on the recharging of drone batteries, which occurs within the standard vehicles.
- Customers can only be served by drones.
- Each drone mission can deliver only one "package" to each customer, and the drone can recharge on the standard vehicle after each mission.
- The "parking" locations for both trucks and ships, unlike other literature, do not coincide with customer locations but are situated at a set of points that are distinct from those of the customers.

The criteria considered as the final objectives are:
1. **SUSTAINABILITY**: Minimizing energy costs in the objective function.
2. **SERVICE QUALITY**: Minimizing the maximum completion time.
3. **PRODUCTIVITY**: Maximizing the number of customers served.

The problem consists of determining the locations where the standard vehicle should stop to allow drones to serve customers within the required time windows while minimizing only the first two objectives mentioned above.
