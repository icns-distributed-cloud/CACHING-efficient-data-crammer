# Efficient Data Crammer

Efficient Data Crammer(EDCrammer) performs caching rate-control between a core cloud and an edge cloud, and cache capacity auto-scaling for resource-limited edge clouds.
EDCrammer considers real-time caching to an edge cloud. 
It can calculate the next amount of caching data by Proportional–Integral–Derivative (PID) controller.
In the calculation, only current cache utilization is required. 
It can performs lightweight calculation. 
By using previous PID output and current PID output, it can estimate the amount of consumed data and the data consumption rate between an edge cloud and a user.
Based on the estimation, it can perform cache capacity auto-scaling for an edge cloud.
