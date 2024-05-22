---
title: Global Planning Infrastructure
description: A guide in my new Starlight docs site.
---

#### Summary
The Global Planner will produce the next waypoint based on some static map information. 
The definition of static map maybe different based on different implementation of the Global planner, but the output of a series of waypoint based is enforced

#### Purpose / Use cases
Racing would require an optimal trajectory. However, the computation of the trajectory does not need to be real-time. 
Furthermore, in racing, we don't need to consider an exorbitant amount of obstacles and unknown scnearios, and therefore HD map might not be strictly necessary

#### Design

#### Inputs / Outputs / API
- Input
  - [Required] Odometry
  - [Required] Configuration file
  - [Optional] Some form of static map
- Output
  - nav_msgs/path
- Inner-workings / Algorithms
  - Depends on implementation
    - Currently have RacingPlanner and Navigation Planner
    - Navigation Planner consists of two types of algorithms
      - Potential Field Algorithm
      - Hybrid A* Algorithm

#### References / External links
[Race Planner](https://n36411s2sqp.larksuite.com/wiki/wikus5sPI9ZusbBryLiFafNkBlh)

[Navigation Planner](https://n36411s2sqp.larksuite.com/wiki/GIVfwH9Hnie1VMkwKGVulypqshc)

#### See also

[Global Planning Lark documentation](https://n36411s2sqp.larksuite.com/wiki/wikus6vr5E8zT2UATN7mv6Jwmng)

[Global Navigation Planning documentation](https://n36411s2sqp.larksuite.com/wiki/PP1bwCTKYiex4RkjeuOuNBZ8sse)