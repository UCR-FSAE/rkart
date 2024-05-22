---
title: Launch Structure
description: A guide in my new Starlight docs site.
---
#### Purpose / Use cases
- Components shall be modularized
- There should be no hard dependencies, the communication between components shall be specified via standard ROS msgs
- Hardware and software design shall mirror each other
- Startup sequence, if needed, shall be enforced. 
  - Such as camera software should start AFTER camera hardware is started. 
#### Design
[Launch Scheme](https://n36411s2sqp.larksuite.com/wiki/D6XbwkyITid4PXkm6qMu5RWBslf)