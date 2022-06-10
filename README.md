# Multibeam Optimization for Joint Communication and Sensing (JCAS)
As a part of the university project, I have optimized a set of multibeams for JCAS using Two-Step Iterative Least Square approach in MATLAB. The idea is coming from the
paper https://ieeexplore.ieee.org/document/8550811. 
Below, you can see an optimized multibeam with two subbeams: one for communication, another for sensing (click for better resolution).
![comAndSensBeams](https://user-images.githubusercontent.com/49762976/173124245-c8179273-62e8-464c-b4c3-09ebd1afa906.png)
As you may see in the following figure, the reproduced set of multibeams slightly differs from the multibeams suggested in the paper. One of the reasons is that no exact
simulation parameters were provided by the authors.
![m_merged](https://user-images.githubusercontent.com/49762976/173126120-8a58afea-2682-4f6f-90b1-d6f487fc4349.png)
Meanwhile, the desired properties are obtained: HPBW of the sensing beam is bigger covering larger area, sidelobe level is low and the multibeams are pointing into
the desired directions.
