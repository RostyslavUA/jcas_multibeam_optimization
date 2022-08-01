# Multibeam Optimization for Joint Communication and Sensing (JCAS)
As a part of the university project, a set of multibeams for JCAS using Two-Step Iterative Least Square approach in MATLAB was optimized. The idea is coming from the
paper [Multibeam for Joint Communication and Radar Sensing Using Steerable Analog Antenna Arrays](https://ieeexplore.ieee.org/document/8550811).
Below, you can see an optimized multibeam with two subbeams: one for communication (pointing to the boresight), another for sensing.
<p align="center">
<img src="https://user-images.githubusercontent.com/49762976/182257923-7be103fa-8604-40e4-bd7c-0dd343f7e847.png" width="500" />
</p>
A benefit of this approach is that a set of multibeams can be computed offline saving computational resources and reducing the latency associated with the packet transmission. In the fiugre below, you can see 8 multibeams pointing to the boresight communication direction, while scanning across the whole range of equivalent directions. 
<p align="center">
<img src="https://user-images.githubusercontent.com/49762976/182258099-f6a9297c-cd4e-4cd2-a8c7-0a0ce337a017.png" width="500" />
</p>
In the figure, one can see that the desired properties of the beams are obtained: relatively low sidelobe level, scanning beams point to the desired scanning directions and the communication beams are pointing into the desired boresight direction.

More details on Multibeam Generation (my part) as well as BF codebook optimization and quantization (@MengshuaiZ) can be found in our report [Report.pdf](https://github.com/RostyslavUA/jcas_multibeam_optimization/files/9237855/Report_Zhang_Olshevskyi.pdf) and final presentation [Presentation.pdf](https://github.com/RostyslavUA/jcas_multibeam_optimization/files/9237858/slides.pdf).
