#### Assignment3

#### Dubin's Car [Tutorial](https://cisprague.github.io/dubins/#/)

Obtain the source code by cloning this repository:

        git clone https://github.com/cisprague/dubins.git

Note that:

- Each steering angle controls[i] is considered to be constant between times[i] and times[i+1], socontrols must be one element shorter than times, i.e. len(controls) == len(times) - 1.
- The initial time must be zeros, i.e. times[0] == 0.
- Each steering angle must be admissible, i.e. -pi/4 <= controls[i] <= pi/4.
- The time sequence must increase, i.e. times[i+1] > times[i].
- The sequence time-step size must be greater than or equal to  0.01, i.e. times[i+1] - times[i] > 0.01.
