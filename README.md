# particle_filter_project
Enrique Collin and Katie Hughes

**How you will initialize your particle cloud (initialize_particle_cloud()).**
We will pick points and orientations uniformly random from within the dimensions of the map. If needed we will re-pick each point until it is within the confines of the map. We will test this by both eyeballing the initial data and plotting it over the grid to make sure it is well distributed.

**How you will update the position of the particles will be updated based on the movements of the robot (update_particles_with_motion_model()).**
We will move each particle according to the movements of the robot. So if the robot moves forward by 2, each particle will move forward by 2. We will test this examining how particle distribution overall and for some individual particles changes after the robot moves.

**How you will compute the importance weights of each particle after receiving the robot's laser scan data (update_particle_weights_with_measurement_model())**
We will compute the weight for each particle as 1 divided by the sums of the absolute differences between each of that particle’s distances to a wall (360 of them since scan gives that many) and the equivalent direction’s distance to the wall for the robot. We will test this by printing out some particle’s weights and computing them in a simpler programming context to ensure they are correct. If a particle has a high weight, it’s close to the actual sensor measurements. 

**How you will normalize the particles' importance weights (normalize_particles()) and resample the particles (resample_particles()).**
We will normalize the particles’ importance weights by dividing each weight by the sum of all the weights, thereby making the sum 1. We will test this by making sure the sum of weights is 1 after it occurs. We will resample the particles using the python function random.choices, which will allow us to pick according to our weight distribution. We can test this by printing out the distribution of things chosen and making sure it is approximately that of our distribution. Every time we resample we will keep the same number of particles. 

**How you will update the estimated pose of the robot (update_estimated_robot_pose())**
One way we have considered estimating the pose of the robot is  as the average of all the particle positions. While this will be quite bad when there are many positions, as the weights converge it will become more accurate. Another implementation we’re considering is setting the estimated robot location to the location that had the maximum particle weight. This may be more useful in the case where there are multiple locations where the robot is likely to be. We will test this by running the program and looking at the estimated pose, and seeing how making it the average performs at different time steps. 

**How you will incorporate noise into your particle filter.**
We will make sure the number of particles is very large. This will ensure that too many actually correct locations are not eliminated due to quickly not being sampled. Further, perhaps the method of estimating the pose off the robot with an average will cause noise to cancel out (eg, some particles will be too high in x and others too low in x, the average will help cancel this).  We will test that this is sufficient by running the program; if our robot’s pose gets offset too quickly it will indicate we’re not doing enough. We will also try to incorporate some random noise every time we move the particles to reflect the fact that there is some drift everytime the robot physically moves. 

**A brief timeline sketching out when you would like to have accomplished each of the components listed above.**
Our plan is to, starting from the first task tomorrow, complete a task every other day. There are 5 tasks so at this pace we will finish the 5 tasks on February 7, leaving us 3 days to debug and get the robot out of the house. 
