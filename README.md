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
====================================================================================================================================================================
# Writeup
## Gif
![gif](particle_filter.gif)


## Objectives description (2-3 sentences): Describe the goal of this project
The goal of this project was to solve the robot localization problem using a particle filter algorithm. More specifically, we had to write code that would determine a robot's position on a given map using its movements and senses. 

## High-level description (1 paragraph): At a high-level, describe how you solved the problem of robot localization. What are the main components of your approach?
We solved the problem by assigning weights to particles according to how much their senses (ie, distance to nearest object in various directions) agreed with the senses of the robot, moving the particles as the robot moved, and resampling particles according to the assigned weights. Over time, the particle cloud converges to the robot's acutal location. We start by initializing the map with the particles placed uniformly. Then, whenever the robot moves far enough: (1) we update all particle positions so they move as the robot moved (2) using a likelihood field and estimated robot pose to assign weights to the different particles and (3) resampling particles according to weights (4) updating the estimated robot pose.

## For each of the 3 main steps of the particle filter (movement, computation of importance weights, and resampling), please provide the following:
### Code location (1-3 sentences): Please describe where in your code you implemented this step of the particle filter.
### Functions/code description (1-3 sentences per function / portion of code): Describe the structure of your code. For the functions you wrote, describe what each of them does and how they contribute to this step of the particle filter.
#### Movement
The movement of the particles is handled in the `update_particles_with_motion_model` function. 

This function computes how far the robot has moved in the x and y directions and how far it has turned in its angle. It then shifts each particles x and y positions and angles by these quantities plus some random Gaussian noise (added to account for the noise in the robot's sensors). 

#### Computation of importance weights
After being all initialized to 1 in `initialize_particle_cloud`, weights for particles are updated first in `update_particle_weights_with_measurement_model`, after which they are normalized in `normalize_particles`. Note that the second of these functions relies on `update_estimated_robot_pose` being called after resampling since it uses the robot's estimated pose.

The function `update_particle_weights_with_measurement_model`uses the likelihood field we wrote in class 06 to assign weights to each particle. In short, this works by translating and rotating the robot's sensor measumreents tom atch the particle's location and orientation, and then finding the nearest obstacle for each measurement end point. This is done for each sensor measurement (in practice we used the sensor measurement from scan topic for one out of every 10 degrees between 0 and 360). `normalize_particles` works by dividing every particle weight by the sum of particle weights so that the particle weights will sum to 1. `update_estimated_robot_pose` simply computed the unweighted mean of the particle locations. 

#### Resampling
Resmapling is handled by the `resample_particles` function. This function works by calling the given function `draw_random_sample` with the importance weights of each particle to draw the same number of particles distributed according to their weights.

## Challenges (1 paragraph): Describe the challenges you faced and how you overcame them.
In writing the code, one challenge we faced was that the initial particle positions weren't being visualized depending on the number of particles used. We determine that this was caused when only a few particles were used and the particle cloud publisher wasn't set up by the time they were all chosen, so we fixed it by adding a quick `rospy.sleep` call after initialization of particles. (Although rarely we have still run into this problem since adding the rospy.sleep statement.) After writing all of the code, our algorithm quickly converged, but not to the correct location. Attempting to solve this problem involved much parsing and thinking about the code. We overcame it in the end by (1) adding gaussian noise when updating particle positons to make it more forgiving (2) increasing the amount of data/number of angles used in `update_particle_weights_with_measurement_model`. 

## Future work (1 paragraph): If you had more time, how would you improve your particle filter?
If the robot's speed is made large enough after the particle cloud has convered, the robot outpaces the cloud. However, there are no particles to resample around its new location, so the cloud gets stuck following behind the robot, or stops moving at all if it gets far enough away. With more time, we would fix this. One possible fix might be to make the robot check for position changes more frequently/after less movement. We could also add functionality for the robot to recalculate its position if it gets moved. If we had more time, we might also change the robot to make it better at determining its position on other maps. As of now the robot seems to converge to its position/pose quite quickly, but on another map where different positions are more similar, perhaps using the standard deviation of different particle positions to gauge uncertainty in robot pose to fine-tune how quickly the robot converges to a position might improve behavior. We could also perhaps improve behavior on other maps by fine-tuning the number of angles used in particle initialization and in the `update_particle_weights_with_measurement_model` function. 

## Takeaways (at least 2 bullet points with 2-3 sentences per bullet point): What are your key takeaways from this project that would help you/others in future robot programming assignments working in pairs? For each takeaway, provide a few sentences of elaboration.
- Focus on understanding the algorithms. We think a lot of confusion regarding how to proceed with programming and how to fix errors we were running into was easily fixed once we reviewed and thought about what the particle filter algorithm was doing and the different components of it. For instance, the challenge mentioned above where our particle cloud localized around the wrong position was easily fixed upon reviewing the likelihood field code and the fact that it relies upon using as much data/as many scan angles as possible. The actual code isn't that difficult once you understand the algorithms/concepts.
- When you get stuck, come together. While generally dividing up the work turned out well for us, when we got stuck or needed to plan out how something tricky would work, coming together to meet and discuss it was crucial for moving forward. We stayed for the entirety of the last class session to discuss the problems our robot was having, and by bouncing ideas off each other managed to come up with solid plans for how to move forward and fix the issues we were facing. Coming up with solutions alone likley would have taken longer.
- Paying attention to the coding practices of your partner can be useful. For instance, I (Enrique) noticed that Katie had a habit of adding print statements to indicate what was going on in the program. If everything works fine this doesn't add anything, but for debugging, it makse it a lot easier to figure out where things are going wrong. I learned from this habit of hers and started using it, finding it quite helpful.
