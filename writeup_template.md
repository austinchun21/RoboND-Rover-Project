## Project: Search and Sample Return
## Austin Chun
---


**The goals / steps of this project are the following:**  

**Training / Calibration**  

* Download the simulator and take data in "Training Mode"
* Test out the functions in the Jupyter Notebook provided
* Add functions to detect obstacles and samples of interest (golden rocks)
* Fill in the `process_image()` function with the appropriate image processing steps (perspective transform, color threshold etc.) to get from raw images to a map.  The `output_image` you create in this step should demonstrate that your mapping pipeline works.
* Use `moviepy` to process the images in your saved dataset with the `process_image()` function.  Include the video you produce as part of your submission.

**Autonomous Navigation / Mapping**

* Fill in the `perception_step()` function within the `perception.py` script with the appropriate image processing functions to create a map and update `Rover()` data (similar to what you did with `process_image()` in the notebook). 
* Fill in the `decision_step()` function within the `decision.py` script with conditional statements that take into consideration the outputs of the `perception_step()` in deciding how to issue throttle, brake and steering commands. 
* Iterate on your perception and decision function until your rover does a reasonable (need to define metric) job of navigating and mapping.  

[//]: # (Image References)

[image1]: ./writeup_images/color_thresh.jpg
[image2]: ./calibration_images/example_grid1.jpg
[image3]: ./calibration_images/example_rock1.jpg 
[image4]: ./writeup_images/blur_tile.jpg
[gif1]: ./output/notebook_video.gif
[gif2]: ./writeup_images/Final_Video_pt1.gif
[gif3]: ./writeup_images/Final_Video_pt2.gif
[gif4]: ./writeup_images/Final_Video_pt3.gif

## [Rubric](https://review.udacity.com/#!/rubrics/916/view) Points
### Here I will consider the rubric points individually and describe how I addressed each point in my implementation.  

---
### Writeup / README

#### 1. Provide a Writeup / README that includes all the rubric points and how you addressed each one.  You can submit your writeup as markdown or pdf.  

You're reading it!

### Notebook Analysis
#### 1. Run the functions provided in the notebook on test images (first with the test data provided, next on data you have recorded). Add/modify functions to allow for color selection of obstacles and rock samples.

To detect obstacles and rocks, I added more functionality to the `color_thresh()` function. Instead of taking just navigation threshold values and outputting a 1 channel binary image of the input image, `color_thresh()` now takes in threshold values for Obstacles and Rocks, and outputs a 3 channel binary image.

I also decided to use color thresholding using HSV rather than RGB. I used a low and high threshold for the HSV ranges. By specifying ranges of HSV for each class (Obs, Rock, and Nav) left some pixels unclassified (that is why there are black regions between the Red and Blue). I was relatively conservative with the color thresholding for higher confidence in the classifications. 

In the below figure, you can see the original image on the left, and the thresholded image in the middle. The navigable terrain is clearly colored as blue, the rock in the distance is identified as green, and the rock walls are colored red. Note that the sky is unclassified, and the tread marks on the ground are unclassified.

![color_thresh()][image1]


#### 1. Populate the `process_image()` function with the appropriate analysis steps to map pixels identifying navigable terrain, obstacles and rock samples into a worldmap.  Run `process_image()` on your test data using the `moviepy` functions provided to create video output of your result. 

I adjusted `rover_coords()` and `pix_to_world()` to process the the obstacle and rock pixels just as the navigable pixels. (I also combined the Rotate and Scale functions into `pix_to_world()`). 

I also wanted to add a weighting to the classifications based on radial distance (meaning I am more confident in the classifications of the close terrain). In order to do that, I changed `pix_to_world()` to output a 'tile' (rather than arrays of pixels). The tile is a 50x50 pixel 3 channel RGB image with the Rover centered in the tile, and the pixel classifications oriented properly. (The tile is rotated, and scaled, but not translated). Additionally, custom weights can be used for each class, so that every time you see a rock pixel, you add 255, whereas for navigable pixels, you only add 10, because rock pixels are more rare and shouldn't be overwhelmed by the more common obstacle and navigable terrain.

Now, with the tile of the Rover's new scan, I can apply a radial weighting filter to attenuate classifications further from the rover. I do this by creating a tile of the same size with the desired radial weighting (weights ranging from 1 in the middle, to 0 further out). Then simply multiply the weighting with the tile to get the blurred tile. The blurring tile is on the left, the scan tile is in the middle, and the right tile is the result.

![blur_tile][image4]

Now, with the blurred tile, the world_map must be updated. Since I wanted `data.worldmap` to keep the binary classifications of the terrain, I added a new variable, `data.worldmap_sum` to keep track of the accumulated classifications for each pixel. I wrote a new function `update_worldmap()` to do the following: 

* Position the blurred tile of the Rover scan to the correct location using the global coordinates 
* Add the tile to `data.worldmap_sum` (Being careful of edge cases) 
* Update `data.worldmap` for the tile region, taking the maximum classification for each pixel

A sample video output from the notebook is shown below, or can be found in `output/notebook_video.mp4`.

![gif][gif1]


### Autonomous Navigation and Mapping

#### 1. Fill in the `perception_step()` (at the bottom of the `perception.py` script) and `decision_step()` (in `decision.py`) functions in the autonomous mapping scripts and an explanation is provided in the writeup of how and why these functions were modified as they were.

** Perception **

I copied over all of the changed functions from the notebook to this file. One addition I made to the perception step was calculating front, right, and left distances (`get_front_dist()` line 122, `get_right_dist()` line 136, and `get_left_dist()` line 148).

`get_front_dist()` returns the distance (in meters) to the closest obstacle in front of the rover. I used Cartesian coordinates of the obstacle pixels, looking at a 1 m wide channel of obstacle pixels (+/- 0.5m in y), and returning the minimum x value. (I scaled the pixel to meters before returning the distance)

`get_right_dist()` and `get_left_dist()` are fundamentally the same. Using the polar coordinates for navigable terrain, I return the average distance of the navigable terrain for a small slice along the right and left edges of the coordinate transformed scan. Technically, these are right/left diagonal distances (at about 45 degrees) and not directly to the right/left. (I considered trying to extract the direct right/left distances by using the worldmap generated by the rover, but that was too reliant on the accuracy of the worldmap and did not have great resolution). 

Also, in `perception_step()`, I added some conditionals to limit useless calculations. The first is at line 344, which is to skip all calculations if the rover is just picking up a rock. Down at line 372, I do not want to update the front, right, and left distances unless the rover is flat. Lastly at line 378, I do not want to update the worldmap unless the rover is flat and actually moving.

** Decision **

I made a lot of changes to `decision_step()`. Overall, I added proportional control to steering, and throttle. I changed the steering to follow the right wall. I added a 'rock' mode, to go pick up rocks that it sees. I added a 'stuck' mode for when the robot gets stuck.

There are now four rover modes: forward, stop, rock, and stuck. 

In forward mode, the rover will try and follow the right wall. The desired right wall distance (ex 1.5m) is defined in the Rover class (When a passage is narrow and staying 1.5m from the right wall may make the rover too close to the left wall, the desired distance is changed to be the average of the left and right distance, meaning it aims for the middle of the passage, line 37). Proportional control is used for steering (line 42/44), as well as velocity and acceleration. I used two different constants for left/right steering to more aggressively steer away from the wall, and gently follow the wall. The desired velocity is dictated by the front distance (line 46), and the throttle is dictated by the difference between the current velocity and the desired velocity (line 47). As before, there is a stop condition if there is not enough navigable pixels ahead (line 30), but I also added that there cannot be an obstacle too close ahead using the front dist (line 27). If the rover sees a rock, and it is close-by (within 4 meters), then it will transition into 'rock' mode (line 53-56).

Stop mode is similar to before (lines 80-95), except with the addition that there must not be an obstacle too close in front before returning to forward mode. 

In rock mode (lines 61-77), the rover uses proportional control for steering and velocity/throttle just as in forward mode, except now with the rock distance and angle as the inputs.

No matter the mode, the rover will always check whether it is stuck. Stuck is defined as having a non-zero throttle with a zero velocity. I added some timing such that the rover must be stuck for some 3 seconds before it is actually considered stuck.

In stuck mode (lines 98-116), the rover will alternate between trying to rotate in place, and going forwards (it alternates every 2 seconds). Once there is a significant velocity, then it is considered unstuck.

A small hack to make sure the rover starts on a wall is to make the rover go straight for the first 5 seconds (line 132). 

I've also added a small case (line 136) to stop if a rock is nearby.

#### 2. Launching in autonomous mode your rover can navigate and map autonomously.  Explain your results and how you might improve them in your writeup.  

**Note: running the simulator with different choices of resolution and graphics quality may produce different results, particularly on different machines!  Make a note of your simulator settings (resolution and graphics quality set on launch) and frames per second (FPS output to terminal by `drive_rover.py`) in your writeup when you submit the project so your reviewer can reproduce your results.**

Here I'll talk about the approach I took, what techniques I used, what worked and why, where the pipeline might fail and how I might improve it if I were going to pursue this project further.  

To summarize the main changes I made:

* Terrain is classified as obstacle, rock, or navigable using HSV color thresholding
* The classifications are weighted by radial distances, to attenuate classifications further away
* The rover follows tries to follow the right wall
* If it sees a rock close-by, it will go and pick it up
* If the rover ever gets stuck, it will try to free itself

Overall, the autonomous navigation works rather well. I've tested the algorithm on Screen Resolution: 1024x768 on all three Graphics Quality settings at 13-15 fps for Fantastic, 15-18 fps for Good, and 17-19 fps for Fastest. I could pretty consistently get >80% Mapped with >80% Fidelity, but it takes a long time (sometimes over 20 minutes).

I found that the HSV thresholding worked well, though I am not positive how it compares to RGB since generally the environment coloring is made to be easily distinguishable. One difficult I faced in the color thresholding was the classification of tread marks as obstacles. That resulted in the front distance (the closest obstacle in front) to be inaccurate, and could corner the rover until the tread marks disappeared. 

Weighting the classifications by distance seemed to help with the fidelity of the mapping. One addition could be setting a minimum threshold, such that only locations that have been classified consistently are fixed in the worldmap (often I saw that pixels far away are quickly classified as Obstacles, even though they aren't. Such mistakes would hurt algorithms that try to explore new areas by closing off navigable areas with obstacle areas)

One fundamental shortcoming of the current decision algorithm is that it doesn't consider the past. It doesn't use the worldmap that has been built. It only makes decisions on the current camera measurements (except for when determining when the rover is stuck, which uses some temporal data). One manifestation of this shortcoming is when the rover gets stuck in an infinite loop. This can happen when the rover starts in a wide open space. It will start turning right, and just go in a circle forever (that is why I used the hack of going forward for the first 5 seconds to find a wall). Thus one major improvement would be to adjust the decision algorithm to use past information, specifically the worldmap that is built. By knowing where it has been, it can more efficiently explore unexplored areas.

Another example of how only using the current state to make decisions can be inefficient is when the robot veers off the right wall to go pick up a rock on the left wall. When it goes to pick up the wall, it has now switch walls, and thus does not end up exploring the rest of the canyons. Ultimately, it will loop back around to that area, and continue exploring the canyon without the distraction of the rock, but it obviously is not an efficient method of exploration. Thus it would be great to possibly log the state right before it diverted to go pick up the rock, then after it picks up the rock, move back to the previous state and continue as normal. 

Some comments on the stuck state. I found with the wall following algorithm the rover would often get stuck at rock outcroppings because the low camera view sees clear ground, but above the robot is a rock wall that pins it down. The current method of alternating between rotating and trying to go forward seems to work, but it admittedly is not very elegant. 

I recorded a 12 minute autonomous run. The video is sped up by 8x, and split into three portions, so each gif corresponds to 4 minutes. The simulator was set to 1024x768 at Fantastic, and ran at 13-15 fps. After 12 minutes, the rover had Mapped 81% with 91% Fidelity, located 6 rocks, and collected 5 of them. The full video can be found in `writeup_images/Final_Video.mp4`

![gif2][gif2]
![gif3][gif3]
![gif4][gif4]
