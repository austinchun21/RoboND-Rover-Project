import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    wall_crawl(Rover)
    # # Implement conditionals to decide what to do given perception data
    # # Here you're all set up with some basic functionality but you'll need to
    # # improve on this decision tree to do a good job of navigating autonomously!

    # # Example:
    # # Check if we have vision data to make decisions with
    # if Rover.nav_angles is not None:

    #     print("Len nav_ang:",len(Rover.nav_angles))

    #     # Check for Rover.mode status
    #     if Rover.mode == 'forward': 
    #         if(Rover.front_dist < Rover.min_front_dist):
    #             Rover.throttle = 0
    #             Rover.brake = Rover.brake_set
    #             Rover.steer = 0
    #             Rover.mode = 'stop'
    #             print(" FRONT NOT CLEAR")

    #         # Check the extent of navigable terrain
    #         elif len(Rover.nav_angles) >= Rover.stop_forward:  
    #             # If mode is forward, navigable terrain looks good 
    #             # and velocity is below max, then throttle 
    #             if Rover.vel < Rover.max_vel:
    #                 # Set throttle value to throttle setting
    #                 Rover.throttle = Rover.throttle_set
    #             else: # Else coast
    #                 Rover.throttle = 0
    #             Rover.brake = 0
    #             # Set steering to average angle clipped to the range +/- 15
    #             # Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    #             Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi / 45*15), -15, 15)

    #             # Adjust to minimize roll
    #             Rover.steer -= (180-abs(Rover.roll-180))*2
    #             print("Steer:",Rover.steer,(180-abs(Rover.roll-180)))

    #         # If there's a lack of navigable terrain pixels then go to 'stop' mode
    #         elif len(Rover.nav_angles) < Rover.stop_forward:
    #                 # Set mode to "stop" and hit the brakes!
    #                 Rover.throttle = 0
    #                 # Set brake to stored brake value
    #                 Rover.brake = Rover.brake_set
    #                 Rover.steer = 0
    #                 Rover.mode = 'stop'

    #     # If we're already in "stop" mode then make different decisions
    #     elif Rover.mode == 'stop':
    #         # If we're in stop mode but still moving keep braking
    #         if Rover.vel > 0.2:
    #             Rover.throttle = 0
    #             Rover.brake = Rover.brake_set
    #             Rover.steer = 0
    #         # If we're not moving (vel < 0.2) then do something else
    #         elif Rover.vel <= 0.2:
    #             # Now we're stopped and we have vision data to see if there's a path forward
    #             if len(Rover.nav_angles) < Rover.go_forward:
    #                 Rover.throttle = 0
    #                 # Release the brake to allow turning
    #                 Rover.brake = 0
    #                 # Turn range is +/- 15 degrees, when stopped the next line will induce 4-wheel turning
    #                 Rover.steer = -15 # Could be more clever here about which way to turn
    #             # If we're stopped but see sufficient navigable terrain in front then go!
    #             if len(Rover.nav_angles) >= Rover.go_forward:
    #                 # Set throttle back to stored value
    #                 Rover.throttle = Rover.throttle_set
    #                 # Release the brake
    #                 Rover.brake = 0
    #                 # Set steer to mean angle
    #                 Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
    #                 Rover.mode = 'forward'
    # # Just to make the rover do something 
    # # even if no modifications have been made to the code
    # else:
    #     Rover.throttle = Rover.throttle_set
    #     Rover.steer = 0
    #     Rover.brake = 0
        
    # # If in a state where want to pickup a rock send pickup command
    # if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
    #     Rover.send_pickup = True
    
    return Rover

def get_right_dist(Rover):
    """
    Get the distance to the right wall, based on worldmap
    Find the appropriate line extending to the right of the robot, 
    then sequentially check each pixel moving outwards, until an obstacle wall
    appears, and return the distance.
    """
    x1, y1 = int(Rover.pos[0]), int(Rover.pos[1])
    thd = Rover.yaw - 90
    th = np.deg2rad(thd)
    height, width = Rover.worldmap.shape[0], Rover.worldmap.shape[1]
    
    # Initialize right_dist to -1, which indicates if no wall found
    right_dist = -1

    # checkmap = Rover.worldmap[:,:,0].astype(np.uint8)
    checkmap = np.invert(Rover.ground_truth[:,:,1].astype(np.uint8), dtype=np.uint8)
   
    print("Checkmap type:",checkmap.dtype)

    # If wall is upwards, traverse y
    if( (thd >= 45) & (thd <= 135)):
        print("     Upward wall")
        # make sure not directly upwards
        if(th == 90):
            x = x1
            for y in range(y1,0,-1): # Check pixels, going upwards
                if (checkmap[y,x] != 0): # Check if obstacle
                    right_dist = y1 - y
                    break
        # Traverse y, calculating each x
        m = np.tan(th) # Calc slope
        for y in range(y1,0,-1):
            x = np.int((y-y1)/m + x1)
            if( (x < 0) | (x >= width)): # Make sure x not out of bnounds
                break
            # Still in bounds, so check if wall
            if(checkmap[y,x] != 0):
                right_dist = np.sqrt( (x-x1)**2 + (y-y1)**2 )
                break       
    # If wall is downwards, traverse y
    elif( (thd >= 225) & (thd <= 315)):
        print("     Downward wall")
        # make sure not directly upwards
        if(th == 270):
            x = x1
            for y in range(y1,0,-1): # Check pixels, going upwards
                if (checkmap[y,x] != 0): # Check if obstacle
                    right_dist = y1 - y
                    break
        # Traverse y, calculating each x
        m = np.tan(th) # Calc slope
        for y in range(y1,height):
            x = np.int((y-y1)/m + x1)
            if( (x < 0) | (x >= width)): # Make sure x not out of bnounds
                break
            # Still in bounds, so check if wall
            if(checkmap[y,x] != 0):
                right_dist = np.sqrt( (x-x1)**2 + (y-y1)**2 )
                break       
    # If wall is leftwars, traverse x
    elif( (thd > 135) & (thd < 225)):
        print("     Leftward wall")
        m = np.tan(th) # Calc slope
        for x in range(x1,0,-1):
            y = np.int(y1 + m*(x-x1))
            if( (y < 0) | (y >= height)): # Make sure y not out of bnounds
                break
            else: # Still in bounds, so check if wall
                if(checkmap[y,x] != 0):
                    right_dist = np.sqrt( (x-x1)**2 + (y-y1)**2 )
                    break
    # Else, wall is rightwards, traveres x
    else:
        print("     Rightward wall")
        m = np.tan(th) # Calc slope
        for x in range(x1,width):
            y = np.int(y1 + m*(x-x1))
            if( (y < 0) | (y >= height)): # Make sure y not out of bnounds
                break
            else: # Still in bounds, so check if wall
                if(checkmap[y,x] != 0):
                    right_dist = np.sqrt( (x-x1)**2 + (y-y1)**2 )
                    break

    print("     Rover pose: (%d,%d)"%(x1,y1))
    print("     Theta(deg): %.3f"%thd)
    print("     Slope     : %.3f"%m)
    print("     Wall coord: (%d,%d)"%(x,y))
    print("     Right Dist: %.3f"%right_dist)
    
    return right_dist

def wall_crawl(Rover):

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        # Calculate distance to objects in front
        front_inds = (Rover.nav_angles < np.deg2rad(3)) & (Rover.nav_angles > -np.deg2rad(3)) # Front is +/- 5deg
        
        # if(np.sum(front_inds) < 10):
        #     front_dist = 0
        #     Rover.mode = 'stop'
        # else:
        #     print(np.sum(front_inds))
        #     # print(len(front_inds))
        #     # print(len(Rover.nav_dists))
        #     front_dist = np.mean(Rover.nav_dists[front_inds])


        ## HACK
        if(np.sum(front_inds) < 10):
            front_dist = np.min(Rover.nav_dists[front_inds])
        else:
            front_dist = -1


        min_front_dist = 2.0
        reverse_dist = 0.7


        des_right_dist = 4.0
        ksteer = 2.0    
        kv = 5.0/15  # Max vel (2.0 m/s) when front_dist = 15m
        kt = 0.2/0.5   # Max accel (0.2) when Vel diff = 2.0 m/s   

        print("\n\n")
        print("  Mode:",Rover.mode)
        print("  Front Dist:",front_dist)

        if(Rover.mode == 'forward'):

            if(front_dist < min_front_dist):
                Rover.mode = "stop"
            else:
                # Calc dist to right wall using global map
                right_dist = get_right_dist(Rover)
                # print(" x,y = (%d,%d)"%(Rover.pos[0], Rover.pos[1]))
                # print("  Right Dist:",right_dist)
                # Check if dist is invalid
                if(right_dist == -1):
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    print("   mean_steer")
                else:
                    Rover.steer = np.clip(ksteer*(right_dist - des_right_dist), -15, 15)
                    print("   diff steer: %.3f"%Rover.steer)
                Rover.des_vel = np.min([Rover.max_vel, kv*(front_dist-min_front_dist)])
                Rover.throttle = kt * (Rover.des_vel - Rover.vel)

                Rover.brake = 0



        elif(Rover.mode == 'stop'):
            # If moving, brake to a stop
            if(Rover.vel > 0.2):
                Rover.setTSB(0,0,Rover.brake_set)
                print("   Braking")
            # Stopped
            else:
                # # Too close to the wall, so reverse a little
                # if(front_dist < reverse_dist):
                #     Rover.setTSB(-0.2, 0, 0)
                #     print("   Reversing")

                # Stopped, and not tooc lose to wall
                # else:
                
                # If not enough space in front, turn left in place
                if((front_dist <  min_front_dist) | (np.sum(front_inds) < 10)):
                    Rover.setTSB(0,15,0)
                    print("   Rotating")
                # There is space, so continue crawling
                else:
                    print(" Go forward again")
                    Rover.mode = 'forward'
                    print("  should be forward:",Rover.mode)


        print("  Rov Comms TSB:",Rover.throttle, Rover.steer, Rover.brake)


    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0




    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True
    
    return Rover
