import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        # des_right_dist = 4.0
        ksteerPos = -10.0 # Turning right
        ksteerNeg = -30.0  # Turning left
        kRockSteer = 20.0
        kv = 5.0/15  # Max vel (2.0 m/s) when front_dist = 15m
        kt = 0.25/0.5   # Max accel (0.2) when Vel diff = 2.0 m/s   

        print("-----------------------------")
        print("Time: %.1f"%Rover.total_time)
        print("  FPS: %.1f"%Rover.fps)
        print("  Mode:",Rover.mode)
        print("  Front Dist: %.2f"%Rover.front_dist)
        print("  Right Dist: %.2f"%Rover.right_dist)
        print("  Len of angs: ", len(Rover.nav_angles))

        if(Rover.mode == 'forward'):
            if(Rover.front_dist < Rover.min_front_dist):
                Rover.mode = "stop"
                print("     Forward obs too close")
            elif(len(Rover.nav_angles) < Rover.stop_forward):
                Rover.mode = "stop"
                print("     Not enough forward space")
            else:
                # Check if dist is invalid
                if(Rover.right_dist == -1):
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    print("   mean_steer")
                else:
                    # Under usual conditions, stay des_right_dist from wall (ex 1.5m)
                    # But for narrow passages (when 1.5m from right wall is too far, and might run into left wall)
                    # aim for middle of passage (average of left and right distances)
                    des_dist = np.min([Rover.des_right_dist, (Rover.right_dist+Rover.left_dist)/2.0]) 
                    steer_dif = (Rover.right_dist - des_dist) 

                    if(steer_dif > 0):
                        Rover.steer = np.clip(ksteerPos*steer_dif, -15, 10)
                    else:
                        Rover.steer = np.clip(ksteerNeg*steer_dif, -15, 10)
                    print("   diff steer: %.3f"%Rover.steer)
                Rover.des_vel = np.clip(kv*(Rover.front_dist-Rover.min_front_dist), Rover.min_forward_vel, Rover.max_vel)
                Rover.throttle = np.clip(kt * (Rover.des_vel - Rover.vel),-Rover.max_acc, Rover.max_acc)
                print("   Des Vel: %.2f"%Rover.des_vel)
                print("   Cur Vel: %.2f"%Rover.vel)
                print("   Cur Thr: %.2f"%Rover.throttle)

                # Go towards rocks (any within 4 meters)
                if(len(Rover.rock_dists) > 0):
                    if(np.mean(Rover.rock_dists < 40)):
                            Rover.mode = 'rock'
                            print("Going to get a rock")

                Rover.brake = 0

        elif(Rover.mode == 'rock'):
            # don't see any rocks
            if(len(Rover.rock_dists) == 0):
                Rover.mode = 'stop'
            else:
                rock_ang = np.mean(Rover.rock_angles)
                Rover.steer = np.clip(kRockSteer*rock_ang, -15, 15)
                print("   Rock steer: %.3f"%Rover.steer)

                rock_dist = np.mean(Rover.rock_dists)
                Rover.des_vel = np.clip(0.6*kv*(rock_dist), Rover.min_forward_vel, 0.6*Rover.max_vel)
                Rover.throttle = np.clip(kt * (Rover.des_vel - Rover.vel),-Rover.max_acc, Rover.max_acc)
                print("   Des Vel: %.2f"%Rover.des_vel)
                print("   Cur Vel: %.2f"%Rover.vel)
                print("   Cur Thr: %.2f"%Rover.throttle)

            Rover.brake = 0

        elif(Rover.mode == 'stop'):
            # If moving, brake to a stop
            if(Rover.vel > 0.2):
                Rover.setTSB(0,0,Rover.brake_set)
            # Stopped
            else:               
                # If not enough space in front, turn left in place
                if(Rover.front_dist <  Rover.min_front_dist):
                    Rover.setTSB(0,10,0)
                    print(" Front Dist Too close")
                elif(len(Rover.nav_angles) < Rover.go_forward):
                    Rover.setTSB(0,10,0)
                    print(" Not enough front room")
                # There is space, so continue crawling
                else:
                    Rover.mode = 'forward'


        elif(Rover.mode == 'stuck'):
            # # Turn 45 deg in place
            # if( Rover.yaw-Rover.first_stuck_angle < np.deg2rad(45)):
            

            stuck_time = Rover.total_time - Rover.first_stuck_time
            print("     FirstStuckTime: %.1f"%Rover.first_stuck_time)
            print("     StuckTimeMod: %.1f"%(stuck_time%4.0))

            # Alternate between turning, and trying to go forwards
            if (stuck_time%4.0 < 2.0):
                Rover.setTSB(0,15,0) # Try turn leftwards in place
            else:
                Rover.setTSB(2,0,0)



            # If able to move now, consider unstuck
            if( Rover.vel > 0.2):
                Rover.mode = 'stop'
                # Rover.just_got_stuck = False




        # Check if stuck (must be stuck for significant time)
        if((Rover.throttle > 0.09) & (abs(Rover.vel) < 0.09) & (Rover.mode != 'stuck')):
            if(not Rover.just_got_stuck):
                Rover.just_got_stuck = True
                Rover.first_stuck_time = Rover.total_time
                Rover.first_stuck_angle = Rover.yaw
            # Check if stuck for long time
            if(Rover.total_time - Rover.first_stuck_time > Rover.time_stuck):
                Rover.mode = 'stuck'
        else:
            Rover.just_got_stuck = False


        # Stop when close to rock
        if(Rover.near_sample and not Rover.picking_up):
            Rover.setTSB(0,0,Rover.brake_set)

        print("  Rov Comms TSB: %.2f, %.2f, %.2f"%(Rover.throttle, Rover.steer, Rover.brake))


    # Just to make the rover do something 
    # even if no modifications have been made to the code
    else:
        Rover.throttle = Rover.throttle_set
        Rover.steer = 0
        Rover.brake = 0


    # If in a state where want to pickup a rock send pickup command
    if Rover.near_sample and Rover.vel == 0 and not Rover.picking_up:
        Rover.send_pickup = True


    print("")

  
    return Rover
