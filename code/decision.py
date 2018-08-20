import numpy as np

# This is where you can build a decision tree for determining throttle, brake and steer 
# commands based on the output of the perception_step() function
def decision_step(Rover):

    # Check if we have vision data to make decisions with
    if Rover.nav_angles is not None:

        # des_right_dist = 4.0
        ksteerPos = -4.0 # Turning right
        ksteerNeg = -30.0  # Turning left
        kv = 5.0/15  # Max vel (2.0 m/s) when front_dist = 15m
        kt = 0.25/0.5   # Max accel (0.2) when Vel diff = 2.0 m/s   

        print("\n\n")
        print("  Mode:",Rover.mode)
        print("  Front Dist:",Rover.front_dist)
        print("  Len of angs: ", len(Rover.nav_angles))

        if(Rover.mode == 'forward'):
            if(Rover.front_dist < Rover.min_front_dist):
                Rover.mode = "stop"
                print("Forward obs too close")
            elif(len(Rover.nav_angles) < Rover.stop_forward):
                Rover.mode = "stop"
                print("Not enough forward space")
            else:
                # Check if dist is invalid
                if(Rover.right_dist == -1):
                    Rover.steer = np.clip(np.mean(Rover.nav_angles * 180/np.pi), -15, 15)
                    print("   mean_steer")
                else:
                    steer_dif = (Rover.right_dist - Rover.des_right_dist)
                    if(steer_dif > 0):
                        Rover.steer = np.clip(ksteerPos*steer_dif, -12, 12)
                    else:
                        Rover.steer = np.clip(ksteerNeg*steer_dif, -12, 12)
                    print("   diff steer: %.3f"%Rover.steer)
                Rover.des_vel = np.min([Rover.max_vel, kv*(Rover.front_dist-Rover.min_front_dist)])
                Rover.des_vel = np.max([Rover.des_vel, Rover.min_forward_vel])
                Rover.throttle = np.clip(kt * (Rover.des_vel - Rover.vel),0, Rover.max_acc)
                print("   Des Vel: %.2f"%Rover.des_vel)
                print("   Cur Vel: %.2f"%Rover.vel)
                print("   Cur Thr: %.2f"%Rover.throttle)

                # Check if stuck
                if((Rover.throttle > 0.09) & (abs(Rover.vel) < 0.09)):
                    if(not Rover.just_got_stuck):
                        Rover.just_got_stuck = True
                        Rover.first_stuck_time = Rover.total_time
                    # Check if stuck for long time
                    if(Rover.total_time - Rover.first_stuck_time > Rover.time_stuck):
                        Rover.mode = 'reverse'
                        print("#####")
                        print("STUCK")
                        print("#####")
                else:
                    Rover.just_got_stuck = False


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

        elif(Rover.mode == 'reverse'):
            # Not quite reversing yet
            if(Rover.vel > -0.3):
                Rover.setTSB(-1,-10,0)
                print("REVERSING")
                # Stuck going backwards
                if(Rover.total_time - Rover.first_stuck_time > 2*Rover.time_stuck):
                    Rover.mode = 'stop'
            # Already built negative velocity, so switch modes
            else:
                Rover.mode = 'stop'

        if(Rover.near_sample and not Rover.picking_up):
            Rover.setTSB(0,0,Rover.brake_set)

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
