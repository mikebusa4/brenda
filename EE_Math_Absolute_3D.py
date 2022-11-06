import math

L1 = 4
L2 = 3.75
L3_full = 6.5
L3_mid = 5.5

j5_new = 0
j4_new = 0
j3_new = 0

def pos_is_legal(j6, j5, j4, j3):
    if((j6>=0 and j6<=1000) and (j5>=125 and  j5<=875) and (j4>=0 and j4<=1000) and (j3>=65 and j3<=1000)):
        return True
    else:
        return False

def verify_pos(t1, t2, t3, x, y, L3):
    xee = L1*math.cos(math.radians(t1)) + L2*math.cos(math.radians(t1 + t2)) + L3*math.cos(math.radians(t1 + t2 + t3))
    yee = L1*math.sin(math.radians(t1)) + L2*math.sin(math.radians(t1 + t2)) + L3*math.sin(math.radians(t1 + t2 + t3))

    if abs(xee-x) < .01 or abs(yee-y) < .01:
        return True
    else:
        return False
    

degrees_per_ticks = .24

while True:
    which_L3 = input('\n----------------------------------\nEnter 1 for end-of-gripper EE or 2 for middle-of-gripper EE: ')
    if which_L3 == '1':
        L3 = L3_full
    elif which_L3 == '2':
        L3 = L3_mid
    else:
        continue

    while True:
        try:
            x_new = input('Enter x: ')
            if x_new == '':
                continue
            else:
                x_new = float(x_new)
        
            y_new = float(input('Enter y: '))
            z_new = float(input('Enter z: '))
        except:
            continue

        gamma = math.degrees(math.atan(z_new / x_new))

        if x_new<0 and z_new<0:
            gamma = -180 + gamma
        elif x_new<0:
            gamma = 180 + gamma
        
        j6_new = 500 - (gamma/degrees_per_ticks)

        d_new = math.sqrt(x_new*x_new + z_new*z_new)
        if x_new < 0:
            d_new*=-1
        
        phi_new = 0
        while phi_new < 360:
            try:
                theta_2_new = math.degrees(math.acos(((d_new - L3*math.cos(math.radians(phi_new)))**2 + (y_new - L3*math.sin(math.radians(phi_new)))**2 - L1**2 - L2**2) / (2*L1*L2)))
                theta_1_new = math.degrees(math.acos(((L1 + L2*math.cos(math.radians(theta_2_new)))*(d_new - L3*math.cos(math.radians(phi_new))) + (L2*math.sin(math.radians(theta_2_new)))*(y_new - L3*math.sin(math.radians(phi_new)))) / ((d_new - L3*math.cos(math.radians(phi_new)))**2 + (y_new - L3*math.sin(math.radians(phi_new)))**2)))
                theta_3_new = phi_new - (theta_1_new + theta_2_new)
            except:
                phi_new += 0.1
                continue

            j5_new = ((theta_1_new)/degrees_per_ticks) + 125
            j4_new = 500 - (theta_2_new/degrees_per_ticks)
            j3_new = ((theta_3_new/degrees_per_ticks) + 500)%1500

            if pos_is_legal(j6_new, j5_new, j4_new, j3_new) and verify_pos(theta_1_new, theta_2_new, theta_3_new, d_new, y_new, L3):
                #print('\nNew Theta1: '+str(theta_1_new))
                #print('New Theta2: '+str(theta_2_new))
                #print('New Theta3: '+str(theta_3_new))
                
                print('\nPhi: '+ str(phi_new))
                print('Gamma: '+ str(gamma))
                print('New Servo 6 (Turret): ' +str(int(j6_new)))
                print('New Servo 5 (Shoulder): '+str(int(j5_new)))
                print('New Servo 4 (Elbow 1): '+str(int(j4_new)))
                print('New Servo 3 (Elbow 2): '+str(int(j3_new)), end='\n\n')
                break

            else:
                theta_2_new *= -1
                theta_1_new = math.degrees(math.acos(((L1 + L2*math.cos(math.radians(theta_2_new)))*(d_new - L3*math.cos(math.radians(phi_new))) + (L2*math.sin(math.radians(theta_2_new)))*(y_new - L3*math.sin(math.radians(phi_new)))) / ((d_new - L3*math.cos(math.radians(phi_new)))**2 + (y_new - L3*math.sin(math.radians(phi_new)))**2)))
                theta_3_new = phi_new - theta_2_new - theta_1_new

                j5_new = ((theta_1_new)/degrees_per_ticks) + 125
                j4_new = 500 - (theta_2_new/degrees_per_ticks)
                j3_new = ((theta_3_new/degrees_per_ticks) + 500)%1500

                if pos_is_legal(j6_new, j5_new, j4_new, j3_new) and verify_pos(theta_1_new, theta_2_new, theta_3_new, d_new, y_new, L3):
                    #print('\nNew Theta1: '+str(theta_1_new))
                    #print('New Theta2: '+str(theta_2_new))
                    #print('New Theta3: '+str(theta_3_new))

                    print('\nPhi: '+ str(phi_new))
                    print('Gamma: '+ str(gamma))
                    print('New Servo 6 (Turret): ' +str(int(j6_new)))
                    print('New Servo 5 (Shoulder): '+str(int(j5_new)))
                    print('New Servo 4 (Elbow 1): '+str(int(j4_new)))
                    print('New Servo 3 (Elbow 2): '+str(int(j3_new)), end='\n\n')
                    break
                    

            phi_new += 0.1
        if phi_new >= 360:
            print('EE Position not Possible', end='\n\n')
    
