import math

L1 = 4
L2 = 3.75
L3_full = 6.5
L3_mid = 5.5

j5_new = 0
j4_new = 0
j3_new = 0

def pos_is_legal(j5, j4, j3):
    if((j5>=125 and  j5<=875) and (j4>=0 and j4<=1000) and (j3>=0 and j3<=1000)):
        return True
    else:
        return False

def verify_pos(t1, t2, t3, x, y, L3):
    xee = L1*math.cos(math.radians(t1)) + L2*math.cos(math.radians(t1 + t2)) + L3*math.cos(math.radians(t1 + t2 + t3))
    yee = L1*math.sin(math.radians(t1)) + L2*math.sin(math.radians(t1 + t2)) + L3*math.sin(math.radians(t1 + t2 + t3))

    if abs(xee-x) < .1 or abs(yee-y) < .1:
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
        j5 = input('Enter Servo 5, Shoulder(125-875): ')
        if j5 == '':
            j5 = j5_new
            j4 = j4_new
            j3 = j3_new
        elif j5 == 'q':
            break
        else:
            try:
                j5 = float(j5)
                j4 = float(input('Enter Servo 4, Elbow 1 (0-1000): '))
                j3 = float(input('Enter Servo 3, Elbow 2 (0-1000): '))
            except:
                continue

        if pos_is_legal(j5, j4, j3) == False:
            print('Illegal Position\n')
            continue

        theta1 = (j5-125)*degrees_per_ticks
        theta2 = (500-j4)*degrees_per_ticks
        theta3 = (j3-500)*degrees_per_ticks
        phi = theta1+theta2+theta3

        #print('\nTheta1: '+str(theta1))
        #print('Theta2: '+str(theta2))
        #print('Theta3: '+str(theta3))
        print('Phi: '+str(phi), end='\n\n')

        xee = L1*math.cos(math.radians(theta1)) + L2*math.cos(math.radians(theta1 + theta2)) + L3*math.cos(math.radians(theta1 + theta2 + theta3))
        yee = L1*math.sin(math.radians(theta1)) + L2*math.sin(math.radians(theta1 + theta2)) + L3*math.sin(math.radians(theta1 + theta2 + theta3))

        print('Xee = ' + str(xee))
        print('Yee = ' + str(yee), end='\n\n')

        dx = input('Enter delta x: ')
        if dx == '':
            continue
        else:
            dx = float(dx)
        
        dy = float(input('Enter delta y: '))

        x_new = xee + dx
        y_new = yee + dy

        phi_new = 0
        while phi_new < 360:
            try:
                theta_2_new = math.degrees(math.acos(((x_new - L3*math.cos(math.radians(phi_new)))**2 + (y_new - L3*math.sin(math.radians(phi_new)))**2 - L1**2 - L2**2) / (2*L1*L2)))
                theta_1_new = math.degrees(math.acos(((L1 + L2*math.cos(math.radians(theta_2_new)))*(x_new - L3*math.cos(math.radians(phi_new))) + (L2*math.sin(math.radians(theta_2_new)))*(y_new - L3*math.sin(math.radians(phi_new)))) / ((x_new - L3*math.cos(math.radians(phi_new)))**2 + (y_new - L3*math.sin(math.radians(phi_new)))**2)))
                theta_3_new = phi_new - (theta_1_new + theta_2_new)
            except:
                phi_new += 1
                print('Next Test: '+str(phi_new))
                continue

            #print('\nNew Theta1: '+str(theta_1_new))
            #print('New Theta2: '+str(theta_2_new))
            #print('New Theta3: '+str(theta_3_new))

            j5_new = (theta_1_new/degrees_per_ticks) + 125
            j4_new = 500 - (theta_2_new/degrees_per_ticks)
            j3_new = (theta_3_new/degrees_per_ticks) + 500

            if pos_is_legal(j5_new, j4_new, j3_new) and verify_pos(theta_1_new, theta_2_new, theta_3_new, x_new, y_new, L3):
                print('\nNew Servo 5 (Shoulder): '+str(int(j5_new)))
                print('New Servo 4 (Elbow 1): '+str(int(j4_new)))
                print('New Servo 3 (Elbow 2): '+str(int(j3_new)), end='\n\n')
                break
            phi_new += 1

        if phi_new == 360:
            print('EE Position Not Possible\n\n')

    
