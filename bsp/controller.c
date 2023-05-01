// #include <stdio.h>
#include <stdlib.h>

#include "controller.h"
#include "servo.h"
#include "board.h"

int flag;
void turn_ctrlr(int dirn, float ang_des, float ang_curr){ // Only turning
    /*
    servo_out(int traj, int dirn, int speed );
    traj=1 => straight; dirn=1 => frwd; dirn=-1 => bwd;
    traj=0 => circle; dirn=1 => CCW; dirn=-1 => CW;
    dirn=0 => stop
    speed levels limited to 6
    */ 
    // printf("\n[CONTROLLER] TURN CONTROLLER");
    float tolr = 15.0;
    // float ang_curr = computeHeading();
    // print_float_ctrl(abs(ang_des - ang_curr),4);
    
    if((abs(ang_des - ang_curr) >= tolr) && flag==1){
    // if((abs(ang_des - ang_curr) >= tolr) ){
        servo_out(0,dirn,3);
        led_off(2,2);
    }
    else{
        // printf("stopping");
        servo_out(0,0,3);
        flag=0;
        led_on(2,2);
    }
}

void move_ctrlr(int dirn, float orient, float val){ 
    // printf("\n[CONTROLLER] MOVE CONTROLLER");
    float err;
    // float val = computeHeading();
    float tolr = 20.0;
    flag = 1;
    if(dirn == 0)
        servo_out(1,0,3);
    else{
        err = val - orient ; 
        if(abs(err)<= tolr){
            servo_out(1,dirn,3);
        }
        else{
            // if(abs(err) > 200){
            //     if(err > 0){   
            //         turn_ctrlr(1, orient,val); //oriented left, so command right
            //         // servo_out(0,1,3);
            //         printf("correcting to left");
            //     }
            //     else if(err < 0){
            //         turn_ctrlr(-1, orient,val);
            //         // servo_out(0,-1,3);
            //         printf("correcting to ryt");
            //     }   
            // }
            // else{
                if( err > 0){   
                    turn_ctrlr(-1, orient,val); //oriented left, so command right
                    // servo_out(0,-1,3);
                    // printf("correcting to ryt");
                }
                else if(err < 0){
                    turn_ctrlr(1, orient,val);
                    // servo_out(0,1,3);
                    // printf("correcting to left");
                }
            // }
        }
    }
    // with pins facing forward(180deg), left-ang increases, 
    // Assuming "if" for smooth transition than "while"
    
}

// void move_ctrlr()


// if(dirn != 0){
//         if(abs(val - orient) > 200){
//             if((val - orient) >= tolr){   
//                 // turn_ctrlr(1, orient,val); //oriented left, so command right
//                 servo_out(0,1,3);
//                 printf("correcting to left");
//             }
//             else if((orient - val) >= tolr){
//                 // turn_ctrlr(-1, orient,val);
//                 servo_out(0,-1,3);
//                 printf("correcting to ryt");
//             }
//         }
//         else{
//             if((val - orient) >= tolr){   
//                 // turn_ctrlr(-1, orient,val); //oriented left, so command right
//                 servo_out(0,-1,3);
//                 printf("correcting to ryt");
//             }
//             else if((orient - val) >= tolr){
//                 // turn_ctrlr(1, orient,val);
//                 servo_out(0,1,3);
//                 printf("correcting to left");
//             }
//         }
        
//     }


void print_float_ctrl(float data, int dec_places){

    int64_t k = 1;
    for (int i = 0; i < dec_places; i++)
        k *= 10;
    int num, dec;
      num = (int) (data);
      dec = (int) ((data-num)*k); 
      dec = dec > 0 ? dec : -dec;
      printf("%d.%d ",num,dec);
}