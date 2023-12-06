
/**
 * V. Hunter Adams (vha3@cornell.edu)
 * 
 * This demonstration utilizes the MPU6050.
 * It gathers raw accelerometer/gyro measurements, scales
 * them, and plots them to the VGA display. The top plot
 * shows gyro measurements, bottom plot shows accelerometer
 * measurements.
 * 
 * HARDWARE CONNECTIONSpo0
 *  - GPIO 16 ---> VGA Hsync
 *  - GPIO 17 ---> VGA Vsync
 *  - GPIO 18 ---> 330 ohm resistor ---> VGA Red
 *  - GPIO 19 ---> 330 ohm resistor ---> VGA Green
 *  - GPIO 20 ---> 330 ohm resistor ---> VGA Blue
 *  - RP2040 GND ---> VGA GND
 *  - GPIO 8 ---> MPU6050 SDA
 *  - GPIO 9 ---> MPU6050 SCL
 *  - 3.3v ---> MPU6050 VCC
 *  - RP2040 GND ---> MPU6050 GND
 *
 * SERIAL CONNECTIONS
 *  - GPIO 0        -->     UART RX (white)
 *  - GPIO 1        -->     UART TX (green)
 *  - RP2040 GND    -->     UART GND
 */


// Include standard libraries
#include <stdio.h>
#include <stdlib.h>
#include <math.h>
#include <string.h>
// Include PICO libraries
#include "pico/stdlib.h"
#include "pico/multicore.h"
#include "pico/divider.h"
// Include hardware libraries
#include "hardware/pwm.h"
#include "hardware/dma.h"
#include "hardware/irq.h"
#include "hardware/adc.h"
#include "hardware/pio.h"
#include "hardware/i2c.h"
#include "hardware/clocks.h"
#include "hardware/structs/clocks.h"
// Include custom libraries
#include "vga_graphics.h"
#include "mpu6050.h"
#include "pt_cornell_rp2040_v1.h"

// === the fixed point macros ========================================
#define sqrtfix(a)  float2fix15(sqrt(fix2float15(a)))

// Some macros for max/min/abs
#define min(a,b) ((a<b) ? a:b)
#define max(a,b) ((a<b) ? b:a)
#define abs(a) ((a>0) ? a:-a)
#define l2distance(x1,y1,x2,y2) sqrt((x1-x2)*(x1-x2)+(y1-y2)*(y1-y2))
// semaphore
static struct pt_sem vga_semaphore ;

// draw speed
int threshold = 10 ;

// character array
char screentext[40];

// Some paramters for PWM (50Hz)
#define WRAPVAL 50000
#define CLKDIV  50.0
#define PI 3.14159265359
#define CYCLELIMIT 133
#define THRESHOLD 1950

#define x1 -20
#define y1 0
#define x2 0
#define y2 40
#define x3 20
#define y3 0

//variables for motor control 
uint slice_num ;
volatile int old_controlTilt=0;
volatile int old_control_turn=0;
volatile int controlTurn=1000;
volatile int controlTilt=1000;
volatile int motor_dispTurn=0;
volatile int motor_dispTilt=0;
volatile uint16_t microphone_reading[3]; 
volatile uint16_t max_reading[3]; 
volatile bool detected[3]={false,false,false}; 
volatile long cycle_detected[3]={0,0,0};
volatile int cycle_after[3]={0,0,0};
volatile int cycle=0;
volatile int first_cycle=0;
volatile int pwm_turn=0;
volatile int num_detected=0;
volatile int magnitude=100;
volatile float angle= 0;
volatile float lcos_value= 0;
fix15 cmpercycle = float2fix15(.75);
fix15 negtwo = int2fix15(-2);
const float conversion_factor = 3.3f / (1 << 12);
volatile int sweep =0;
volatile int oldAngle=0;

void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    if(old_control_turn!=controlTurn){
        old_control_turn=controlTurn;
    }
    if(old_controlTilt!=controlTilt){
        old_controlTilt=controlTilt;
    }
        // motor_dispTilt = motor_dispTilt + ((controlTilt - motor_dispTilt)>>4) ;
        // motor_dispTurn = motor_dispTurn + ((controlTurn - motor_dispTurn)>>4) ;
 
    // logic to control the pwm from the microphone
              
            int first_value=min(cycle_after[0],min(cycle_after[1],cycle_after[2]));
            int first_one=0;
            int second_one=0;
            int second_value=0;
            if(first_value==cycle_after[0]){
                second_value=min(cycle_after[1],cycle_after[2]);
            }
            else if(first_value==cycle_after[1]){
                second_value=min(cycle_after[0],cycle_after[2]);
                first_one=1;
            }
            else {
                second_value=min(cycle_after[0],cycle_after[1]);
                first_one=2;
            }
            if(second_value==cycle_after[2]){
                second_one=2;
            }
            else if(second_value==cycle_after[1]){
                
                second_one=1;
            }
           
            int delay= first_value-second_value;
            lcos_value=delay;
            angle = asin((delay)*3 / (801.0) );

            if(first_value==0 &&second_value==1){
                angle=PI/3 -angle;
            }
            if(second_value==0 &&first_value==1){
                angle=PI/3 +angle;
            }
            if(first_value==1 &&second_value==2){
                angle=2*PI/3+angle;
            }
            if(second_value==1 &&first_value==2){
                angle=2*PI/3-angle;
            }
            if(first_value==0 &&second_value==2){
                angle=-PI/3-angle;
            }
            if(second_value==0 &&first_value==2){
                angle=-PI/3 +angle;
             }
            
    //20,000 - -20,000
    int turn=angle*3000 ;
    if(angle!=oldAngle){
        if(angle<0){
            sweep=-1;
        }
        if (angle>0){
            sweep=1;
        }
        oldAngle=angle;
    }
    turn+=3000;
    pwm_turn=turn;
    if (sweep==-1){
        controlTurn=2000;
        sweep--;
    }
    else if(sweep ==-2){
        controlTurn=6500;
        sweep--;
    }
    else if(sweep==-3){
        controlTurn=4000;
        sweep=0;
    }
    if(sweep==1){
        controlTurn=12000;
        sweep++;
    }
    else if(sweep ==2){
        controlTurn=6500;
        sweep++;
    }
    else if(sweep==3){
        controlTurn=12000;
        sweep=0;
    }
    pwm_set_chan_level(slice_num, PWM_CHAN_B, controlTurn);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, controlTilt );
    


    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// polling 
static PT_THREAD (protothread_gpio(struct pt *pt)) {
    PT_BEGIN(pt) ;
    
    while(true){
        bool gp26=false;
        bool gp27=false;
        bool gp28=false;
        cycle=0;
        adc_set_round_robin	(7)	;
        while(!(gp26||gp27||gp28)){
            
            microphone_reading[0]=adc_read();
            
            microphone_reading[1]=adc_read();
            microphone_reading[2]=adc_read();
            gp26= microphone_reading[0]>THRESHOLD;
            gp27=microphone_reading[1] >THRESHOLD;
            gp28=microphone_reading[2] >THRESHOLD;
            cycle++;
        }
        first_cycle=cycle;
        int last_cycle=first_cycle+CYCLELIMIT;
        gpio_put(25, 0);
        
        if(gp26&&gp27&&gp28){
            cycle_detected[0]=cycle;
            cycle_detected[1]=cycle;
            cycle_detected[2]=cycle;
            gpio_put(15, !(gpio_get(15)));
            gpio_put(14, !(gpio_get(14)));
            gpio_put(13, !(gpio_get(13)));
        }
        else if(gp26&&gp27){
            cycle_detected[0]=cycle;
            cycle_detected[1]=cycle;
            gpio_put(15, !(gpio_get(15)));
            gpio_put(14, !(gpio_get(14)));
            adc_set_round_robin	(4)	;
            while(!gp28&&last_cycle>=cycle){
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
                if(cycle_detected[2]<last_cycle){
                    gpio_put(13, !(gpio_get(13)));
                }
        }
        else if(gp26&&gp28){
            cycle_detected[0]=cycle;
            cycle_detected[2]=cycle;
            gpio_put(15, !(gpio_get(15)));
            gpio_put(13, !(gpio_get(13)));
            adc_set_round_robin	(2)	;
            while(!gp27&&last_cycle>=cycle){
                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
                if(cycle_detected[1]<last_cycle){
                    gpio_put(14, !(gpio_get(14)));
                }
        }
        else if(gp27&&gp28){
            cycle_detected[1]=cycle;
            cycle_detected[2]=cycle;
            gpio_put(13, !(gpio_get(13)));
            gpio_put(14, !(gpio_get(14)));
            adc_set_round_robin	(1)	;
            while(!gp26&&last_cycle>=cycle){
                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
                if(cycle_detected[0]<last_cycle){
                    gpio_put(15, !(gpio_get(15)));
                }
        }
        else if(gp26){
            cycle_detected[0]=cycle;
            gpio_put(15, !(gpio_get(15)));
            adc_set_round_robin	(6)	;
            while(!(gp27||gp28)&&last_cycle>=cycle){
                gp27=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
            if((gp27&&gp28)||last_cycle<cycle){
                cycle_detected[1]=cycle;
                cycle_detected[2]=cycle;
                 if(cycle_detected[1]<last_cycle){
                    gpio_put(14, !(gpio_get(14)));
                    gpio_put(13, !(gpio_get(13)));
                }
            }
            else if(gp27){
                cycle_detected[1]=cycle;
                gpio_put(14, !(gpio_get(14)));
                adc_set_round_robin	(4)	;
                while(!gp28&&last_cycle>=cycle){
                    
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
                if(cycle_detected[2]<last_cycle){
                    gpio_put(13, !(gpio_get(13)));
                }
            }
            else{
                cycle_detected[2]=cycle;
                gpio_put(13, !(gpio_get(13)));
                adc_set_round_robin	(2)	;
                while(!gp27&&last_cycle>=cycle){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
                if(cycle_detected[1]<last_cycle){
                    gpio_put(14, !(gpio_get(14)));
                }
                
            }
            
        }
        else if(gp27){
            cycle_detected[1]=cycle;
            gpio_put(14, !(gpio_get(14)));
            adc_set_round_robin	(5)	;
            while(!(gp26||gp28)&&last_cycle>=cycle){
                gp26=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
             if((gp26&&gp28)||last_cycle<cycle){
                cycle_detected[0]=cycle;
                cycle_detected[2]=cycle;
              if(cycle_detected[2]<last_cycle){
                    gpio_put(13, !(gpio_get(13)));
                    
                    gpio_put(15, !(gpio_get(15)));
                
                }
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                
                    gpio_put(15, !(gpio_get(15)));
                
                adc_set_round_robin	(4)	;
                while(!gp28&&last_cycle>=cycle){

                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
                if(cycle_detected[2]<last_cycle){
                    gpio_put(13, !(gpio_get(13)));
                }
            }
            else{
                cycle_detected[2]=cycle;
                gpio_put(13, !(gpio_get(13)));
                adc_set_round_robin	(1)	;
                while(!gp26&&last_cycle>=cycle){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
                 if(cycle_detected[0]<last_cycle){
                    gpio_put(15, !(gpio_get(15)));
                }
            }
        }
        else{
            cycle_detected[2]=cycle;
            gpio_put(13, !(gpio_get(13)));
            adc_set_round_robin	(3)	;
            while(!(gp26||gp27)&&last_cycle>=cycle){
                gp26=adc_read() >THRESHOLD;
                gp27=adc_read() >THRESHOLD;
                cycle++;
            }
            if((gp26&&gp27)||last_cycle<cycle){
                cycle_detected[0]=cycle;
                cycle_detected[1]=cycle;
                if(cycle_detected[0]<last_cycle){
                    gpio_put(15, !(gpio_get(15)));
                    gpio_put(14, !(gpio_get(14)));
                }
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                gpio_put(15, !(gpio_get(15)));
                adc_set_round_robin	(2)	;
                while(!gp27&&last_cycle>=cycle){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
                if(cycle_detected[1]<last_cycle){
                    gpio_put(14, !(gpio_get(14)));
                }
            }
            else{
                cycle_detected[1]=cycle;
                gpio_put(14, !(gpio_get(14)));
                adc_set_round_robin	(1)	;
                while(!gp26&&last_cycle>=cycle){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
                if(cycle_detected[0]<last_cycle){
                    gpio_put(15, !(gpio_get(15)));
                }
            }
        }
        gpio_put(25, 1);

            int min_value=cycle_detected[0];
            num_detected=0;                                                                                                      
            for(int i=0;i<3;i++){
                detected[i]=false;
                if(min_value>cycle_detected[i]){
                    min_value=cycle_detected[i];
                }
            }
            for(int i=0;i<3;i++){
                cycle_after[i]=cycle_detected[i]-min_value;
            }

        //}
        
        
    }
    PT_END(pt);
}

static PT_THREAD (protothread_vga(struct pt *pt))
{
    // Indicate start of thread
    PT_BEGIN(pt) ;

    // We will start drawing at column 81
    static int xcoord = 81 ;
    
    // Rescale the measurements for display
    static float OldRange = 500. ; // (+/- 250)
    static float NewRange = 150. ; // (looks nice on VGA)
    static float OldMin = -250. ;
    static float OldMax = 250. ;

    // Control rate of drawing
    static int throttle ;

    // Draw the static aspects of the display
    
    setTextColor(WHITE);
    setCursor(210, 50) ;
    setTextSize(3);
    sprintf(screentext, "Soundar Tracking") ;
    writeString(screentext) ;
    // Draw bottom plot


   // Draw top plot
    drawCircle(350,250,150,GREEN);
    drawVLine(350, 100, 300, GREEN) ;
    drawHLine(200, 250, 300, GREEN) ;
    fillRect(200,150,150, 300, BLACK) ;
    while (true) {
        // Wait on semaphore
        PT_SEM_WAIT(pt, &vga_semaphore);
        // Increment drawspeed controller
        throttle += 1 ;
        // If the controller has exceeded a threshold, draw
        if (throttle >= threshold) { 
            // Zero drawspeed controller
            throttle = 0 ;
             // NOTE! This is in 15.16 fixed point. Accel in g's, gyro in deg/s
            // If you want these values in floating point, call fix2float15() on
            // the raw measurements.
            



            // Erase a column
            fillCircle(350,250,149,BLACK);
            //drawCircle(350,250,150,GREEN);
           
            drawVLine(350, 100, 300, GREEN) ;
            drawHLine(350, 250, 150, GREEN) ;
            fillRect(200,100,150, 300, BLACK) ;
            // THis way didn't work because sound does not travel linearly 
            //Approach  using law of cosines 
            // fix15 x= multfix15(int2fix15(cycle_after[0]),cmpercycle) ;
            // fix15 y= multfix15(int2fix15(cycle_after[1]),cmpercycle) ;
            // fix15 z= multfix15(int2fix15(cycle_after[2]),cmpercycle) ;
            // float fx= fix2float15(x);
            // float fy= fix2float15(y);
            // float fz= fix2float15(z);
            
            // fix15 a =int2fix15(-2);
            // fix15 b =multfix15(negtwo,y)+ multfix15(negtwo,x) +multfix15(negtwo,z);
            // fix15 c=int2fix15(40*40+40*40-40*40)-multfix15(y,y)+multfix15(multfix15(int2fix15(-4),x),z);
            // fix15 d=multfix15(int2fix15(40*40),x) +multfix15(int2fix15(40*40),z)-multfix15(int2fix15(40*40),y);
            //     d+=-1*multfix15(x,multfix15(z,z))-multfix15(x,multfix15(y,y))-multfix15(z,multfix15(x,x))-multfix15(z,multfix15(y,y));
            //     d+=multfix15(y,multfix15(x,x))+multfix15(y,multfix15(y,y));

            // // setup for cubic formula
            // fix15 p = divfix(-1*b,multfix15(int2fix15(3),a));
            // fix15 q = multfix15(p,multfix15(p,p)) + divfix(multfix15(b,c)-multfix15(int2fix15(3),multfix15(a,d) ) ,(multfix15(int2fix15(6),multfix15(a,a))));
            // fix15 r = divfix(c,multfix15(3,a));

            //angle=acos(lcos_value);
            // float s= cbrt(float2fix15(q) + sqrt(fix2float15(multfix15(q,q)) + pow(fix2float15(r-multfix15(p,p)),3))); 
            //     s+= cbrt(float2fix15(q)- sqrt(fix2float15(multfix15(q,q)) + pow(fix2float15(r-multfix15(p,p)),3))) +fix2float15(p);
            // lcos_value=((40*40-(s+fx)*(s+fx)-(s+fz)*(s+fz)))/(2*(s+fz)*(s+fx));


            ///Approach using the logic that the group that bruce told us to look at 
  

            // Approach using the greedy method of 
            // int targetloss= (cycle_after[0]-cycle_after[1])*(cycle_after[0]-cycle_after[1])+(cycle_after[1]-cycle_after[2])*(cycle_after[1]-cycle_after[2]);
            
            // long long  minLoss =100000000000;
            // int minX=0;
            // int minY=0;
            // for(int x=-200; x<200;x++){
            //     for(int y=-200;y<200;y++){
            //         int d1=l2distance(x1,y1,x,y);
            //         int d2=l2distance(x2,y2,x,y);
            //         int d3=l2distance(x3,y3,x,y);
            //         long long  loss= abs(targetloss-((long)d1-d2)*(d1-d2)+(d2-d3)*(d2-d3));
            //         if(loss<minLoss){
            //             minLoss=loss;
            //             minX=x;
            //             minY=y;
            //         }
            //     }
            // }
            //angle +=PI/2;
            
            
            // Draw bottom plot (multiply by 800)
           int xcoord = magnitude*cos(angle)+350;
            int ycoord = magnitude*sin(angle)+250;
            //  int xcoord = minX+350;
            // int ycoord = minY+250;
            

            // Draw top plot
            fillCircle(xcoord, ycoord, 10, RED);

                // Read the IMU
           
            // Update horizontal cursor
        
        }
    }
    // Indicate end of thread
    PT_END(pt);
}



// User input thread. User can change draw speed
static PT_THREAD (protothread_serial(struct pt *pt))
{
    PT_BEGIN(pt) ;
    static char classifier ;
    static int test_in ;
    static float float_in ;

    while(1) {
        
        
        //sprintf(pt_serial_out_buffer, "input I if you want to tilt and U if you want to turn \r\n" );
        // serial_write ;
        sprintf(pt_serial_out_buffer, "cycle detected: %d,%d,%d \r\n",cycle_detected[0],cycle_detected[1],cycle_detected[2] );
        serial_write ;
        sprintf(pt_serial_out_buffer, "angle : %f, quad_value %f \r\n",angle,lcos_value);
        serial_write ;
        sprintf(pt_serial_out_buffer, "microphone reading: %d,%d,%d \r\n",microphone_reading[0],microphone_reading[1],microphone_reading[2]);
        serial_write ;

        serial_read;
        sscanf(pt_serial_in_buffer,"%c", &classifier) ;

        if(classifier=='i'||classifier=='I'){
            sprintf(pt_serial_out_buffer, "enter a tilt constant  (~1000-6500): \r\n");
            serial_write ;
            // spawn a thread to do the non-blocking serial read
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            controlTilt=test_in;
        }
        else if(classifier=='u'||classifier=='U'){
            sprintf(pt_serial_out_buffer, "enter a turn constant  (~2500-5000): \r\n");
            serial_write ;
            // spawn a thread to do the non-blocking serial read
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            controlTurn=test_in;
        }
        if(classifier=='s'||classifier=='s'){
            sprintf(pt_serial_out_buffer, "enter a tilt constant  (~1000-6500): \r\n");
            serial_write ;
            // spawn a thread to do the non-blocking serial read
            serial_read ;
            // convert input string to number
            sscanf(pt_serial_in_buffer,"%d", &test_in) ;
            classifier="e";
            sweep=test_in;
        }


        //yield in case we add any more threads on the core
        PT_YIELD_usec(300) ;
    }
    PT_END(pt) ;
}


void core1_entry() {
    pt_add_thread(protothread_gpio) ;
    pt_schedule_start ;
}


int main() {
    initVGA() ;
    set_sys_clock_khz(250000,true);
    // Initialize stdio
    stdio_init_all();
    //Initialize ADC
    adc_init();
    //initialize clock
    //clocks_init();
    
    // initialize frequency to 1 microsecond(1Mz)
    // clock_configure(clk_sys,
                    // CLOCKS_CLK_SYS_CTRL_SRC_VALUE_CLKSRC_CLK_SYS_AUX,
                    // CLOCKS_CLK_SYS_CTRL_AUXSRC_VALUE_CLKSRC_PLL_USB,
                    //  MHZ,
                    //   MHZ);

    //   // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    // // Select ADC input 0 (GPIO26)
    // adc_fifo_setup(true,false,10,false,false);
    adc_set_round_robin	(7)	;
    adc_set_clkdiv (50000);
     gpio_init(25) ;
    
     gpio_set_dir(25, GPIO_OUT) ;
    
     gpio_init(15) ;
     gpio_init(14) ;
     gpio_init(13) ;
      gpio_set_dir(15, GPIO_OUT) ;
     gpio_set_dir(14, GPIO_OUT) ;
     gpio_set_dir(13, GPIO_OUT) ;
    // gpio_set_dir(27, GPIO_IN) ;
    // gpio_set_dir(28, GPIO_IN) ;
    //struct repeating_timer timer;
    //timer to get readings every 1 microseconds
   // add_repeating_timer_us(-17, repeating_timer_callback, NULL, &timer);

    ////////////////////////////////////////////////////////////////////////
    ///////////////////////// PWM CONFIGURATION ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Tell GPIO's 4,5 that they allocated to the PWM
    gpio_set_function(5, GPIO_FUNC_PWM);
    gpio_set_function(4, GPIO_FUNC_PWM);



    // Find out which PWM slice is connected to GPIO 5 (it's slice 2, same for 4)
    slice_num = pwm_gpio_to_slice_num(5);

    // Mask our slice's IRQ output into the PWM block's single interrupt line,
    // and register our interrupt handler
    pwm_clear_irq(slice_num);
    pwm_set_irq_enabled(slice_num, true);
    irq_set_exclusive_handler(PWM_IRQ_WRAP, on_pwm_wrap);
    irq_set_enabled(PWM_IRQ_WRAP, true);

    // This section configures the period of the PWM signals
    pwm_set_wrap(slice_num, WRAPVAL) ;
    pwm_set_clkdiv(slice_num, CLKDIV) ;

    // This sets duty cycle
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0000);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0000);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Initialize VGA
    
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

}
