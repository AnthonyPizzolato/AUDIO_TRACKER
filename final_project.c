
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
#define CYCLELIMIT 21676
#define THRESHOLD 2600

//variables for motor control 
uint slice_num ;
volatile int old_controlTilt=0;
volatile int old_control_turn=0;
volatile int controlTurn=0;
volatile int controlTilt=0;
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
const float conversion_factor = 3.3f / (1 << 12);

void on_pwm_wrap() {

    // Clear the interrupt flag that brought us here
    pwm_clear_irq(pwm_gpio_to_slice_num(5));

    if(old_control_turn!=controlTurn){
        old_control_turn=controlTurn;
    }
    if(old_controlTilt!=controlTilt){
        old_controlTilt=controlTilt;
    }
        motor_dispTilt = motor_dispTilt + ((controlTilt - motor_dispTilt)>>4) ;
        motor_dispTurn = motor_dispTurn + ((controlTurn - motor_dispTurn)>>4) ;
 
    // logic to control the pwm from the microphone
    //20,000 - -20,000
    int turn=cycle_after[0]-cycle_after[1];
    turn +=CYCLELIMIT;
    turn=(int)(turn*12.5);
    turn+=1000;
    pwm_turn=turn;

    pwm_set_chan_level(slice_num, PWM_CHAN_B, controlTilt);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, pwm_turn);
    


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
        //cycle=0;
        adc_set_round_robin	(7)	;
        while(!gp26||!gp27||!gp28){
            microphone_reading[0]=adc_read();
            microphone_reading[1]=adc_read();
            microphone_reading[2]=adc_read();
            gp26= microphone_reading[0]>THRESHOLD;
            gp27=microphone_reading[1] >THRESHOLD;
            gp28=microphone_reading[2] >THRESHOLD;
            cycle++;
        }
        if(gp26&&gp27&&gp28){
            cycle_detected[0]=cycle;
            cycle_detected[1]=cycle;
            cycle_detected[2]=cycle;
        }
        else if(gp26&&gp27){
            cycle_detected[0]=cycle;
            cycle_detected[1]=cycle;
            adc_set_round_robin	(4)	;
            while(!gp28){
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
        }
        else if(gp26&&gp28){
            cycle_detected[0]=cycle;
            cycle_detected[2]=cycle;
            adc_set_round_robin	(2)	;
            while(!gp27){
                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
        }
        else if(gp27&&gp28){
            cycle_detected[1]=cycle;
            cycle_detected[2]=cycle;
            adc_set_round_robin	(1)	;
            while(!gp26){
                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
        }
        else if(gp26){
            cycle_detected[0]=cycle;
            adc_set_round_robin	(6)	;
            while(!gp27||!gp28){
                gp27=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
            if(gp27&&gp28){
                cycle_detected[1]=cycle;
                cycle_detected[2]=cycle;
            }
            else if(gp27){
                cycle_detected[1]=cycle;
                adc_set_round_robin	(4)	;
                while(!gp28){
                    
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
            }
            else{
                cycle_detected[2]=cycle;
                adc_set_round_robin	(2)	;
                while(!gp27){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
            }
            
        }
        else if(gp27){
            cycle_detected[1]=cycle;
            adc_set_round_robin	(5)	;
            while(!gp26||!gp28){
                gp26=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
             if(gp26&&gp28){
                cycle_detected[0]=cycle;
                cycle_detected[2]=cycle;
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                adc_set_round_robin	(4)	;
                while(!gp28){

                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
            }
            else{
                cycle_detected[2]=cycle;
                adc_set_round_robin	(1)	;
                while(!gp26){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
            }
        }
        else{
            cycle_detected[2]=cycle;
            adc_set_round_robin	(3)	;
            while(!gp26||!gp27){
                gp26=adc_read() >THRESHOLD;
                gp27=adc_read() >THRESHOLD;
                cycle++;
            }
              if(gp26&&gp27){
                cycle_detected[0]=cycle;
                cycle_detected[1]=cycle;
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                adc_set_round_robin	(2)	;
                while(!gp27){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
            }
            else{
                cycle_detected[1]=cycle;
                adc_set_round_robin	(1)	;
                while(!gp26){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
            }
        }


        // bool all_detected=false;
        // for(int i=0;i<3;i++){
        //     microphone_reading[i] = gpio_get(26+i);
        // }
        // int prev_num_detected=num_detected;
        // int index_detected=0;
        // for(int i=0;i<3;i++){
        //     if(detected[i]==true){
        //         num_detected++;
        //         index_detected=i;
        //     }
        //     else if(serial_write ;) {
        //         detected[i]=true;
        //         cycle_detected[i]=cycle;
        //         all_detected=true;
        //         for(int y=0;y<3;y++){
        //             if(detected[y]==false)
        //                 all_detected=false;
        //         }
            
        //     }
        // }
        // if(num_detected==1||prev_num_detected==0){
        //     first_cycle=cycle_detected[index_detected];
        // }
        // if(cycle-first_cycle==CYCLELIMIT){
        //     all_detected=true;
        //     for(int x=0;x<3;x++){
        //         if(detected[x]==false){
        //             cycle_detected[x]=cycle;
        //         }
        //     }
        // }
        //if(all_detected){
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
        cycle++;
        
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
    setTextSize(1) ;
    setTextColor(WHITE);

    // Draw bottom plot


   // Draw top plot
    drawCircle(300,200,150,GREEN);
    drawVLine(300, 50, 300, GREEN) ;
    drawHLine(150, 200, 300, GREEN) ;

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
            fillCircle(300,200,149,BLACK);
            drawCircle(300,200,150,GREEN);
            drawVLine(300, 50, 300, GREEN) ;
            drawHLine(150, 200, 300, GREEN) ;
            int diff=cycle_after[0]-cycle_after[1];
            angle=((double)diff)/CYCLELIMIT;
            angle*=PI/2;
            angle+=PI/2;
            
            // Draw bottom plot (multiply by 800)
            int xcoord = magnitude*cos(angle)+300;
            int ycoord = magnitude*sin(angle)+200;
            

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
        sprintf(pt_serial_out_buffer, "microphone readings: %d,%d,%d \r\n",cycle_detected[0],cycle_detected[1],cycle_detected[2] );
        serial_write ;
        sprintf(pt_serial_out_buffer, "turn readings : %d, \r\n",pwm_turn);
        serial_write ;
        sprintf(pt_serial_out_buffer, "cycle readings : %f,%f,%f \r\n",microphone_reading[0]*conversion_factor,microphone_reading[1]*conversion_factor,microphone_reading[2]*conversion_factor);
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
    set_sys_clock_khz(250000,true);
    // Initialize stdio
    stdio_init_all();
    //Initialize ADC
    adc_init();
    

    

    //   // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    // // Select ADC input 0 (GPIO26)
    // adc_fifo_setup(true,false,10,false,false);
    adc_set_round_robin	(7)	;
    adc_set_clkdiv (50000);
    // gpio_init(26) ;
    // gpio_init(27) ;
    // gpio_init(28) ;
    // gpio_set_dir(26, GPIO_IN) ;
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
    pwm_set_chan_level(slice_num, PWM_CHAN_B, 0);
    pwm_set_chan_level(slice_num, PWM_CHAN_A, 0);

    // Start the channel
    pwm_set_mask_enabled((1u << slice_num));


    ////////////////////////////////////////////////////////////////////////
    ///////////////////////////// ROCK AND ROLL ////////////////////////////
    ////////////////////////////////////////////////////////////////////////
    // Initialize VGA
    initVGA() ;
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;

}
