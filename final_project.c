
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
#define CYCLELIMIT 143
#define THRESHOLD 1800

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
        }
        else if(gp26&&gp27){
            cycle_detected[0]=cycle;
            cycle_detected[1]=cycle;
            adc_set_round_robin	(4)	;
            while(!gp28&&last_cycle>=cycle){
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
        }
        else if(gp26&&gp28){
            cycle_detected[0]=cycle;
            cycle_detected[2]=cycle;
            adc_set_round_robin	(2)	;
            while(!gp27&&last_cycle>=cycle){
                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
        }
        else if(gp27&&gp28){
            cycle_detected[1]=cycle;
            cycle_detected[2]=cycle;
            adc_set_round_robin	(1)	;
            while(!gp26&&last_cycle>=cycle){
                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
        }
        else if(gp26){
            cycle_detected[0]=cycle;
            adc_set_round_robin	(6)	;
            while(!(gp27||gp28)&&last_cycle>=cycle){
                gp27=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
            if((gp27&&gp28)||last_cycle<cycle){
                cycle_detected[1]=cycle;
                cycle_detected[2]=cycle;
            }
            else if(gp27){
                cycle_detected[1]=cycle;
                adc_set_round_robin	(4)	;
                while(!gp28&&last_cycle>=cycle){
                    
                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
            }
            else{
                cycle_detected[2]=cycle;
                adc_set_round_robin	(2)	;
                while(!gp27&&last_cycle>=cycle){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
            }
            
        }
        else if(gp27){
            cycle_detected[1]=cycle;
            adc_set_round_robin	(5)	;
            while(!(gp26||gp28)&&last_cycle>=cycle){
                gp26=adc_read() >THRESHOLD;
                gp28=adc_read() >THRESHOLD;
                cycle++;
            }
             if((gp26&&gp28)||last_cycle<cycle){
                cycle_detected[0]=cycle;
                cycle_detected[2]=cycle;
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                adc_set_round_robin	(4)	;
                while(!gp28&&last_cycle>=cycle){

                    gp28=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[2]=cycle;
            }
            else{
                cycle_detected[2]=cycle;
                adc_set_round_robin	(1)	;
                while(!gp26&&last_cycle>=cycle){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
            }
        }
        else{
            cycle_detected[2]=cycle;
            adc_set_round_robin	(3)	;
            while(!(gp26||gp27)&&last_cycle>=cycle){
                gp26=adc_read() >THRESHOLD;
                gp27=adc_read() >THRESHOLD;
                cycle++;
            }
              if((gp26&&gp27)||last_cycle<cycle){
                cycle_detected[0]=cycle;
                cycle_detected[1]=cycle;
            }
            else if(gp26){
                cycle_detected[0]=cycle;
                adc_set_round_robin	(2)	;
                while(!gp27&&last_cycle>=cycle){

                    gp27=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[1]=cycle;
            }
            else{
                cycle_detected[1]=cycle;
                adc_set_round_robin	(1)	;
                while(!gp26&&last_cycle>=cycle){

                    gp26=adc_read() >THRESHOLD;
                    cycle++;
                }
                cycle_detected[0]=cycle;
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
    setTextSize(1) ;
    setTextColor(WHITE);
    setCursor(300, 50) ;
    
    sprintf(screentext, "Sound direction tracking") ;
    writeString(screentext) ;
    // Draw bottom plot


   // Draw top plot
    drawCircle(350,250,150,GREEN);
    drawVLine(350, 100, 300, GREEN) ;
    drawHLine(200, 250, 300, GREEN) ;

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
            drawCircle(350,250,150,GREEN);
            drawVLine(350, 100, 300, GREEN) ;
            drawHLine(200, 250, 300, GREEN) ;
            int x= cycle_after[0]/6;
            int y= cycle_after[1]/6;
            int z= cycle_after[2]/6;
            int b =(-2*y+2*x+2*z);
            int a =-4;
            int c=-3*y*y+x*x+z*z+593;
            int s= (-(b)+ sqrt(b*b-4*a*c))/(2*a);
             
            
            angle=acos(((double)(40*40-(s+x)*(s+x)-(s+z)*(s+z)))/(2*(s+z)*(s+x)));
            
            
            // Draw bottom plot (multiply by 800)
            int xcoord = magnitude*cos(angle)+350;
            int ycoord = magnitude*sin(angle)+250;
            

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
        sprintf(pt_serial_out_buffer, "turn readings : %d, \r\n",pwm_turn);
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
    // gpio_init(27) ;
    // gpio_init(28) ;
     gpio_set_dir(25, GPIO_OUT) ;
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
