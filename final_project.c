
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


//variables for motor control 
uint slice_num ;
volatile int old_controlTilt=0;
volatile int old_control_turn=0;
volatile int controlTurn=0;
volatile int controlTilt=0;
volatile int motor_dispTurn=0;
volatile int motor_dispTilt=0;
volatile uint16_t microphone_reading[3]; 
volatile bool detected[3]={false,false,false}; 
volatile long cycle_detected[3]={0,0,0};
volatile int cycle12=0;
volatile int cycle23=0;
volatile int cycle13=0;
volatile int cycle=0;


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
 

    pwm_set_chan_level(slice_num, PWM_CHAN_B, controlTilt);
     pwm_set_chan_level(slice_num, PWM_CHAN_A, controlTurn);

    // Signal VGA to draw
    PT_SEM_SIGNAL(pt, &vga_semaphore);
}

// Timer ISR
bool repeating_timer_callback(struct repeating_timer *t) {
	// DDS phase and sine table lookup
    bool all_detected=false;
    for(int i=0;i<3;i++)
        adc_read();

        

    for(int i=0;i<3;i++){
        microphone_reading[i] = adc_fifo_get();
    }
    for(int i=0;i<3;i++){
        if(detected[i]==true){
            
        }
        else if(microphone_reading[i] >2500 || microphone_reading[i]<2000){
            detected[i]=true;
            cycle_detected[i]=cycle;
            all_detected=true;
            for(int y=0;y<3;y++){
                if(detected[y]==false)
                    all_detected=false;
            }
            
    }
    }
    if(all_detected){
        for(int i=0;i<3;i++)
            detected[i]=false;
        cycle12=cycle_detected[0]-cycle_detected[1];
        cycle13=cycle_detected[0]-cycle_detected[2];
        cycle23=cycle_detected[1]-cycle_detected[2];
        
    }
    cycle++;
    return true;
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
    drawHLine(75, 430, 5, CYAN) ;
    drawHLine(75, 355, 5, CYAN) ;
    drawHLine(75, 280, 5, CYAN) ;
    drawVLine(80, 280, 150, CYAN) ;
    sprintf(screentext, "2500") ;
    setCursor(50, 350) ;
    writeString(screentext) ;
    sprintf(screentext, "+2500") ;
    setCursor(50, 280) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(50, 425) ;
    writeString(screentext) ;

   // Draw top plot
    drawHLine(75, 230, 5, CYAN) ;
    drawHLine(75, 155, 5, CYAN) ;
    drawHLine(75, 80, 5, CYAN) ;
    drawVLine(80, 80, 150, CYAN) ;
    sprintf(screentext, "+2500") ;
    setCursor(50, 150) ;
    writeString(screentext) ;
    sprintf(screentext, "+2500") ;
    setCursor(45, 75) ;
    writeString(screentext) ;
    sprintf(screentext, "0") ;
    setCursor(45, 225) ;
    writeString(screentext) ;
    

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
            drawVLine(xcoord, 0, 480, BLACK) ;


            // Draw bottom plot (multiply by 800)
            drawPixel(xcoord, 490 - (int)(0.2000*((float)((fix2float15(motor_dispTurn))))), RED) ;

            // Draw top plot
            drawPixel(xcoord, 230 - (int)(2000*((float)((fix2float15(motor_dispTilt))))), GREEN) ;

                // Read the IMU
           
            // Update horizontal cursor
            if (xcoord < 609) {
                xcoord += 1 ;
            }
            else {
                xcoord = 81 ;
            }
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
        
        
        // sprintf(pt_serial_out_buffer, "input I if you want to tilt and U if you want to turn \r\n" );
        // serial_write ;
        sprintf(pt_serial_out_buffer, "microphone readings : %d,%d,%d \r\n",microphone_reading[0],microphone_reading[1],microphone_reading[2] );
        serial_write ;
        sprintf(pt_serial_out_buffer, "cycle readings : %d,%d,%d \r\n",cycle12,cycle13,cycle23 );
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
    pt_add_thread(protothread_vga) ;
    pt_schedule_start ;
}


int main() {

    // Initialize stdio
    stdio_init_all();

    // Initialize VGA
    initVGA() ;

    //Initialize ADC
     adc_init();

      // Make sure GPIO is high-impedance, no pullups etc
    adc_gpio_init(26);
    adc_gpio_init(27);
    adc_gpio_init(28);
    // Select ADC input 0 (GPIO26)
    adc_fifo_setup(true,false,10,false,false);
    adc_set_round_robin	(7)	;

    struct repeating_timer timer;
    //timer to get readings every 25 microseconds
    add_repeating_timer_us(-25, repeating_timer_callback, NULL, &timer);

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
    // start core 1 
    multicore_reset_core1();
    multicore_launch_core1(core1_entry);

    // start core 0
    pt_add_thread(protothread_serial) ;

    pt_schedule_start ;

}
