
#if true
//
//  How to access GPIO registers from C-code on the Raspberry-Pi
//  Example program
//  15-January-2012
//  Dom and Gert
//  Revised: 15-Feb-2013

// https://elinux.org/RPi_GPIO_Code_Samples#Direct_register_access

// Access from ARM Running Linux

namespace {

#define BCM2708_PERI_BASE        0x20000000
#define GPIO_BASE                (BCM2708_PERI_BASE + 0x200000) /* GPIO controller */

#include <stdio.h>
#include <stdlib.h>
#include <fcntl.h>
#include <sys/mman.h>
#include <unistd.h>

#define PAGE_SIZE (4*1024)
#define BLOCK_SIZE (4*1024)

   int  mem_fd;
   void* gpio_map;

   // I/O access
   volatile unsigned int* gpio;


   // GPIO setup macros. Always use INP_GPIO(x) before using OUT_GPIO(x) or SET_GPIO_ALT(x,y)
#define INP_GPIO(g) *(gpio+((g)/10)) &= ~(7<<(((g)%10)*3))
#define OUT_GPIO(g) *(gpio+((g)/10)) |=  (1<<(((g)%10)*3))
#define SET_GPIO_ALT(g,a) *(gpio+(((g)/10))) |= (((a)<=3?(a)+4:(a)==4?3:2)<<(((g)%10)*3))

#define GPIO_SET *(gpio+7)  // sets   bits which are 1 ignores bits which are 0
#define GPIO_CLR *(gpio+10) // clears bits which are 1 ignores bits which are 0

#define GET_GPIO(g) (*(gpio+13)&(1<<g)) // 0 if LOW, (1<<g) if HIGH

#define GPIO_PULL *(gpio+37) // Pull up/pull down
#define GPIO_PULLCLK0 *(gpio+38) // Pull up/pull down clock

} // namespace

void set_gpio_to_output(int pin)
{
   INP_GPIO(pin); // must use INP_GPIO before we can use OUT_GPIO
   OUT_GPIO(pin);
}

void set_gpio_to_input(int pin)
{
   INP_GPIO(pin); 
}

void set_gpio(int pin, int state)
{
   if( state != 0)
      GPIO_SET = 1 << pin;
   else
      GPIO_CLR = 1 << pin;
}

int get_gpio(int pin)
{
   if (GET_GPIO(pin) == 0)
      return 0;
   else
      return 1;
}


//
// Set up a memory regions to access GPIO
//
bool setup_gpio()
{
   /* open /dev/mem */
   if ((mem_fd = open("/dev/gpiomem", O_RDWR | O_SYNC)) < 0) {
      printf("can't open /dev/gpiomem \n");
      return false;
   }

   /* mmap GPIO */
   gpio_map = mmap(
      NULL,             //Any adddress in our space will do
      BLOCK_SIZE,       //Map length
      PROT_READ | PROT_WRITE,// Enable reading & writting to mapped memory
      MAP_SHARED,       //Shared with other processes
      mem_fd,           //File to map
      GPIO_BASE         //Offset to GPIO peripheral
   );

   close(mem_fd); //No need to keep mem_fd open after mmap

   if (gpio_map == MAP_FAILED) {
      printf("mmap error\n");  // (int)gpio_map);//errno also set!
      return false;
   }

   // Always use volatile pointer!
   gpio = (volatile unsigned*)gpio_map;

   return true;

} // setup_io


#if false
#include <pigpio.h>
#include <stdio.h>
#include <unistd.h>

bool setup_gpio()
{
   return (gpioInitialise() >= 0);
}

void set_gpio_to_output(int pin)
{
   gpioSetMode(pin, PI_OUTPUT);
}

void set_gpio_to_input(int pin)
{
   gpioSetMode(pin, PI_INPUT);
}

void set_gpio(int pin, int state)
{
   gpioWrite(pin, state);
}

int get_gpio(int pin)
{
   return gpioRead(pin);
}
#endif

#if false
#include <wiringPi.h>
#include <stdio.h>
#include <unistd.h>

bool setup_gpio()
{
   wiringPiSetupGpio();
}

void set_gpio_to_output(int pin)
{
   pinMode(pin, OUTPUT);
}

void set_gpio_to_input(int pin)
{
   pinMode(pin, INPUT);
}

void set_gpio(int pin, int state)
{
   digitalWrite(pin, state);
}

int get_gpio(int pin)
{
   digitalRead(pin);
}
#endif




#if false
void printButton(int g)
{
   if (GET_GPIO(g)) // !=0 <-> bit is 1 <- port is HIGH=3.3V
      printf("Button pressed!\n");
   else // port is LOW=0V
      printf("Button released!\n");
}
#endif


#if false
int demo(int argc, char** argv)
{
   int g, rep;

   // Set up gpi pointer for direct register access
   setup_io();

   // Switch GPIO 7..11 to output mode

  /************************************************************************\
   * You are about to change the GPIO settings of your computer.          *
   * Mess this up and it will stop working!                               *
   * It might be a good idea to 'sync' before running this program        *
   * so at least you still have your code changes written to the SD-card! *
  \************************************************************************/

  // Set GPIO pins 7-11 to output
   for (g = 7; g <= 11; g++)
   {
      INP_GPIO(g); // must use INP_GPIO before we can use OUT_GPIO
      OUT_GPIO(g);
   }

   for (rep = 0; rep < 10; rep++)
   {
      for (g = 7; g <= 11; g++)
      {
         GPIO_SET = 1 << g;
         //        sleep(1);
      }
      for (g = 7; g <= 11; g++)
      {
         GPIO_CLR = 1 << g;
         //        sleep(1);
      }
   }

   return 0;

} // main
#endif

#endif

