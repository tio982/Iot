
#include "dev/light-sensor.h"
#include <stdio.h>
#include <math.h>

#define MAX_BUFFER_SIZE 8
#define LOW_THRESHOLD 50
#define HIGH_THRESHOLD 200
#define MEDIUM_SEGMENT 2

static float light_readings[MAX_BUFFER_SIZE];
static int readings_index = 0;
float autocorr[MAX_BUFFER_SIZE];
float dct[MAX_BUFFER_SIZE];
float result[MAX_BUFFER_SIZE];
static float ema_readings[MAX_BUFFER_SIZE] = {0};

//Integer part of float
int d1(float f)
{
return (int)f;
}

//Fractional part of float
unsigned int d2(float f)
{
  if(f>0)
  {
   return (unsigned int)(1000*(f-(float)d1(f)));
   }

   else
     {
       return(unsigned int)(1000*((float)d1(f)-f));
      }
}


//Custom square root function using Newton-Raphson method
float custom_sqrt(float number)
{
 if(number <=0)
  {
   return 0;
  }

  float estimate = number;
  float epsilon = 0.0001; // precision of number
  while(1)
    {
     float new_estimate = (estimate + number/ estimate)/2;
     if(fabs(new_estimate - estimate) < epsilon)
      {
       break;
       }
       estimate = new_estimate;
     }
    return estimate;
}


float getLight(void)
{
  float V_sensor = 1.5 * light_sensor.value(LIGHT_SENSOR_PHOTOSYNTHETIC)/4096;
  float I = V_sensor / 100000;
  float light_lx = 0.0625 *1e6*I*1000;
  return light_lx;
}


float calculate_average(float data[], int size)
{
  float total = 0.0;
  int i;
  for(i=0; i < size; i++)
   {
     total += data[i];
   }
   return total/size;
}


float calculate_standard_deviation(float data[], int size, float mean)
{
  float sum = 0.0;
  int i;
  for(i=0; i < size; i++)
    {
      sum += (data[i] - mean)*(data[i] - mean);
     }
   return custom_sqrt(sum/size);
}


float custom_pow(float base, int exponent)
{
  float value = 1;
  while(exponent > 0)
    {
     value *= base;
     -- exponent;
     }
     return value;
}


float factorial(int n)
{
  float result = 1;
  int i;
  for(i=1; i <= n; i++)
    {
     result += i;
     }
     return result;
}


float calculate_cos(float x)
{
  float sum = 0;
  int i;
  int sign = 1;

  for(i=0; i < 8; i++)
    {
      sum += sign * custom_pow(x, 2*i)/ factorial(2*i);
      sign = -sign;
     }
     return sum;
}


void process_data(float std_dev, float mean)
{
  // Buffer contents
  printf("B = [");
  int i;

  for(i=0; i < MAX_BUFFER_SIZE; i++)
    {
     if(i < MAX_BUFFER_SIZE -1)
      {
       printf("%d.%03u, ", d1(light_readings[i]), d2(light_readings[i]));
       }
        else
          {
           printf("%d.%03u, ", d1(light_readings[i]), d2(light_readings[i]));
           }
      }
      printf("\n");

      // Standard deviation
      printf("StdDev = %d.%03u\n", d1(std_dev), d2(std_dev));

      if(std_dev < LOW_THRESHOLD)
        {
         // Low activity detected
         printf("Low Activity Detected\n");
         printf("X = [%d.%03u]\n", d1(mean), d2(mean));
         }

         else if(std_dev < HIGH_THRESHOLD)
          {
            printf("Medium Activity Detected\n");
            printf("Aggregation = 4-into-1\n");
            printf("X = [");
            int i;
           
            for(i=0; i < MAX_BUFFER_SIZE; i += MEDIUM_SEGMENT)
              {
               float segment_average = calculate_average(&light_readings[i], MEDIUM_SEGMENT);
               if(i + MEDIUM_SEGMENT < MAX_BUFFER_SIZE)
                {
                  printf("%d.%03u\n", d1(segment_average), d2(segment_average));
                 }
                 else
                   {
                     printf("%d.%03u\n", d1(segment_average), d2(segment_average));
                    }
                }
                printf("]\n");
            }
            else
              {
               printf("High Activity Detected\n");
               printf("X = [");
               int i;

       for(i=0; i < MAX_BUFFER_SIZE; i++)
                 {
                  if(i < MAX_BUFFER_SIZE -1)
                    {
                    printf("%d.%03u, ", d1(light_readings[i]), d2(light_readings[i]));
                    }
                    else
                     {
                     printf("%d.%03u, ", d1(light_readings[i]), d2(light_readings[i]));
                      }
                  }
                  printf("\n");
                 }
}


void calculate_autocorrelation(const float data[], int size, float result[])
{
  int i;
  int j;
  float sum;
  float normalisation_factor;

  normalisation_factor = 0;
  for(i=0; i < size; i++)
    {
     normalisation_factor += data[i]*data[i];
    }

   for(i=0; i < size; i++)
     {
      sum = 0;
      for(j=0; j < size -i; j++)
        {
         sum += data[j]*data[j+i];
         }
         result[i] = sum/ normalisation_factor;
      }
}


void calculate_dct(const float data[], int length, float result[])
{
  int k;
  int n;
  float cumul_sum;
  float factor;

  factor = M_PI/(2.0*length);

  for(k=0; k < length; k++)
    {
      cumul_sum = 0;
      for(n=0; n < length; n++)
        {
          cumul_sum += 2*data[n]*calculate_cos(factor * k *(2*n+1));
         }
         result[k] = cumul_sum;
      }
}

void buffer_calculations(float *buffer, int size)
  {

    float mean;
    float std_dev;
    float autocorr[MAX_BUFFER_SIZE];
    float dct[MAX_BUFFER_SIZE];
    int i;
   

    autocorr[MAX_BUFFER_SIZE];
    dct[MAX_BUFFER_SIZE];

    calculate_autocorrelation(buffer, size, autocorr);
    calculate_dct(autocorr, size, dct);

     printf("Autocorrelation Values: [");

        for(i=0; i < MAX_BUFFER_SIZE; i++)
          {
           printf("%d.%03u", d1(autocorr[i]), d2(autocorr[i]));
           if(i < MAX_BUFFER_SIZE -1)
             {
              printf(", ");
              }
           }
           printf("]\n");

         printf("DCT Values: [");

        for(i=0; i < MAX_BUFFER_SIZE; i++)
          {
           printf("%d.%03u", d1(dct[i]), d2(dct[i]));
           if(i < MAX_BUFFER_SIZE -1)
             {
              printf(", ");
              }
           }
           printf("]\n");
    }

void calculate_ema(float *readings, float *ema_readings, float s_factor, int size)
  {
   printf("Smoothing Factor = %d.%03u, ", d1(s_factor), d2(s_factor));
   ema_readings[0] = readings[0];
   int i;
   for(i=1; i < size; i++)
     {
       ema_readings[i] = s_factor*readings[i] +(1 - s_factor)*(1 - ema_readings[i-1]);
      }
   
    printf("EMA = [");
    for(i=0; i < size; i++)
      {
        printf("%d.%03u, ", d1(ema_readings[i]), d2(ema_readings[i]));
       }
       printf("]\n");
   }

/*---------------------------------------------------------------------------*/
PROCESS(sensor_reading_process, "Sensor reading process");
AUTOSTART_PROCESSES(&sensor_reading_process);
/*---------------------------------------------------------------------------*/
PROCESS_THREAD(sensor_reading_process, ev, data)
{
  static struct etimer timer;
  float light_reading;
  static float ema_readings[MAX_BUFFER_SIZE] = {0};
  float s_factor = 0.6;

  PROCESS_BEGIN();
  etimer_set(&timer, CLOCK_CONF_SECOND/2);
                                           
  SENSORS_ACTIVATE(light_sensor);

  while(1)
  {
    PROCESS_WAIT_EVENT_UNTIL(ev=PROCESS_EVENT_TIMER);
    light_reading = getLight();
    light_readings[readings_index] = light_reading;


    //printf("Current Reading: %d.%03u lux/n", d1(light_readings[readings_index]), d2(light_readings[readings_index]));

    readings_index = (readings_index + 1) % MAX_BUFFER_SIZE;

    if(readings_index == 0)
      {
        //buffer_calculations(light_readings, MAX_BUFFER_SIZE);
float mean = calculate_average(light_readings, MAX_BUFFER_SIZE);
        float std_dev = calculate_standard_deviation(light_readings, MAX_BUFFER_SIZE, mean);
process_data(std_dev, mean);
        calculate_ema(light_readings, ema_readings, s_factor, MAX_BUFFER_SIZE);
      }
         
    etimer_reset(&timer);
  }
  PROCESS_END();
}
/*---------------------------------------------------------------------------*/  