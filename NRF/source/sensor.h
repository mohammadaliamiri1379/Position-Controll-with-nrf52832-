// this library used for sensor functions
#include <mbed.h>



extern float  MAX ; 
// when max_2 reach 0 , return max_1 (in real sensor It's possible 
//that the sensor does not work well so at the end an error will show up
//so we use down_lim for best estimation.

inline float calibrate (float max_1 ,float max_2)
{
    float max = ::MAX ;
    float down_lim = 650.0f; 
    float up_lim = 5000.0f ;
         
    if (max_2 <= down_lim && max_1 > max && max_1>=up_lim)
        {  
        
        ::MAX = max_1 ;
        return((float)max_1) ;  
        }
    else
    {
        return 0 ;     
    }
}
// you must use the code below in int function for each analog in

/*
int main_()
{

int a , b , i;
float max_b ,max_a , max_a_s , max_b_s;
////start
::MAX =0 ;
i =0 ; 
do{
    a = asen1_1.read_u16();
    b = asen1_2.read_u16();  
    g_Serial_pc.printf("analog 1 :  %d \n", a);
    g_Serial_pc.printf("analog 2 :  %d \n", b);
    g_Serial_pc.printf("\n");
    max_a_s =calibrate((float)a , (float)b) ;
if (max_a_s == 0){
g_Serial_pc.printf("please maximize the potentiometer clockwise \n");
}

if (i>=10 && max_a_s!=0)
{
    g_Serial_pc.printf("potentiometer deos not work well \n");
    g_Serial_pc.printf("max_a_s = %f \n" , max_a_s);
    break;
}

thread_sleep_for(2000);
i++ ;
}while(max_a_s==0 || i<=5);
/// end

}
*/


// check if sensor work well
inline bool evaluate_sensor(float ch1 , float ch2)
{
    float c;
    float up_lim = 1.3 ;
    float down_lim = 0.7 ; 
    c = ch1 + ch2 ;
    if (c >= up_lim || c<= down_lim)
    {
        return 1 ;
        
        }
    else
    {
        return 0 ;
        }
    } 
    