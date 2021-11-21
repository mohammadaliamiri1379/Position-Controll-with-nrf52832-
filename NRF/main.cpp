/* mbed Microcontroller Library
 * Copyright (c) 2017 ARM Limited
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

#include <stdio.h>
#include "mbed.h"
#include <sensor.h>

// #include "pid_controller.h"

#include "platform/Callback.h"
#include "events/EventQueue.h"
#include "platform/NonCopyable.h"

#include "ble/BLE.h"
#include "ble/Gap.h"
#include "ble/GattClient.h"
#include "ble/GapAdvertisingParams.h"
#include "ble/GapAdvertisingData.h"
#include "ble/GattServer.h"
#include "BLEProcess.h"
#include <math.h>
// variables 
// DigitalOut led(p25);
#define BAUDRATE 9600
Serial g_Serial_pc(p15, p16 , BAUDRATE);      // tx    rx
I2C i2c(p7, p6); // I2C device sda, scl

AnalogIn   ain1_1(p5);
AnalogIn   ain1_2(p4);
AnalogIn   asen1_1(p3);
AnalogIn   asen1_2(p2 );

DigitalOut En(p18 , 0);
DigitalOut forw(p17 , 0);
DigitalOut back(p19 , 0);
PwmOut speed(p24);
float P = 0.4f ;
float dead_band  =  0.02f ;
float  MAX=0 ;  

uint16_t kp ;   // PID variables
uint16_t setpoint_value ; // the setpoint value will store hear
uint16_t feedback_value ; // the feedback value 
uint16_t MaxR , MaxL ;    // safty requirements. this value show the max angle that steer can spin in clockwise and Counterclockwise
//   char DATA[4];
uint16_t setpoint_setting ; // this value shows kind setpoint plant uses
uint16_t feedback_setting ; // this value shows kind feedback plant uses
uint16_t error_mic ;       // this value store error code 

unsigned int counter = 0 ;

/////////////////////


using mbed::callback;

/**
 * A Clock service that demonstrate the GattServer features.
 *
 * The clock service host three characteristics that model the current hour,
 * minute and second of the clock. The value of the second characteristic is
 * incremented automatically by the system.
 *
 * A client can subscribe to updates of the clock characteristics and get
 * notified when one of the value is changed. Clients can also change value of
 * the second, minute and hour characteristric.
 */
class EPSService {
    typedef EPSService Self;

public:
    EPSService() :
        _setpoint_setting("485f4145-52b9-4644-af1f-7a6b9322490f", 0),
        _feedback_setting("0a924ca7-87cd-4699-a3bd-abdcd9cf126a", 0),
        _control_setting("8dd6a1b7-bc75-4741-8a26-264af75807de", 0),
        _deadband("afe820e4-a6b6-4ab7-9d1d-63fa94574014" , 0) ,
        _Kp("74155475-b081-472a-9860-0f931cb54a94" , 0) ,
        _Ki("12bdf99a-3e19-4dac-9d19-a8b97f58357e" , 0) ,
        _Kd("53660b5a-1a02-467d-9996-e18a98d20dce" , 0) ,
        _sensitivity("dbb649f3-cc15-480f-a3d9-b34194f02c89" , 0) ,
        
        _EPS_service(
            /* uuid */ "51311102-030e-485f-b122-f8f381aa84ed",
            /* characteristics */ _EPS_characteristics,
            /* numCharacteristics */ sizeof(_EPS_characteristics) /
                                     sizeof(_EPS_characteristics[0])
        ),
        _server(NULL),
        _event_queue(NULL)
    {
        // update internal pointers (value, descriptors and characteristics array)
        _EPS_characteristics[0] = &_setpoint_setting;
        _EPS_characteristics[1] = &_feedback_setting;
        _EPS_characteristics[2] = &_control_setting;
        _EPS_characteristics[3] = &_deadband;
        _EPS_characteristics[4] = &_Kp;
        _EPS_characteristics[5] = &_Ki;
        _EPS_characteristics[6] = &_Kd;
        _EPS_characteristics[7] = &_sensitivity;
        // setup authorization handlers
        _setpoint_setting.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _feedback_setting.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _control_setting.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _deadband.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _Kp.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _Ki.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _Kd.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
        _sensitivity.setWriteAuthorizationCallback(this, &Self::authorize_client_write);
    }



    void start(BLE &ble_interface, events::EventQueue &event_queue)
    {
         if (_event_queue) {
            return;
        }
        
        forw = 0;
    back = 0;
    speed.period(0.001f);      // 1000 hz
    speed.write(1.0f);      // 
    
        _server = &ble_interface.gattServer();
        _event_queue = &event_queue;

        // register the service
        printf("Adding demo service\r\n");
        ble_error_t err = _server->addService(_EPS_service);

        if (err) {
            printf("Error %u during demo service registration.\r\n", err);
            return;
        }

        // read write handler
        _server->onDataSent(as_cb(&Self::when_data_sent));
        _server->onDataWritten(as_cb(&Self::when_data_written));
        _server->onDataRead(as_cb(&Self::when_data_read));

        // updates subscribtion handlers
        _server->onUpdatesEnabled(as_cb(&Self::when_update_enabled));
        _server->onUpdatesDisabled(as_cb(&Self::when_update_disabled));
        _server->onConfirmationReceived(as_cb(&Self::when_confirmation_received));

        // print the handles
       /* printf("clock service registered\r\n");
        printf("service handle: %u\r\n", _EPS_service.getHandle());
        printf("\thour characteristic value handle %u\r\n", _setpoint_setting.getValueHandle());
        printf("\tminute characteristic value handle %u\r\n", _feedback_setting.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _control_setting.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _deadband.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _Kp.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _Ki.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _Kd.getValueHandle());
        printf("\tsecond characteristic value handle %u\r\n", _sensitivity.getValueHandle());
    */
        
    }

private:

    /**
     * Handler called when a notification or an indication has been sent.
     */
     typedef union
    {
        uint16_t u16;
        int16_t i16;
        uint8_t b[2];
    }data16;
    
    void when_data_sent(unsigned count)
    {
        
        printf("sent %u updates\r\n", count);
    }
    
void stable ()
{
    
    forw = 0;
    back = 0;
    speed.period(0.001f);      // 1000 hz
    speed.write(1.0f);      // 
    }
      
bool Interblock(float S1 , float S2 )
{
    float up_dead_band , down_dead_band ;
    up_dead_band = ((float)I2CEE3ReadInt8(40))/100.0f ;
    down_dead_band = ((float)I2CEE3ReadInt8(48))/100.0f ;
    if(S1 >= up_dead_band|| S2 >=up_dead_band || S1<=down_dead_band|| S2<=down_dead_band)
    {
        g_Serial_pc.printf("Interblock  %f-%f \n" , S1 , S2);
        stable();
        return 1 ;
        }
    else
    {
        return 0 ;
        }
    }
    void counterclockwise_motor(float Er)
{
    float kp = ((float)I2CEE3ReadInt8(32))/100.0f ;
    float out ;
    
    Er = -Er ;
    Er = 1.0f - Er ;    // 
    if(Er >1)
    {
        Er = 1;
        }
    if(Er< 0 )
    {
        Er = 0 ;
        }
    
    back = 0;
    forw = 1 ;
    speed.period(0.001f);      // 1000 hz
    out = Er*kp ;
    speed.write(out);      // 
    }
    
       
void clockwise_motor(float Er)
{
    float kp = ((float)I2CEE3ReadInt8(32))/100.0f ;
    float out;
    Er = 1.0f - Er ;
    if(Er >1)
    {
        Er = 1;
        }
    if(Er< 0 )
    {
        Er = 0 ;
        }
    En = 1;
    back = 1;
    forw = 0 ;
    speed.period(0.001f);      // 1000 hz
    out = Er*kp ;
    speed.write(out);      // 
    
    }
    

uint8_t I2CEE3ReadInt8(uint16_t Address)
{
   int b = 1 ;
   int c = 1;
   
    char da[2];
    char re[2] ;
    da[0]= (Address>> 8) & 0xff;
    da[1]= Address & 0xff;
    b= i2c.write(0xA0 + 1, da, 2); 
    //thread_sleep_for(10);
    wait_ms(3) ;
    c= i2c.read(0xA0 + 1, re, 2);
    
    if ((b!= 0) || (c!= 0))
    {
        g_Serial_pc.printf("error in e2prom read \n") ;
    }
    
    return re[0] ;
    
}

bool I2CEE3WriteByte(uint16_t Address, uint8_t data)
{
    char da[3];
    da[0]= (Address >> 8) & 0xff;
    da[1]= Address & 0xff;
    da[2]= data;
    int a;
    bool ret= true;
            a= 0; 
            a= i2c.write(0xA0, da, 3);
            if (a!= 0)
            {
                g_Serial_pc.printf("error in e2prom write \n") ;
                ret= false;
            }
           // ThisThread::sleep_for(10);
           wait_ms(3);
            return ret;
}

uint16_t Read16(uint16_t Address)

{
    data16 data ;
    data.b[1] = I2CEE3ReadInt8(Address) ;
    data.b[0] = I2CEE3ReadInt8(Address+4) ;
    
    return data.u16 ;
    }
    
void Write16(uint16_t Address , uint16_t data)
{
    
    data16 data_ ;
    data_.u16 = data;
    I2CEE3WriteByte(Address  , data_.b[1]) ;
    I2CEE3WriteByte(Address+4  , data_.b[0]) ;
    
    
    }

void I2CAddressScan(bool ShowAllAddresses)
{
    char da[3];
    da[1]= 0;
    int a, b, cnt= 0;
    
    g_Serial_pc.printf("\n\r   *** Scanning All Registers of I2C devices.\n\r");
        for(int addr= 0; addr< 0xff; addr++) //Registers scan 256
        {
           // watchdogTimer.kick();
            //DEBUG("Scanning Address= %d\n\r", addr);
            a= 1; 
            b= 1;// 0 is correct reply of connected I2C Device (even without pull up resistor.
            da[0]= addr;
            a= i2c.write(addr, da, 2);
            da[0]= 0;
            //b= i2c.write(WriteAddress+ 1, da, 1);
            b= i2c.read(addr+ 1, (char *)da, 1);//from read address
            
            if ((a== 0) || (b== 0))
            {
                //I2CAddressFound[cnt]= addr;
                if (cnt< 10)
                {
                    cnt++;
                }
                g_Serial_pc.printf("\n\rAddress: %X    ReadValue: %X  on write: %d  on read: %d  Correct on write or on read acknowlage is ""NOT Zero"".  \n\r",  addr, da[0], a, b);//If a, b, c ==0 is OK
            }
            ThisThread::sleep_for(100);
        }
}


    /**
     * Handler called after an attribute has been written.
     */
        void print_address(const Gap::Address_t &addr)
{
           g_Serial_pc.printf("%02x:%02x:%02x:%02x:%02x:%02x\r\n",
           addr[5], addr[4], addr[3], addr[2], addr[1], addr[0]);
}

    void when_data_written(const GattWriteCallbackParams *e)
    {
       i2c.start() ;
       int i ,max = 0 ,Data ;  
       float max_b ,max_a , max_a_s , max_b_s;
       uint16_t address = 0x0000;  
       int a = 0 ;
       int b = 0 ;
        if (e->handle == _setpoint_setting.getValueHandle())
        {
            
        if (e->data[0] == 1 )
        {
            g_Serial_pc.printf("start input calibrating:\n");
        address += 4 ;    
        i = 0 ;
        // input calibrate
        ::MAX = 0 ;
        do{
            
            a = ain1_1.read_u16();
            b = ain1_2.read_u16();  
            g_Serial_pc.printf("analog 1 :  %d \n", a);
            g_Serial_pc.printf("analog 2 :  %d \n", b);
            g_Serial_pc.printf("\n");
            max_a =calibrate((float)a , (float)b) ;
            if (max_a > max )
            {
                max = max_a ;
                }
        if (max_a == 0){
        g_Serial_pc.printf("please maximize the potentiometer counterclockwise :%d \n" , i );
        }
        
        if (i>=10 && max!=0)
        {
            g_Serial_pc.printf("potentiometer deos not work well \n");
            g_Serial_pc.printf("max_a = %f \n" , max);
            break;
        }
        
        wait_ms(2000);
        i++ ;
        }while(max==0 || i<=5);
        ////
        max_a = max ; 
        Write16(address,max_a) ;
        g_Serial_pc.printf("max  :%f   eeprom: %d \n" , max_a , (int)Read16(address));
        address+=8 ;
        
        g_Serial_pc.printf("please reverse \n");
        wait_ms(2000);
        i =0 ;
        max =0 ;
        ////
        ::MAX = 0 ;
        do{
            a = ain1_1.read_u16();
            b = ain1_2.read_u16();  
            g_Serial_pc.printf("analog 1 :  %d \n", a);
            g_Serial_pc.printf("analog 2 :  %d \n", b);
            g_Serial_pc.printf("\n");  
            max_b =calibrate(float(b) , (float)a ) ;
            if (max_b > max )
            {
                max = max_b ;
                }
        if (max_b == 0){
        g_Serial_pc.printf("please maximize the potentiometer clockwise :%d \n" , i );}
        if (i>=10 && max!=0)
        {
            g_Serial_pc.printf("potentiometer deos not work well \n");
            g_Serial_pc.printf("max_b = %f \n" , max);
            break;
        }
        
        wait_ms(2000);
        i++ ;
        }while(max==0 || i<=5);
        max_b = max ;
       
        Write16(address,max_b) ;
        // end of input calibrate
        g_Serial_pc.printf("max  :%f   eeprom: %d \n" , max_b , (int)Read16(address));
        g_Serial_pc.printf("now calibrating sensor \n");
        wait_ms(2000);
        address+=8 ;
        // start of sensor calibrate
        
        /*
        i=0 ;
        ::MAX =0 ;
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
        */
        ////
        g_Serial_pc.printf("please reverse \n");
        wait_ms(2000);
        i =0 ;
        max =0 ;
        ////
        ::MAX = 0 ;
        do{
            a = asen1_1.read_u16();
            b = asen1_2.read_u16();  
            g_Serial_pc.printf("analog 1 :  %d \n", a);
            g_Serial_pc.printf("analog 2 :  %d \n", b);
            g_Serial_pc.printf("\n");  
            max_b_s =calibrate(float(b) , (float)a ) ;
            if (max_b_s > max )
            {
                max = max_b_s ;
                }
        if (max_b_s == 0){
        g_Serial_pc.printf("please maximize the potentiometer clockwise :%d \n" , i);  }
        if (i>=10 && max !=0)
        {
            g_Serial_pc.printf("potentiometer deos not work well \n");
            g_Serial_pc.printf("max_b_s = %f \n" , max_b_s);
            break;
        }
        wait_ms(2000);
        i++ ;
        }while(max==0 || i<=10);
        max_b_s = max ;
        Write16(address , max_b_s) ;
        g_Serial_pc.printf("max  :%f   eeprom: %d \n" , max_b_s ,(int)Read16(address));
        g_Serial_pc.printf("max  :%f \n" , max_b_s);
        g_Serial_pc.printf("calibrate finish \n" );
        address+=8 ;
            }
        else 
            {
            g_Serial_pc.printf("if you want to calibrate input please send 1 \n") ;
            }   
        
        
        }
        //                                                    struct_data_1( e ) ;
        
        if (e->handle == _feedback_setting.getValueHandle())
        {
            
        // done in setpoint 
        }
        if (e->handle == _control_setting.getValueHandle())
        {
         i =0 ;
         switch(e->data[0])
         {
             
             case 1: 
                g_Serial_pc.printf("max_a (input)= %d \n" ,Read16(4)) ; 
                g_Serial_pc.printf("max_b (input) = %d \n" ,Read16(12)) ;
                g_Serial_pc.printf("max_s_b(sensor) = %d \n" ,Read16(20)) ;
                g_Serial_pc.printf("dead_band = %f \n" ,((float)I2CEE3ReadInt8(28))/100.0f) ;
                g_Serial_pc.printf("P = %f \n" ,((float)I2CEE3ReadInt8(32))/100.0f) ;
                g_Serial_pc.printf("up_interblock = %f \n" ,((float)I2CEE3ReadInt8(40))/100.0f) ;
                
                g_Serial_pc.printf("down_interblock = %f \n" ,((float)I2CEE3ReadInt8(48))/100.0f) ;
                
                break ;
             case 2:
                 while(i<10){
                    a  = ain1_2.read_u16();  
                    b = asen1_2.read_u16();  
                    g_Serial_pc.printf("analog 1 :  %d \n", a);
                    g_Serial_pc.printf("analog 2 :  %d \n", b);
                    g_Serial_pc.printf("\n");  
                    i++ ;
                    wait(1) ;
                    }
             break ;
            case 26:
            
                g_Serial_pc.printf("Mohammad Ali Amiri - MANDAL -YektaFan \n");
                
                break;
            case 12:
                g_Serial_pc.printf("Mamnoonam Mr Parvizi - behtarin ostadi ke dashtam \n");
                break ; 
            case 0 :
                /////////////////////////////////////////////////////////////////////////////////////////////////
                
                int g =-1 ;
                int flag = 0 ;
                float max_a , max_b , max_b_s , dead_band_;
                float In_1 , In_2 , Se_1,Se_2 , Er;
                max_a = (float)Read16(4) ;
                max_b = (float)Read16(12) ;
                max_b_s =(float)Read16(20);
                dead_band_ = ((float)I2CEE3ReadInt8(28))/100.0f ;
                while(true)
                {
                g+=1 ;
                
                //goto    
                start_again :
                
                In_1 = (float)ain1_1.read_u16();
                In_1 = In_1 / max_a ;
                In_2 = (float)ain1_2.read_u16();  
                In_2 = In_2 / max_b ;
                //Se_1 = (float)asen1_1.read_u16();
                // Se_1 = Se_1 / max_a_s ;
                Se_2 = (float)asen1_2.read_u16();  
                Se_2 = Se_2 / max_b_s ;
                
                
                if(evaluate_sensor(In_1 , In_2) )
                {
                    g_Serial_pc.printf("there is problem in Input  \n");
                    wait_ms(2000);
                    goto start_again;
                    
                    }
                    /*
                if(evaluate_sensor(Se_1 , Se_2) )
                {
                    g_Serial_pc.printf("there is problem in sensor  \n");
                    thread_sleep_for(2000);
                    goto start_again;
                    }
                    */
                if(Interblock(0.5f , Se_2 )) //// Se_1 does not work 
                {
                    g_Serial_pc.printf("out of range \n");
                    float up_dead_band , down_dead_band ;
                    
                    up_dead_band = ((float)I2CEE3ReadInt8(40))/100.0f ;
                    down_dead_band = ((float)I2CEE3ReadInt8(48))/100.0f ;
                    if (Se_2 >= up_dead_band)
                    {
                        flag = 1 ;
                        }
                    if (Se_2 <= down_dead_band)
                    {
                        flag = -1 ;
                        }
                    wait_ms(2000);
                    //goto start_again;
                    }
                else
                {
                    flag = 0 ;
                    }
                
                Er = In_2 - Se_2 ;   // Se_1 kharab   
                if (Er > dead_band_ && flag ==0 ) 
                {
                    clockwise_motor(Er);
                    }
                if (Er < -dead_band_ && flag ==0  )
                {
                    counterclockwise_motor(Er);
                    
                    }
                if (Er<=dead_band_ && Er>= -dead_band_ )
                {
                    stable() ;
                    }
                    
                if(Er > dead_band_ && flag == -1)
                {
                    clockwise_motor(Er);
                    }  
                if(Er < -dead_band_ && flag == 1)
                {
                    counterclockwise_motor(Er);
                    } 
                    
                
                if(g%1000 == 0)
                {
                   g_Serial_pc.printf("input = %f , sensor = %f ,Error = %f \n" , In_2 ,Se_2, Er);
                    }
                //wait_ms(1);
                
                }
                break ;
            //default :
                 //g_Serial_pc.printf("Invalid \n");
                 
            } 
        }
        
        if (e->handle == _deadband.getValueHandle())
        {
            if(e->data[0] <0 || e->data[0]>50)
            {
                g_Serial_pc.printf("incorrect dead_band \n") ;   
                }
            else
            {
            ::dead_band = e->data[0] ;
            I2CEE3WriteByte(28 , ::dead_band) ;
            g_Serial_pc.printf("dead_band set on %f and eeprom is: %f \n" , ((float)::dead_band)/100.0f ,((float)I2CEE3ReadInt8(28))/100.0f ) ;   
            }
            
         }
         
         
         if (e->handle == _Kp.getValueHandle())
         {
             if(e->data[0] <0 || e->data[0]>=200)
            {
                g_Serial_pc.printf("incorrect kp \n") ;   
                }
            else
            {
            ::P = e->data[0] ;
            I2CEE3WriteByte(32 , ::P) ;
            g_Serial_pc.printf("Kp set on %f and eeprom is: %f \n" , ((float)::P)/100.0f ,((float)I2CEE3ReadInt8(32))/100.0f ) ;  
             
             }
             }
             
        if(e->handle == _Ki.getValueHandle())              // Dead_band_interblock_up        /1000
        {
             if(e->data[0] <0 || e->data[0]>=255)
            {
                g_Serial_pc.printf("incorrect Dead_band_interblock_up \n") ;   
                }
            else
            {
            Data = e->data[0] ;
            I2CEE3WriteByte(40 , Data) ;
            g_Serial_pc.printf("interblock dead band up set on %f and eeprom is: %f \n" , ((float)Data)/100.0f ,((float)I2CEE3ReadInt8(40))/100.0f ) ;  
             
             }
            
            }
            
         if(e->handle == _Kd.getValueHandle())    // Dead_band_interblock_down        /1000
         {
             if(e->data[0] <0 || e->data[0]>=200)
            {
                g_Serial_pc.printf("incorrect Dead_band_interblock_down \n") ;   
                }
            else
            {
            Data = e->data[0] ;
            I2CEE3WriteByte(48 , Data) ;
            g_Serial_pc.printf("interblock dead band down set on %f and eeprom is: %f \n" , ((float)Data)/100.0f ,((float)I2CEE3ReadInt8(48))/100.0f ) ;  
             
             }
             }
        /*
        printf("data written:\r\n");
        printf("\tconnection handle: %u\r\n", e->connHandle);
        printf("\tattribute handle: %u", e->handle);
        if (e->handle == _setpoint_setting.getValueHandle()) {
            printf(" (hour characteristic)\r\n");
        } else if (e->handle == _feedback_setting.getValueHandle()) {
            printf(" (minute characteristic)\r\n");
        } else if (e->handle == _control_setting.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else if (e->handle == _deadband.getValueHandle()) {
            printf(" (second characteristic)\r\n");  
        } else if (e->handle == _Kp.getValueHandle()) {
            printf(" (second characteristic)\r\n"); 
        } else if (e->handle == _Ki.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else if (e->handle == _Kd.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else if (e->handle == _sensitivity.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else {
            printf("\r\n");
        }
        
       

        printf("\r\n");   
         */
    }

    /**
     * Handler called after an attribute has been read.
     */
    void when_data_read(const GattReadCallbackParams *e)
    {
        
        printf("data read:\r\n");
        printf("\tconnection handle: %u\r\n", e->connHandle);
        printf("\tattribute handle: %u", e->handle);
        if (e->handle == _control_setting.getValueHandle()) {
            printf(" (hour characteristic)\r\n");
        } else if (e->handle == _feedback_setting.getValueHandle()) {
            printf(" (minute characteristic)\r\n");
        } else if (e->handle == _setpoint_setting.getValueHandle()) {
            printf(" (second characteristic)\r\n");
        } else {
            printf("\r\n");
       }
    }



     /* Handler called after a client has subscribed to notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void when_update_enabled(GattAttribute::Handle_t handle)
    {
        printf("update enabled on handle %d\r\n", handle);
    }

    /**
     * Handler called after a client has cancelled his subscription from
     * notification or indication.
     *
     * @param handle Handle of the characteristic value affected by the change.
     */
    void when_update_disabled(GattAttribute::Handle_t handle)
    {
        printf("update disabled on handle %d\r\n", handle);
    }

    /**
     * Handler called when an indication confirmation has been received.
     *
     * @param handle Handle of the characteristic value that has emitted the
     * indication.
     */
    void when_confirmation_received(GattAttribute::Handle_t handle)
    {
        printf("confirmation received on handle %d\r\n", handle);
    }

    /**
     * Handler called when a write request is received.
     *
     * This handler verify that the value submitted by the client is valid before
     * authorizing the operation.
     */
    void authorize_client_write(GattWriteAuthCallbackParams *e)
    {
        printf("characteristic %u write authorization\r\n", e->handle);

        if (e->offset != 0) {
            printf("Error invalid offset\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_OFFSET;
            return;
        }

        if (e->len != 1) {
            printf("Error invalid len\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_INVALID_ATT_VAL_LENGTH;
            return;
        }
/*
        if ((e->data[0] >= 60) ||
            ((e->data[0] >= 24) && (e->handle == _control_setting.getValueHandle()))) {
            printf("Error invalid data\r\n");
            e->authorizationReply = AUTH_CALLBACK_REPLY_ATTERR_WRITE_NOT_PERMITTED;
            return;
        }
*/     
        e->authorizationReply = AUTH_CALLBACK_REPLY_SUCCESS;
    }

    /**
     * Increment the second counter.
     */

    /**
     * Increment the minute counter.
     */
    void increment_minute(void)
    {
        uint8_t minute = 0;
        ble_error_t err = _feedback_setting.get(*_server, minute);
        if (err) {
            printf("read of the minute value returned error %u\r\n", err);
            return;
        }

        minute = (minute + 1) % 60;

        err = _feedback_setting.set(*_server, minute);
        if (err) {
            printf("write of the minute value returned error %u\r\n", err);
            return;
        }

        if (minute == 0) {
            increment_hour();
        }
    }

    /**
     * Increment the hour counter.
     */
    void increment_hour(void)
    {
        uint8_t hour = 0;
        ble_error_t err = _control_setting.get(*_server, hour);
        if (err) {
            printf("read of the hour value returned error %u\r\n", err);
            return;
        }

        hour = (hour + 1) % 24;

        err = _control_setting.set(*_server, hour);
        if (err) {
            printf("write of the hour value returned error %u\r\n", err);
            return;
        }
    }

private:
    /**
     * Helper that construct an event handler from a member function of this
     * instance.
     */
    template<typename Arg>
    FunctionPointerWithContext<Arg> as_cb(void (Self::*member)(Arg))
    {
        return makeFunctionPointer(this, member);
    }

    /**
     * Read, Write, Notify, Indicate  Characteristic declaration helper.
     *
     * @tparam T type of data held by the characteristic.
     */
    template<typename T>
    class ReadWriteNotifyIndicateCharacteristic : public GattCharacteristic {
    public:
        /**
         * Construct a characteristic that can be read or written and emit
         * notification or indication.
         *
         * @param[in] uuid The UUID of the characteristic.
         * @param[in] initial_value Initial value contained by the characteristic.
         */
        ReadWriteNotifyIndicateCharacteristic(const UUID & uuid, const T& initial_value) :
            GattCharacteristic(
                /* UUID */ uuid,
                /* Initial value */ &_value,
                /* Value size */ sizeof(_value),
                /* Value capacity */ sizeof(_value),
                /* Properties */ GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_READ |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_WRITE |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_NOTIFY |
                                GattCharacteristic::BLE_GATT_CHAR_PROPERTIES_INDICATE,
                /* Descriptors */ NULL,
                /* Num descriptors */ 0,
                /* variable len */ false
            ),
            _value(initial_value) {
        }

        /**
         * Get the value of this characteristic.
         *
         * @param[in] server GattServer instance that contain the characteristic
         * value.
         * @param[in] dst Variable that will receive the characteristic value.
         *
         * @return BLE_ERROR_NONE in case of success or an appropriate error code.
         */
        ble_error_t get(GattServer &server, T& dst) const
        {
            uint16_t value_length = sizeof(dst);
            return server.read(getValueHandle(), &dst, &value_length);
        }

        /**
         * Assign a new value to this characteristic.
         *
         * @param[in] server GattServer instance that will receive the new value.
         * @param[in] value The new value to set.
         * @param[in] local_only Flag that determine if the change should be kept
         * locally or forwarded to subscribed clients.
         */
        ble_error_t set(
            GattServer &server, const uint8_t &value, bool local_only = false
        ) const {
            return server.write(getValueHandle(), &value, sizeof(value), local_only);
        }

    private:
        uint8_t _value;
    };
        
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _setpoint_setting;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _feedback_setting;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _control_setting;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _deadband;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _Kp;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _Ki;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _Kd;
    ReadWriteNotifyIndicateCharacteristic<uint8_t> _sensitivity;

    // list of the characteristics of the clock service
    GattCharacteristic* _EPS_characteristics[8];

    // demo service
    GattService _EPS_service;

    GattServer* _server;
    events::EventQueue *_event_queue;
};

int main() {
    BLE &ble_interface = BLE::Instance();
    events::EventQueue event_queue;
    EPSService demo_service;
    BLEProcess ble_process(event_queue, ble_interface);

    ble_process.on_init(callback(&demo_service, &EPSService::start));

    // bind the event queue to the ble interface, initialize the interface
    // and start advertising
    ble_process.start();

    // Process the event queue.
    event_queue.dispatch_forever();

    return 0;  
}
