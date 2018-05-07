//---------------------------------------------------------------------------

#pragma hdrstop

#include "algorithm.h"

//---------------------------------------------------------------------------

Tsearch::Tsearch()
{
}

void Tsearch::reset()
{
}

void Tsearch::process()
{
}

void Tsearch::add_data(int data)
{
    process();
}

void Talg_data::reset()
{
    step = 20;
    search_max = 1;
    max = -100;
    min = 100;
    limit = 100;
    LR = _Unknown;

    interval = 0;

    X_alfa = 0;
    Y_alfa = 0;
    Z_alfa = 0;

    X_speed = 0;
    Y_speed = 0;
    Z_speed = 0;

    X_moove = 0;
    Y_moove = 0;
    Z_moove = 0;

    Gx = 0;
    Gy = 0;
    Gz = 0;
    Ax = 0;
    Ay = 0;
    Az = 0;

    A = 0;
    G = 0;

    AlgType = 0;
    ZG_SRCH = 0;
    AzM8 = 0;
    turn_ON = 0;

    AM = 0;
    GM = 0;
    AYM = 0;
    AXM = 0;
}

Talg_data::Talg_data()
{

    reset();
}

Talg_data::Talg_data(LR_type _LR)
{
    Talg_data();
    LR = _LR;
}

void Talg_data::Process()
{
// if
//   if (LR==_Unknown) return;

/* integral */
#define angle_error 0.99

/*
   X_alfa = angle_error*X_alfa + Gx;
   Y_alfa = angle_error*Y_alfa + Gy;
   Z_alfa = angle_error*Z_alfa + Gz;
    */

/* integral */
#define speed_error 0.95

    //X_speed = speed_error*X_speed + Ax; //speed
    Y_speed = speed_error * Y_speed + Ay;
    //Z_speed = speed_error*Z_speed + Az;

    G = (float)sqrt((float)Gx * Gx + Gy * Gy + Gz * Gz);

    /*
   #define moove_error 0.95
   #define moove_K 10


   X_moove = moove_error*X_moove + X_speed/moove_K; //speed
   Y_moove = moove_error*Y_moove + Y_speed/moove_K;
   Z_moove = moove_error*Z_moove + Z_speed/moove_K;

   
   GM = GM*KST_G+(1-KST_G)*G;

   ZG_SRCH = ZG_SRCH*k_zg_avg + (1-k_zg_avg)*Gz;

   AzM8 = AzM8*0.8 + Az*0.2;

   AYM = AYM*KST_A2 + Ay * (1-KST_A2);
   AXM = AXM*KST_A2 + Ax * (1-KST_A2);

   A = sqrt ((float)Ax*Ax + Ay*Ay + Az*Az);
   AM = AM*KST_A+(1-KST_A)*A;
   */
    //if ((Y_speed<-200) && (interval>500)) {
    if ((Y_speed < -400) && (interval > 100) && (G > 40)) {
        interval = 0;
        turn_ON = 1;
    }
    else
        turn_ON = 0;
    /*
   if (LR==_Right)
   {

        if (search_max)
            {

                if (max<AYM-AXM)
                    max = AYM-AXM;
                else
                    if (max>AYM-AXM+step)
                    {
                         search_max=0;
                        min = max - step;
                        limit = max - step*2;
                    };
            } else {
                if (limit>AYM-AXM)
                    turn_ON=1; else turn_ON=0; 
                if (min>AYM-AXM)
                    min = AYM-AXM;
                else
                    if (min<AYM-AXM-step)
                    {
                        search_max=1;
                        max = min + step;
                        limit = min + step*2;
                    };
            }



   }//LR==RIght
   else
   if (LR==_Left)
   {
    
        if (search_max)
            {

                if (max<AYM+AXM)
                    max = AYM+AXM;
                else
                    if (max>AYM+AXM+step)
                    {
                        search_max=0;
                        min = max - step;
                        limit = max - step*2;
                    };
            } else {
                if (limit>AYM+AXM)
                    turn_ON=1; else turn_ON=0; 
                if (min>AYM+AXM)
                    min = AYM+AXM;
                else
                    if (min<AYM+AXM-step)
                    {
                        search_max=1;
                        max = min + step;
                        limit = min + step*2;
                    };
            }

   }*/
}

void Talg_data::AddData(signed char _Gx, signed char _Gy, signed char _Gz, signed char _Ax, signed char _Ay, signed char _Az)
{
    interval++;

    Gx = _Gx;
    Gy = _Gy;
    Gz = _Gz;

    Ax = _Ax;
    Ay = _Ay + 50;
    Az = _Az;
}

#pragma package(smart_init)
