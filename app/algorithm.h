//---------------------------------------------------------------------------

#ifndef algorithmH
#define algorithmH

#include <math.h>

#define KST_A 0.98
#define KST_A2 0.98
#define KST_G 0.98
#define ms_K 5

#define k_zg_avg 0.95

#define K_res 0.9

enum LR_type { _Unknown,
    _Left,
    _Right };

class Tsearch {
public:
    int max;
    int min;
    int search_max;
    int period; //period za posleznii cykl
    int trigger; //turn_ON
    void reset();
    void add_data(int data);
    void process();
    Tsearch();
};

class Talg_data {
public:
    int interval; //interval, more than 600 needs

    //current data
    int Gx, Gy, Gz;
    int Ax, Ay, Az;

    int X_alfa, Y_alfa, Z_alfa;
    int X_speed, Y_speed, Z_speed;
    int X_moove, Y_moove, Z_moove;

    char AlgType;
    //0 - algorithm gyro dec.13,
    //1  -algorithm acc dec.13,
    //2 - alg gyro+acc jan.14

    int ZG_SRCH;
    int AzM8;

    LR_type LR; // left or not
    int search_max;
    int min;
    int max;
    int limit;
    int step;
    int turn_ON; //activate the electric shock

    float AM, GM;
    float AYM, AXM;
    int A, G; //sqrt(a1**2+...+a3**2)
    Talg_data(LR_type _LR);
    Talg_data();
    void reset();
    void Process();
    void AddData(signed char _Gx, signed char _Gy, signed char _Gz, signed char _Ax, signed char _Ay, signed char _Az);
};

//---------------------------------------------------------------------------
#endif
