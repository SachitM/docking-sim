/*********************************************************************
* Author:  Sachit Mahajan
*********************************************************************/
#include "reeds_shepp.h"
#include <iostream>
using namespace std;
struct base{
    int x;
    int y;
};
    class reed_shepps {
        public:
        void get() ;
        private:
    static int printConfiguration(double q[3], void* user_data);

};

int reed_shepps::printConfiguration(double q[3], void* user_data) {
    struct base* some_data = static_cast<struct base*>(user_data);
    some_data->x = 5;
        printf("%f, %f, %f\n", q[0], q[1], q[2]);
    return 0;
}


void reed_shepps::get() {
        double q0[] = { 0,0,0 };
        double q1[] = { 4,4,3.142 };
        double turning_radius = 1.0;


        ReedsSheppStateSpace path(turning_radius);
        cout<<"Hello";
        struct base some_data;
        path.sample( q0, q1, 0.1, printConfiguration, static_cast<void*>(&some_data));
        printf("WOW\n" );
        cout<<some_data.x;
    }

int main()
{
    reed_shepps abc;
    abc.get();
    return 0;
    // x.insert(x.end(), y.begin(), y.end());
}