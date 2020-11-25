#include "dubins.h"
#include <iostream>
using namespace std;
struct base
{
  int x;
  int y;
};
class dubins
{
public:
  void get();

private:
  static int printConfiguration(double q[3], double x, void* user_data);
};

int dubins::printConfiguration(double q[3], double x, void* user_data)
{
  struct base* some_data = static_cast<struct base*>(user_data);
  some_data->x = 5;
  printf("%f, %f, %f, %f\n", q[0], q[1], q[2], x);
  return 0;
}

void dubins::get()
{
  double q0[] = { 0, 0, 0 };
  double q1[] = { 4, 4, 3.142 };
  double turning_radius = 1.0;
  DubinsPath path;
  cout << "Hello";
  dubins_shortest_path(&path, q0, q1, turning_radius);
  struct base some_data;
  dubins_path_sample_many(&path, 0.1, printConfiguration, static_cast<void*>(&some_data));
  printf("WOW\n");
  cout << some_data.x;
}

int main()
{
  dubins abc;
  abc.get();
  return 0;
  // x.insert(x.end(), y.begin(), y.end());
}