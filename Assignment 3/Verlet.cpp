
#include <iostream>
#include <fstream>
#include <vector>
#include <cstring>

int main(int argc, char *argv[]) {
  clock_t start,end;
  start = clock();
  double m, k, x, v, t_max, dt, t, a;

  //Set Default Values
  m = 1;
  k = 1;
  x = 0;
  v = 1;

  //Simulation time and timestep
  t_max = 100;
  dt = 0.001;


  for (int i = 1; i < argc; i++)
  {
    if (i%2 == 1){
        if (strcmp(argv[1], "--help")==0)
        {
          std::cerr << "VERLET.EXE HELPER" << std::endl << std::endl;
          std::cerr << "euler [options]\n";
          std::cerr << "Run a numerical simulation of a mass-spring system using Euler approximation\n";
          std::cerr << "OPTIONS:" << std::endl;
          std::cerr <<"-m      mass\n" << "-k      spring constant\n" << "-x      initial displacement\n" << "-v      initial velocity\n"
            << "-t      timestep\n"     <<"-T      max time" << std::endl << std::endl;
          std::cerr << "DEFAULTS:\nm=1\nx=0\nv=1\nt_max=100\ndt=0.001\n";
          return 0;
        }

        if (strcmp(argv[i], "-m")==0)
        {
          m=atof(argv[i+1]);
        }
        else if (strcmp(argv[i], "-k")==0)
        {
          k=atof(argv[i+1]);
        }
        else if (strcmp(argv[i], "-x")==0)
        {
          x=atof(argv[i+1]);
        }
        else if (strcmp(argv[i], "-v")==0)
        {
          v=atof(argv[i+1]);
        }
        else if (strcmp(argv[i], "-t")==0)
        {
          
          dt=atof(argv[i+1]);
        }
        else if (strcmp(argv[i], "-T")==0)
        {
          t_max=atof(argv[i+1]);
        }
    }
  }

  // declare variables
  std::vector<double> t_list, x_list, v_list;

//Prefil the t vector. This is for simplicity and should not add any computation time. 
  for (t=0; t<t_max; t+=dt){
    t_list.push_back(t);
  }

//Resize x and v list to be of the same size as the t_list
x_list.resize(t_list.size());
v_list.resize(t_list.size());
  // Euler integration
  for (int i=0; i <= t_list.size(); i++){
    if (i==0)
    {
      //Initial Conditions
      x_list.push_back(x);
      v_list.push_back(v);

      //Apply Euler to find the first case 
      a = -k * x_list[i] / m;
      x_list.push_back(x_list[0]+dt*v_list[i]);

    } else if (i+1 <= t_list.size()) {
      //Verlet Algorithm
      a = -k * x_list[i] / m;
      x_list[i+1] = 2*x_list[i]-x_list[i-1]+dt*dt* a;
      v_list[i] = (x_list[i+1] - x_list[i-1]) / (2*dt);
    } else {
      //Estimate last velocity measurement using first order approximation
      v_list[i] = (x_list[i]-x_list[i-1])/dt;
    }
  }


  // Write the trajectories to file
  /*std::ofstream fout;
  fout.open("verlet trajectories.txt");
  if (fout) { // file opened successfully
    for (int i = 0; i < t_list.size(); i++) {
      fout << t_list[i] << ' ' << x_list[i] << ' ' << v_list[i] << std::endl;
    }
  } else { // file did not open successfully
    std::cout << "Could not open trajectory file for writing" << std::endl;
  }*/

  end = clock();
  std::cout << "Time: " << double(end-start)/double(CLOCKS_PER_SEC) << std::endl;
}
