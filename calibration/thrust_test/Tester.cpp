#include <algorithm>

const int n_thrusts = 20;
const double max_thrust = 1;
const double thrust_step = max_thrust/n_thrusts;

const int iterations_per_phase = 50;

class Tester {
public:
    int phase_step = 0;
    int curr_thrust_index = 0;
    bool done = false;
    int direction = 1;

    double get_next_thrust(){
        if(done) return 0;

        phase_step++;
        if(phase_step >= iterations_per_phase){
            phase_step = 0;
            curr_thrust_index += direction;

            if (curr_thrust_index > n_thrusts){
                curr_thrust_index = n_thrusts;
                direction = -1;
            }

            if (curr_thrust_index < 0){
                done = true;
                return 0;
            }
        }

      return curr_thrust_index * thrust_step;
    }

    void reset(){
        done = false;
        phase_step = 0;
        curr_thrust_index = 0;
    }
};