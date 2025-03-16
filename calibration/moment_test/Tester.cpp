#include <algorithm>

const int n_diffs = 20;
const double max_diff = 0.2;
const double diff_step = max_diff/n_diffs;

const int iterations_per_phase = 100;

const double p = 0.002;

const double max_stabl_diff = 0.2;

class Tester {
public:
    bool stabilising = true;
    int phase_step = 0;
    int curr_diff_index = 0;

    double get_next_diff(double z_rate){
        phase_step++;
        if(phase_step >= iterations_per_phase){
            phase_step = 0;

            if(!stabilising){
                stabilising = true;
                curr_diff_index = next_index(curr_diff_index);
            }
            else if(curr_diff_index <= n_diffs){
                // stay in stabilising after n_diffs
                stabilising = false;
            }
        }
        
        if(stabilising){
            return std::clamp(p * z_rate, -max_diff, max_diff);
        } else {
            return curr_diff_index * diff_step;
        }
    }

    void reset(){
        stabilising = true;
        phase_step = 0;
        curr_diff_index = 0;
    }

private:
    int next_index(int i){
        if (i > 0) {
            return -i;
        } else {
            return -i + 1;
        }
    }
};