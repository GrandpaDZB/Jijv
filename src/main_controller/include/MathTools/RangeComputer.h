#include <PSO.h>
#include <common_parameters.h>
class RangeComputer
{
public:
    float** T_set[6];
    float** C_set[6];
    float L_min, L_max;
    PSO model_PSO = PSO(Vector3f(0,0,0), Vector3f(0,0,0), 0);
    vector<Vector2f> coordi_C, coordi_T;

    void set_mechanical_params(Vector2f* coordi_C, Vector2f* coordi_T);
    void run(int iteration, float theta);
    float compute_cost(float delta_f, float theta);
    RangeComputer(float min_delta_f, float max_delta_f, int particle_num, float L_min, float L_max);
    ~RangeComputer();
};

/**
 * @description: 
 * @param {float} delta_f
 * @param {float} theta
 * @return {*}
 */
float RangeComputer::compute_cost(float delta_f, float theta){
    // compute unit direction vector
    Vector2f unit_diret = Vector2f(cos(theta), sin(theta));
    // compute costs and pick up the max & min one
    float costs[6], max_cost, min_cost;
    for(int i = 0; i < 6; i++){
        costs[i] = (this->coordi_T[i]+delta_f*unit_diret-this->coordi_C[i]).norm() ;
        
        if(i == 0){
            max_cost = costs[0];
            min_cost = costs[0];
        }else{
            if(costs[i] > max_cost){ max_cost = costs[i]; }
            if(costs[i] < min_cost){ min_cost = costs[i]; }
        }
    }
    return abs(max_cost - this->L_max)+abs(min_cost - this->L_min);
}

/**
 * @description: 
 * @param {int} iteration
 * @param {float} theta
 * @return {*}
 */
void RangeComputer::run(int iteration, float theta){
    // first iter
    vector<Vector3f> particle = this->model_PSO.local_best_params;
    vector<float> costs;
    for(int i = 0; i < this->model_PSO.particle_num; i++){
        costs.push_back(this->compute_cost(particle[i][0], theta));
    }
    particle = this->model_PSO.run(costs, particle);
    // other iter
    for(int iter = 0; iter < iteration-1; iter++){
        vector<float> costs;
        for(int i = 0; i < this->model_PSO.particle_num; i++){
            costs.push_back(this->compute_cost(particle[i][0], theta));
        }
        particle = this->model_PSO.run(costs, particle);
        std::cout << "Iteration: " << iter+2 << "\t Global best delta_f: " << this->model_PSO.global_best_params[0] << std::endl;
    }
    return;
}

/**
 * @description: 
 * @param {float} min_delta_f
 * @param {float} max_delta_f
 * @param {int} particle_num
 * @param {float} L_min
 * @param {float} L_max
 * @return {*}
 */
RangeComputer::RangeComputer(float min_delta_f, float max_delta_f, int particle_num, float L_min, float L_max){
    this->model_PSO = PSO(Vector3f(min_delta_f,-1,-1), Vector3f(max_delta_f,1,1), particle_num);
    this->L_min = L_min;
    this->L_max = L_max;
    coordi_C.push_back(Tail::C1);
    coordi_C.push_back(Tail::C2);
    coordi_C.push_back(Tail::C3);
    coordi_C.push_back(Tail::C4);
    coordi_C.push_back(Tail::C5);
    coordi_C.push_back(Tail::C6);
    coordi_C.push_back(Tail::T1);
    coordi_C.push_back(Tail::T2);
    coordi_C.push_back(Tail::T3);
    coordi_C.push_back(Tail::T4);
    coordi_C.push_back(Tail::T5);
    coordi_C.push_back(Tail::T6);
    
}

RangeComputer::~RangeComputer()
{
}
