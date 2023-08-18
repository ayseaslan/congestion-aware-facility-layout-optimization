#include <cstdio>
#include <fstream>
#include <vector>
#include <sstream>
#include "gurobi_c++.h"


using namespace std;

int main() {

    //parameters
    int A = 7; // number of facilities
    //dimensions of the 2D floor 
    int L = 5; 
    int H = 8; 
  
    // facilities are rectangular objects 
    vector<double>long_sides = { 1.0, 1.0, 1.0, 1.5, 1.0, 1.5, 2.0 }; // length of the long sides of facilities
    vector<double>short_sides = { 0.7, 0.7, 0.7, 1.0, 0.7, 1.0, 0.7 }; // length of the short sides of facilities
  
    double delta = 0.499; // set the safety/clearance distance to be maintained between facilities

   // weight parameters in the objective
    double w_distance = 1.0; // the weight given to minimizing the total distance travelled
    double w_congestion_risk = 0.1; // the weight given to minimizing the congestion risk 
   
    // read the flow frequencies between facilities 
    vector<vector<int>>flow_between_activities; 
    stringstream name;
    name << "flow_frequency_scenarios.csv"; // read frequencies from data file 
    string na = name.str();
    ifstream infile(na.c_str());
    int T = 0; // the number of scenarios 

    while (infile)
    {
        string s;
        if (!getline(infile, s))
            break;
        istringstream ss(s);
        vector<int>v;
        while (ss)
        {
            string s;
            if (!getline(ss, s, ',')) break;
            int f = stoi(s);
            v.push_back(f);
        }
        flow_between_activities.push_back(v);
    }

    T = (int)flow_between_activities.size(); 
  
    // LP formulation starts here 

    double bigM = 10000000.0; // use to impose if constraints 

    // start to declare the LP MODEL 
    GRBEnv* env = 0;

    // declare variables here;
    GRBVar** distance = 0;  // the rectilinear distance between two facilities 

    GRBVar** distance_x = 0; // distance between two facilites in the x dimension 

    GRBVar** distance_y = 0; // distance between two facilites in the y dimension

    GRBVar** pos_left = 0;

    GRBVar** pos_below = 0;

    GRBVar*** pos_pair_left = 0;

    GRBVar*** pos_pair_below = 0;

    GRBVar*** pos_pair_right = 0;

    GRBVar*** pos_pair_up = 0;

    GRBVar* center_x = 0; // the x coordinate of the center of a facility  
    GRBVar* center_y = 0; // the y coordinate of the center of a facility
    GRBVar* length_x = 0;
    GRBVar* length_y = 0;
    GRBVar* is_longside_on_x = 0; // to understand the orientation of the facilities, which determine their length in the x and y dimensions  

    

    // Model
    env = new GRBEnv();
    GRBModel model = GRBModel(*env);
    model.set(GRB_StringAttr_ModelName, "GUR_TEST");

    // ADD the VARIABLES
    distance = new GRBVar * [A];
    distance_x = new GRBVar * [A];
    distance_y = new GRBVar * [A];
    pos_left = new GRBVar * [A];
    pos_below = new GRBVar * [A];

    pos_pair_left = new GRBVar * *[A];
    pos_pair_right = new GRBVar * *[A];
    pos_pair_below = new GRBVar * *[A];
    pos_pair_up = new GRBVar * *[A];

    center_x = new GRBVar[A];
    center_y = new GRBVar[A];
    length_x = new GRBVar[A];
    length_y = new GRBVar[A];
    is_longside_on_x = new GRBVar[A];

   

    for (int i = 0; i < A; i++) {
        distance[i] = new GRBVar[A];
        distance_x[i] = new GRBVar[A];
        distance_y[i] = new GRBVar[A];
        pos_left[i] = new GRBVar[A];
        pos_below[i] = new GRBVar[A];

        pos_pair_left[i] = new GRBVar * [A];
        pos_pair_right[i] = new GRBVar * [A];
        pos_pair_below[i] = new GRBVar * [A];
        pos_pair_up[i] = new GRBVar * [A];

        center_x[i] = model.addVar(0, H, 0, GRB_CONTINUOUS);
        center_y[i] = model.addVar(0, H, 0, GRB_CONTINUOUS);
        length_x[i] = model.addVar(short_sides[i], long_sides[i], 0, GRB_CONTINUOUS);
        length_y[i] = model.addVar(short_sides[i], long_sides[i], 0, GRB_CONTINUOUS);
        is_longside_on_x[i] = model.addVar(0.0, 1.0, 0, GRB_BINARY);

       

        for (int j = 0; j < A; j++) {
            distance[i][j] = model.addVar(0, L + H, 0, GRB_CONTINUOUS);
            distance_x[i][j] = model.addVar(0, L, 0, GRB_CONTINUOUS);
            distance_y[i][j] = model.addVar(0, H, 0, GRB_CONTINUOUS);
            pos_left[i][j] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
            pos_below[i][j] = model.addVar(0.0, 1.0, 0, GRB_BINARY);

            pos_pair_left[i][j] = new GRBVar[A];
            pos_pair_right[i][j] = new GRBVar[A];
            pos_pair_below[i][j] = new GRBVar[A];
            pos_pair_up[i][j] = new GRBVar[A];

            for (int k = 0; k < A; k++) {
                pos_pair_left[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
                pos_pair_right[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
                pos_pair_below[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
                pos_pair_up[i][j][k] = model.addVar(0.0, 1.0, 0, GRB_BINARY);
            }

        }
    }

    // add constraints 
    for (int i = 0; i < A; i++) {
        model.addConstr(length_x[i] == long_sides[i] * is_longside_on_x[i] + short_sides[i] * (1.0 - is_longside_on_x[i]));
        model.addConstr(length_y[i] == long_sides[i] * (1.0 - is_longside_on_x[i]) + short_sides[i] * is_longside_on_x[i]);

        model.addConstr(center_x[i] - length_x[i] * 0.5 >= 0);
        model.addConstr(center_y[i] - length_y[i] * 0.5 >= 0);
        model.addConstr(center_x[i] + length_x[i] * 0.5 <= L);
        model.addConstr(center_y[i] + length_y[i] * 0.5 <= H);

       
       

        for (int j = 0; j < A; j++) {
            model.addConstr(distance[i][j] >= distance_x[i][j] + distance_y[i][j]);
            model.addConstr(distance_x[i][j] >= center_x[i] - center_x[j]);
            model.addConstr(distance_x[i][j] >= center_x[j] - center_x[i]);
            model.addConstr(distance_y[i][j] >= center_y[i] - center_y[j]);
            model.addConstr(distance_y[i][j] >= center_y[j] - center_y[i]);

            if (i != j) {
                model.addConstr(center_x[i] + 0.5 * length_x[i] + delta <= center_x[j] - 0.5 * length_x[j] + bigM * (1 - pos_left[i][j]));
                model.addConstr(center_y[i] + 0.5 * length_y[i] + delta <= center_y[j] - 0.5 * length_y[j] + bigM * (1 - pos_below[i][j]));
                if (i > j) {
                    model.addConstr(pos_left[i][j] + pos_left[j][i] + pos_below[i][j] + pos_below[j][i] == 1);
                }

            }
            for (int k = 0; k < A; k++) {
                if (j != i && k != i && j != k) {
                    model.addConstr(pos_pair_left[j][k][i] >= pos_left[j][i] + pos_left[k][i] - 1.0);
                    model.addConstr(pos_pair_right[j][k][i] >= pos_left[i][j] + pos_left[i][k] - 1.0);
                    model.addConstr(pos_pair_below[j][k][i] >= pos_below[j][i] + pos_below[k][i] - 1.0);
                    model.addConstr(pos_pair_up[j][k][i] >= pos_below[i][j] + pos_below[i][k] - 1.0);
                }
            }

        }

    }


    // objective function
    GRBLinExpr obj = 0;
    // the first objective: the distances 
    for (int t = 0; t < T; t++) {
        int n = 0;
        for (int i = 0; i < A; i++) {
            for (int j = 0; j < A; j++) {

                obj += w_distance * flow_between_activities[t][n] * distance[i][j];
                n++;
            }
        }
    }

   
    // the second objective: the congestion risk measured from the relative positioning of the facilities 
    for (int t = 0; t < T; t++) {
        int n = 0;

        for (int i = 0; i < A; i++) {

            for (int j = 0; j < A; j++) {

                for (int k = 0; k < A; k++) {

                    int n1 = 0;
                    for (int k1 = 0; k1 < A; k1++) {
                        for (int j1 = 0; j1 < A; j1++) {

                            if (k1 == k && j1 == j) {
                                break;
                            }
                            else {
                                n1++;
                            }

                        }
                    }


                    if (j != i && k > i) {
                        obj += w_congestion_risk * flow_between_activities[t][n] * (flow_between_activities[t][n1] * (pos_pair_left[i][k][j] + pos_pair_right[i][k][j] + pos_pair_below[i][k][j] + pos_pair_up[i][k][j]));

                    }
                }
                n++;
            }
        }
    }

    model.setObjective(obj, GRB_MINIMIZE); // MAXIMIZE PROFITS COLLECTED 
    //model.getEnv().set(GRB_DoubleParam_TimeLimit, 300.0);   
    model.getEnv().set(GRB_IntParam_OutputFlag, 1); 
    model.optimize();

    // write the layout, the coordinates of the facilities 
    FILE* pFile;
    stringstream nameSum;
    nameSum << "layout.txt";
    fopen_s(&pFile, nameSum.str().c_str(), "w");
   
 

    for (int i = 0; i < A; i++) {

        double low_corn_x = center_x[i].get(GRB_DoubleAttr_X) - 0.5 * length_x[i].get(GRB_DoubleAttr_X);
       

        double low_corn_y = center_y[i].get(GRB_DoubleAttr_X) - 0.5 * length_y[i].get(GRB_DoubleAttr_X);
        

        double upp_corn_x = center_x[i].get(GRB_DoubleAttr_X) + 0.5 * length_x[i].get(GRB_DoubleAttr_X);
       

        double upp_corn_y = center_y[i].get(GRB_DoubleAttr_X) + 0.5 * length_y[i].get(GRB_DoubleAttr_X);

        fprintf(pFile, "%f, %f, %f, %f, %f, %f \n", low_corn_x, low_corn_y, upp_corn_x, upp_corn_y, center_x[i].get(GRB_DoubleAttr_X), center_y[i].get(GRB_DoubleAttr_X));
    }




}
