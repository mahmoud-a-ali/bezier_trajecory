
/**
\file   plot2d_waypts_path.cpp
\brief  plot path between waypoint for 2 joints
 *
 *  this node subsscribe to the topic contains the states of the joints "topic_name",and to the cmd_pos/cmd_vel topic,
 *  then it plots the trajectories that are received through the cmd_topic and and topic_name
 * run this node by passing the topic_name of the topic that contains the state of 6 joint and mode for commanded pos/vel
 * $ rosrun plot_trajectory plot2d_trajectory _topic_name:= topic_name _mode:= "pos" or "vel"
 *  topic_name= "/state_last_waypts" or "/state_each_waypts" for the two cases we have
\author  Mahmoud Ali
\date    3/5/2019
*/




#include<vector>
#include<iostream>
#include "ros/ros.h"
#include "std_msgs/Float64MultiArray.h"
//#include "custom_msgs/state_msg.h"

//#include<normal_toppra_traj_instant_1.h>
#include<normal_toppra_traj_instant_3.h>
#include <Eigen/Dense>
#include <math.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace Eigen;
using namespace std;
using Eigen::MatrixXd;

const double sm=180, vm= 130, am=250, jm=1000, frq=125; // pos, vel, acc, jrk max limits





void plot_seg_iterations(int iter, int plot_per_loop, int sbplt_rows, int sbplt_cols, bool last_iter,
                 const std::vector<double> &POS, const std::vector<double> &VEL,
                 const std::vector<double> &ACC, const std::vector<double> &JRK,
                 const std::vector<double> &Tm, const std::vector<double> &P_jt_wpt,
                 const std::vector<double> &V_jt_wpt, const std::vector<double> &A_jt_wpt
                 ){
    //   here each iteration in subfigure
    bool pos_plt=false, vel_plt=true, acc_plt=true, jrk_plt = false;// select states to plot
    pos_plt=false, vel_plt=false, acc_plt=false, jrk_plt = true;
    std::string  sbplt_name;

    if(  (iter)%plot_per_loop == 0 ){
        plt::figure(1);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop   );
        plt::plot( Tm, POS);
        //                plt::plot( T_vec, wpt_states[0],"r*");
        sbplt_name= "pos_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(2);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop   );
        plt::plot(  Tm, VEL);
        //                plt::plot( T_vec, wpt_states[1],"r*");
        sbplt_name= "vel_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(3);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop   );
        plt::plot(  Tm, ACC);
        //                plt::plot( T_vec, wpt_states[2],"r*");
        sbplt_name= "acc_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(4);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop   );
        plt::plot(  Tm, JRK);
        //                plt::plot( T_vec, wpt_states[3],"r*");
        sbplt_name= "jrk_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);
        //    plt::legend(); // Enable legend.

    }
    if(last_iter){ // to plot last iteration
        plt::figure(1);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop  +1);
        plt::plot( Tm, POS);
        //                plt::plot( T_vec, wpt_states[0],"r*");
        sbplt_name= "pos_last_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(2);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop    +1);
        plt::plot(  Tm, VEL);
        //                plt::plot( T_vec, wpt_states[1],"r*");
        sbplt_name= "vel_last_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(3);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop    +1);
        plt::plot(  Tm, ACC);
        //                plt::plot( T_vec, wpt_states[2],"r*");
        sbplt_name= "acc_last_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(4);
        plt::subplot(  sbplt_rows, sbplt_cols,iter/plot_per_loop    +1);
        plt::plot(  Tm, JRK);
        //                plt::plot( T_vec, wpt_states[3],"r*");
        sbplt_name= "jrk_last_iter_" + std::to_string(iter);
        plt::title(  sbplt_name);
        plt::grid(true);

        //    plt::legend(); // Enable legend.
    }

}


void plot_jt_traj( const std::vector<double> &POS, const std::vector<double> &VEL,
                  const std::vector<double> &ACC, const std::vector<double> &JRK,
                  const std::vector<double> &Tm, const std::vector<double> &P_jt_wpt,
                  const std::vector<double> &V_jt_wpt, const std::vector<double> &A_jt_wpt,
                   const std::vector<double> & T_jt_wpt ){
    //   here each iteration in subfigure
        std::string  sbplt_name;
        plt::figure(1);
        plt::subplot(  4, 1,1  );
        plt::plot( Tm, POS);
                        plt::plot( T_jt_wpt, P_jt_wpt,"r*");
        sbplt_name= "pos" ;
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::subplot(  4, 1,2   );
        plt::plot(  Tm, VEL);
                        plt::plot( T_jt_wpt, V_jt_wpt,"r*");
        sbplt_name= "vel" ;
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::subplot(  4, 1,3 );
        plt::plot(  Tm, ACC);
                        plt::plot( T_jt_wpt, A_jt_wpt,"r*");
        sbplt_name= "acc" ;
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::subplot(  4, 1,4 );
        plt::plot(  Tm, JRK);
        //                plt::plot( T_vec, wpt_states[3],"r*");
        sbplt_name= "jrk" ;
        plt::title(  sbplt_name);
        plt::grid(true);
        //    plt::legend(); // Enable legend.
}


bool check_bezier_jerk_limit(double t1, double t2,  double t0_snp, double jm, const std::vector<double> &coef){
    double P0=coef[0], P1=coef[1], P2=coef[2], P3=coef[3], P4=coef[4], P5=coef[5];
    double tm[]= {t1, t0_snp, t2};
    double jrk =0;
     for(auto t : tm){
        jrk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
        std::cout<<"t: "<< t <<"   check_jrk: "<< jrk<<std::endl;
        if(abs(jrk) > jm)
           return false;
     }
     return true;
}
bool check_bezier_acc_limit(double t1, double t2, double t0_jrk1, double t0_jrk2, double am, const std::vector<double> &coef){
    double P0=coef[0], P1=coef[1], P2=coef[2], P3=coef[3], P4=coef[4], P5=coef[5];
    double tm[]= {t0_jrk1, t0_jrk2};
    double acc =0;
     for(auto t : tm){
         acc = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
         std::cout<<"t: "<< t<<"     check_acc: "<< acc<<std::endl;
         if(abs(acc) > am)
           return false;
     }
     return true;
}
bool check_bezier_vel_limit(double t1, double t2,  double t0_snp, double vm, const std::vector<double> &coef){
    double P0=coef[0], P1=coef[1], P2=coef[2], P3=coef[3], P4=coef[4], P5=coef[5];
    double t= t0_snp;
    double vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    std::cout<<"t: "<< t <<"     check_vel: "<< vel<<std::endl;
    if(abs(vel) > vm)
        return false;
     else
        return true;
}


void compute_bezier_segment_coef(double t1, double t2, double p0, double v0, double a0, double pn, double vn, double an, std::vector<double> &coef){
    coef.resize(6); //P0, P1, P2, P3, P4, P5
    coef[0] = -p0/pow((t1 - t2), 5) ;
    coef[1] = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1 - t2),5));
    coef[2] = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1 - t2), 5));
    coef[3] = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1 - t2), 5));
    coef[4] = -(5*pn + t1*vn - t2*vn)/(5*pow((t1 - t2), 5));
    coef[5] = -pn/pow((t1 - t2), 5);
}
void sample_bezier_segment(double t, double t1, double t2, const std::vector<double> &coef, std::vector<double> &state){
    double P0=coef[0], P1=coef[1], P2=coef[2], P3=coef[3], P4=coef[4], P5=coef[5];
    state.resize(4); //pos, vel, acc, jrk
    state[0] = P5*pow( (t-t1), 5) - P0*pow( (t-t2), 5) + 5*P1*(t - t1)*pow( (t-t2), 4) - P4*(5*t - 5*t2)*pow( (t-t1), 4) - 10*P2*pow( (t-t1), 2)*pow( (t-t2), 3) + 10*P3*pow( (t-t1), 3)*pow( (t-t2), 2);
    state[1] = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
    state[2] = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
    state[3] = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
    // if snap required its eq is:
    //snp = 120*P1*(t - t1) - 60*P0*(2*t - 2*t2) + 240*P1*(2*t - 2*t2) - 240*P2*(2*t - 2*t1) - 360*P2*(2*t - 2*t2) + 360*P3*(2*t - 2*t1) + 240*P3*(2*t - 2*t2) - 240*P4*(2*t - 2*t1) + 60*P5*(2*t - 2*t1) - 24*P4*(5*t - 5*t2)
}
void compute_bezier_maxmin_times(double t1, double t2, std::vector<double> &coef, double &t0_snp, double &t0_jrk1, double &t0_jrk2){
    double P0=coef[0], P1=coef[1], P2=coef[2], P3=coef[3], P4=coef[4], P5=coef[5];
    t0_snp = (120*P0*t2 - 120*P1*t1 - 480*P1*t2 + 480*P2*t1 + 720*P2*t2 - 720*P3*t1 - 480*P3*t2 + 480*P4*t1 + 120*P4*t2 - 120*P5*t1)/(120*P0 - 600*P1 + 1200*P2 - 1200*P3 + 600*P4 - 120*P5);
//    std::cout<<"t0_snp: "<< t0_snp<< std::endl;

    // solve quadrtic equations to find when jerk pass through zero
    double A=  180*P1 -  60*P0 - 180*P2 + 60*P3;
    double B= -60*P2  + 180*P3 - 180*P4 + 60*P5;
    double C=  120*P1 - 360*P2 + 360*P3 - 120*P4;
    double a= A+B+C;
    double b= - (2*t2*A + 2*t1*B + t1*C + t2*C);
    double c= (A*t2*t2 + B*t1*t1 + C*t1*t2);
    double disc= b*b - 4*a*c;
    t0_jrk1 = (-b -sqrt(disc) ) / (2*a);
    t0_jrk2 = (-b +sqrt(disc) ) / (2*a);
    //            if(disc<0)
    //                std::cout<<"warn: unexpected disc<0 .... "<<std::endl;
    //            std::cout<<"disc: "<<disc<< "  t0_jrk1: "<<t0_jrk1 << "  t0_jrk2: "<<t0_jrk2 << std::endl;
}


void plot_bezier_segment(double t1, double t2, double frq, const std::vector<double> coef, int iter, bool show_legend=false){
    std::vector<double>  seg_POS, seg_VEL, seg_ACC, seg_JRK, seg_Tm, state;
    double t=t1;
    while(t<t2){
        sample_bezier_segment( t,  t1,  t2, coef, state);
        seg_POS.push_back(state[0]);
        seg_VEL.push_back(state[1]);
        seg_ACC.push_back(state[2]);
        seg_JRK.push_back(state[3]);
        seg_Tm.push_back(t);
        t+=1/frq;
    }

    std::string name="iter_" + std::to_string(iter);
    plt::figure(1);
    if(show_legend)
        plt::named_plot(name,  seg_Tm, seg_POS);
    else
        plt::plot( seg_Tm, seg_POS);
    plt::grid(true);
    plt::legend();
    plt::title("pos");

    plt::figure(2);
    plt::plot(  seg_Tm, seg_VEL);
    plt::grid(true);
    plt::legend();
    plt::title("vel");

    plt::figure(3);
    plt::plot(  seg_Tm, seg_ACC);
    plt::grid(true);
    plt::legend();
    plt::title("acc");

    plt::figure(4);
    plt::plot(  seg_Tm, seg_JRK);
    plt::grid(true);
    plt::legend();
    plt::title("jrk");

}

// finds in which segment time instant t belons to
int find_seg_number (double t, std::vector<double> T_vec){
    // segment changes from 0 to n-2 which mean n-1 seg
    int seg = 0;
    if(t<= T_vec[0]) //less than tstart
        return seg;
    else if(t>= T_vec[T_vec.size() -2 ] ) //if t is greater than the starting time of the last segment
        return T_vec.size() - 2;
    else {
        for (int i=0; i< T_vec.size()-2 ; i++) { // between tstart and tend
            if(t>= T_vec[i] && t< T_vec[i+1])
                seg = i;
        }
        return seg;
    }
}



void sample_bezier_trajectory(double t, const std::vector<double> &T_wpt,
                              const std::vector<double> &P_wpt, const std::vector<double> &V_wpt,
                              const std::vector<double> &A_wpt, std::vector<double> &state){

    int seg= find_seg_number(t, T_wpt);
    std::cout<<"seg: "<< seg <<std::endl;
    double t1=T_wpt[seg],  t2=T_wpt[seg+1];
    double p0=P_wpt[seg],  pn=P_wpt[seg+1];
    double v0=V_wpt[seg],  vn=V_wpt[seg+1];
    double a0=A_wpt[seg],  an=A_wpt[seg+1];
    std::vector<double> coef;
    compute_bezier_segment_coef( t1,  t2,  p0,  v0,  a0,  pn,  vn,  an, coef);
    sample_bezier_segment( t,  t1,  t2, coef, state);
 }




void plot_bezier_trajectory(const std::vector<double> &T_wpt, const std::vector<double> &P_wpt,
                            const std::vector<double> &V_wpt, const std::vector<double> &A_wpt, double frq){

    std::vector<double>  POS, VEL, ACC, JRK, Tm, state;
    double t= T_wpt[0];
    std::cout<<" T_wpt.back(): "<< T_wpt.back() <<std::endl;
    while( t < T_wpt.back() ){
        sample_bezier_trajectory( t, T_wpt, P_wpt, V_wpt, A_wpt, state);
        POS.push_back(state[0]);
        VEL.push_back(state[1]);
        ACC.push_back(state[2]);
        JRK.push_back(state[3]);
        Tm.push_back(t);
        std::cout<<"state: "<< state[0]<< ",  "<< state[1] << ",  "<< state[2] << ",  "<< state[3]<<std::endl;
        t+=1/frq;
    }
    plt::figure(1);
    plt::plot( Tm, POS);
    plt::grid(true);
    plt::title("pos");

    plt::figure(2);
    plt::plot(  Tm, VEL);
    plt::grid(true);
    plt::title("vel");

    plt::figure(3);
    plt::plot(  Tm, ACC);
    plt::grid(true);
    plt::title("acc");

    plt::figure(4);
    plt::plot(  Tm, JRK);
    plt::grid(true);
    plt::title("jrk");

}






int main(int argc, char **argv)
{
    ros::init(argc, argv, "quintic_isp");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //============ read trajectory ==========
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();
    // extract pos, vel ,acc
    std::vector< std::vector<double> > P_jt_wpt, V_jt_wpt, A_jt_wpt,T_jt_wpt;
    P_jt_wpt.resize(n_jts);
    V_jt_wpt.resize(n_jts);
    A_jt_wpt.resize(n_jts);
    T_jt_wpt.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            P_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );
            V_jt_wpt[jt].push_back( traj.points[pt].velocities[jt] );
            A_jt_wpt[jt].push_back( traj.points[pt].accelerations[jt] );
            T_jt_wpt[jt].push_back(0);
        }
    }








    double P0=0, P1=0, P2=0, P3=0, P4=0, P5=0;
    int iter=0;
    int  jt=4;
    double p0=0, pn=0, v0=0, vn=0, a0=0, an=0, t1=0, t2=0;
    double pos=0, vel=0, acc=0, jrk=0, snp;

    t1=0; t2=5*.008;
    std::vector<double> POS, VEL, ACC, JRK, Tm;
    for(int seg= 0; seg<n_pts-1 ; seg++){ //n_pts-1
        ROS_INFO_STREAM("jt: "<< jt <<"    seg: "<< seg);
        t1=T_jt_wpt[jt][seg]; // start time of current segemnt equal end time of previous one
        p0=P_jt_wpt[jt][seg];  pn=P_jt_wpt[jt][seg+1];
        v0=V_jt_wpt[jt][seg];  vn=V_jt_wpt[jt][seg+1];
        a0=A_jt_wpt[jt][seg];  an=A_jt_wpt[jt][seg+1];
        ROS_INFO_STREAM("p0: "<< p0 <<"    pn: "<< pn);
        ROS_INFO_STREAM("v0: "<< v0 <<"    vn: "<< vn);
        ROS_INFO_STREAM("a0: "<< a0 <<"    an: "<< an);

        t2=t1+(5*1/frq);
        bool jrk_lmtd=false, acc_lmtd=true, vel_lmtd=true;
        std::vector<double> state, coef;
        coef.resize(6);
        state.resize(4);

        while(!jrk_lmtd || !acc_lmtd  || !vel_lmtd){   //for each seg, repeat till reach check limits
            ROS_INFO_STREAM("t1: "<< t1 <<"    t2: "<< t2);
            compute_bezier_segment_coef( t1,  t2,  p0,  v0,  a0,  pn,  vn,  an, coef);

            //check changes for each iteration by plotting segment
//            plot_bezier_segment( t1,  t2,  frq, coef,  iter,  true);

            // check maxmin times, vel, acc, jrk limits
            double t0_snp=0, t0_jrk1=0, t0_jrk2=0;
            compute_bezier_maxmin_times(t1, t2, coef, t0_snp, t0_jrk1, t0_jrk2);
            jrk_lmtd = check_bezier_jerk_limit(t1, t2, t0_snp, jm, coef);
            if(!jrk_lmtd)
                t2+= 1/frq;
            else {
                acc_lmtd = check_bezier_acc_limit(t1, t2, t0_jrk1, t0_jrk1, am, coef);
                if(!acc_lmtd)
                    t2+= 1/frq;
                else {
//                    vel_lmtd = check_bezier_vel_limit(t1, t2, t0_snp, vm, coef);
                    if(!vel_lmtd)
                        t2+= 1/frq;
                    else {
                        //plot segment with final parameters
//                        plot_bezier_segment( t1,  t2,  frq, coef,  iter,  true);
                        T_jt_wpt[jt][seg+1] = t2;  //update end time of current segment
                        iter=0;
                        ROS_INFO_STREAM("seg_ "<<seg <<"  done!");
                    }
                }
            }
            iter++;
            ROS_INFO_STREAM("#iteration= "<<iter <<"   jrk_lmtd: "<< jrk_lmtd <<"   acc_lmtd: "<< acc_lmtd <<"   vel_lmtd: "<< vel_lmtd );
//            if(iter >=1)
//                break;

        }//while jrk_lmtd

    }//for each segment

//    plot_jt_traj( POS, VEL, ACC,JRK, Tm, P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt], T_jt_wpt[jt] );
    ROS_INFO_STREAM("#new_times:   " );
    for (int i=0; i< T_jt_wpt[jt].size(); i++)
        ROS_INFO_STREAM("#T_jt_wpt[jt]["<<i <<"] = "<< T_jt_wpt[jt][i] );


    plot_bezier_trajectory(T_jt_wpt[jt], P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt], frq);
    plt::show();


}





