
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





void plot_states(int count, int plot_per_loop, int sbplt_rows, int sbplt_cols, bool last_iter,
                 const std::vector<double> &POS, const std::vector<double> &VEL,
                 const std::vector<double> &ACC, const std::vector<double> &JRK,
                 const std::vector<double> &Tm, const std::vector<double> &P_jt_wpt,
                 const std::vector<double> &V_jt_wpt, const std::vector<double> &A_jt_wpt
                 ){
    //   here each iteration in subfigure
    bool pos_plt=false, vel_plt=true, acc_plt=true, jrk_plt = false;// select states to plot
    pos_plt=false, vel_plt=false, acc_plt=false, jrk_plt = true;
    std::string  sbplt_name;

    if(  (count)%plot_per_loop == 0 ){
        plt::figure(1);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop   );
        plt::plot( Tm, POS);
        //                plt::plot( T_vec, wpt_states[0],"r*");
        sbplt_name= "pos_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(2);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop   );
        plt::plot(  Tm, VEL);
        //                plt::plot( T_vec, wpt_states[1],"r*");
        sbplt_name= "vel_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(3);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop   );
        plt::plot(  Tm, ACC);
        //                plt::plot( T_vec, wpt_states[2],"r*");
        sbplt_name= "acc_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);
        plt::figure(4);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop   );
        plt::plot(  Tm, JRK);
        //                plt::plot( T_vec, wpt_states[3],"r*");
        sbplt_name= "jrk_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);
        //    plt::legend(); // Enable legend.

    }
    if(last_iter){ // to plot last iteration
        plt::figure(1);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop  +1);
        plt::plot( Tm, POS);
        //                plt::plot( T_vec, wpt_states[0],"r*");
        sbplt_name= "pos_last_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(2);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop    +1);
        plt::plot(  Tm, VEL);
        //                plt::plot( T_vec, wpt_states[1],"r*");
        sbplt_name= "vel_last_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(3);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop    +1);
        plt::plot(  Tm, ACC);
        //                plt::plot( T_vec, wpt_states[2],"r*");
        sbplt_name= "acc_last_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);

        plt::figure(4);
        plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop    +1);
        plt::plot(  Tm, JRK);
        //                plt::plot( T_vec, wpt_states[3],"r*");
        sbplt_name= "jrk_last_iter_" + std::to_string(count);
        plt::title(  sbplt_name);
        plt::grid(true);

        //    plt::legend(); // Enable legend.
    }

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

    // get waypoints from trajectory
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
    bool jrk_lmtd=false, acc_lmtd=false, vel_lmtd=false;
    int count=0;
    int  jt=4;
    double p0=0, pn=0, v0=0, vn=0, a0=0, an=0, t1=0, t2=0;
    double pos=0, vel=0, acc=0, jrk=0;

    t1=0; t2=5*.008;

    for(int seg=0; seg<1 ; seg++){ //n_pts-1
        ROS_INFO_STREAM("jt: "<< jt <<"    seg: "<< seg);
        if(seg==0)
            T_jt_wpt[jt][seg]=t1;
        else
            t1=T_jt_wpt[jt][seg-1];

        p0=P_jt_wpt[jt][seg];  pn=P_jt_wpt[jt][seg+1];
        v0=V_jt_wpt[jt][seg];  vn=V_jt_wpt[jt][seg+1];
        a0=A_jt_wpt[jt][seg];  an=A_jt_wpt[jt][seg+1];
        ROS_INFO_STREAM("p0: "<< p0 <<"    pn: "<< pn);
        ROS_INFO_STREAM("v0: "<< v0 <<"    vn: "<< vn);
        ROS_INFO_STREAM("a0: "<< a0 <<"    an: "<< an);

        t2=t1+5*1/frq;
        while(!jrk_lmtd){// || !acc_lmtd  || !vel_lmtd){ //for each seg repeat till reach check limits
            ROS_INFO_STREAM("t1: "<< t1 <<"    t2: "<< t2);

            P0 = -p0/5*pow((t1 - t2), 5) ;
            P1 = -(5*p0 - t1*v0 + t2*v0)/(5*pow((t1 - t2),5));
            P2 = -(a0*t1*t1 - 2*a0*t1*t2 - 8*v0*t1 + a0*t2*t2 + 8*v0*t2 + 20*p0)/(20*pow((t1 - t2), 5));
            P3 = -(an*t1*t1 - 2*an*t1*t2 + 8*vn*t1 + an*t2*t2 - 8*vn*t2 + 20*pn)/(20*pow((t1 - t2), 5));
            P4 = -(5*pn + t1*vn - t2*vn)/(5*pow((t1 - t2), 5));
            P5 = -pn/pow((t1 - t2), 5);

            std::vector<double> POS, VEL, ACC, JRK, Tm;
            double t=t1;
            while(t<t2){
                pos = P5*pow( (t-t1), 5) - P0*pow( (t-t2), 5) + 5*P1*(t - t1)*pow( (t-t2), 4) - P4*(5*t - 5*t2)*pow( (t-t1), 4) - 10*P2*pow( (t-t1), 2)*pow( (t-t2), 3) + 10*P3*pow( (t-t1), 3)*pow( (t-t2), 2);
                vel = 5*P1*pow( (t-t2), 4) - 5*P0*pow( (t-t2), 4) - 5*P4*pow( (t-t1), 4) + 5*P5*pow( (t-t1), 4) + 20*P1*(t - t1)*pow( (t-t2), 3) - 10*P2*(2*t - 2*t1)*pow( (t-t2), 3) + 10*P3*(2*t - 2*t2)*pow( (t-t1), 3) - 4*P4*(5*t - 5*t2)*pow( (t-t1), 3) - 30*P2*pow( (t-t1), 2)*pow( (t-t2), 2) + 30*P3*pow( (t-t1), 2)*pow( (t-t2), 2);
                acc = 40*P1*pow( (t-t2), 3) - 20*P0*pow( (t-t2), 3) - 20*P2*pow( (t-t2), 3) + 20*P3*pow( (t-t1), 3) - 40*P4*pow( (t-t1), 3) + 20*P5*pow( (t-t1), 3) + 60*P1*(t - t1)*pow( (t-t2), 2) - 60*P2*(2*t - 2*t1)*pow( (t-t2), 2) - 30*P2*(2*t - 2*t2)*pow( (t-t1), 2) + 30*P3*(2*t - 2*t1)*pow( (t-t2), 2) + 60*P3*(2*t - 2*t2)*pow( (t-t1), 2) - 12*P4*(5*t - 5*t2)*pow( (t-t1), 2);
                jrk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
                POS.push_back(pos);
                VEL.push_back(vel);
                ACC.push_back(acc);
                JRK.push_back(jrk);
                Tm.push_back(t);
                t+=1/frq;
            }
            // check jerk limits
            std::vector<double> tc;
            tc.push_back( t1);
            tc.push_back( (t1+t2)/2);
            tc.push_back( t2);
            jrk_lmtd=true;
            for (int ptc=0; ptc<tc.size(); ptc++) {
                t=tc[ptc];
                jrk = 180*P1*pow( (t-t2), 2) - 60*P0*pow( (t-t2), 2) - 60*P2*pow( (t-t1), 2) - 180*P2*pow( (t-t2), 2) + 180*P3*pow( (t-t1), 2) + 60*P3*pow( (t-t2), 2) - 180*P4*pow( (t-t1), 2) + 60*P5*pow( (t-t1), 2) + 60*P1*(2*t - 2*t2)*(t - t1) - 90*P2*(2*t - 2*t1)*(2*t - 2*t2) + 90*P3*(2*t - 2*t1)*(2*t - 2*t2) - 12*P4*(2*t - 2*t1)*(5*t - 5*t2);
                if(abs(jrk)> jm)
                    jrk_lmtd=false;
            }
            if(!jrk_lmtd)
                t2+= 1/frq;

            // plot section =================================
            count++;
            int plot_per_loop =3; //to plot 9 iteration to fit on sbplts number
            int sbplt_rows= 3, sbplt_cols= 4;
            plot_states(count, plot_per_loop, sbplt_rows,  sbplt_cols, jrk_lmtd,POS, VEL,  ACC, JRK,   Tm, P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt]  );
            ROS_INFO_STREAM("#iteration= "<<count <<"   jrk_lmtd: "<< jrk_lmtd );
//            if(count >=100)
//                break;




        }//while jrk_lmtd



    }//for each segment




    ROS_INFO_STREAM("#loops= "<<count);
    plt::show();


}










//if(count%plot_per_loop == 0 ){
//    plt::figure(1);
//    plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop  +1);

//    if(pos_plt)t1
//        plt::named_plot("pos" ,  T, POS);
//   if(vel_plt)
//       plt::named_plot("vel" ,  T, VEL);
//   if(acc_plt)
//       plt::named_plot("acc" ,  T, ACC);
//   if(jrk_plt)
//       plt::named_plot("jrk" ,  T, JRK);

//   sbplt_name= "iteration" + std::to_string(count);
//   plt::title(  sbplt_name); // Add graph title
//   plt::legend(); // Enable legend.

//}
//if(lmtd_jrk){
//    plt::figure(1);
//    plt::subplot(  sbplt_rows, sbplt_cols,count/plot_per_loop  +2);
//    if(pos_plt)
//       plt::named_plot("last_iter_pos" ,  T, POS);
//   if(vel_plt)
//       plt::named_plot("last_iter_vel" ,  T, VEL);
//   if(acc_plt)
//       plt::named_plot("last_iter_acc" ,  T, ACC);
//   if(jrk_plt)
//       plt::named_plot("last_iter_jrk" ,  T, JRK);

//    sbplt_name= "iteration" + std::to_string(count);
//    plt::title(  sbplt_name); // Add graph title
//    plt::legend(); // Enable legend.
//}



//here each iteration in subfigure
//if(count%plot_per_loop == 0 ){
//    plt::figure(count/plot_per_loop  +1);
//    plt::subplot(  sbplt_rows, sbplt_cols,1);
//    plt::plot( T, POS);
//    plt::plot( T_vec, wpt_states[0],"r*");
//    sbplt_name= "pos" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,2);
//    plt::plot(  T, VEL);
//    plt::plot( T_vec, wpt_states[1],"r*");
//    sbplt_name= "vel" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,3);
//    plt::plot(  T, ACC);
//    plt::plot( T_vec, wpt_states[2],"r*");
//    sbplt_name= "acc" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,4);
//    plt::plot(  T, JRK);
//    plt::plot( T_vec, wpt_states[3],"r*");
//    sbplt_name= "jrk" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

////    plt::legend(); // Enable legend.

//}
//if(lmtd_jrk){
//    plt::figure(count/plot_per_loop  +2);
//    plt::subplot(  sbplt_rows, sbplt_cols,1);
//    plt::plot( T, POS);
//    plt::plot( T_vec, wpt_states[0],"r*");
//    sbplt_name= "pos" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,2);
//    plt::plot(  T, VEL);
//    plt::plot( T_vec, wpt_states[1],"r*");
//    sbplt_name= "vel" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,3);
//    plt::plot(  T, ACC);
//    plt::plot( T_vec, wpt_states[2],"r*");
//    sbplt_name= "acc" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

//    plt::subplot(  sbplt_rows, sbplt_cols,4);
//    plt::plot(  T, JRK);
//    plt::plot( T_vec, wpt_states[3],"r*");
//    sbplt_name= "jrk" + std::to_string(count);
//    plt::title(  sbplt_name);
//    plt::grid(true);

////    plt::legend(); // Enable legend.
//}



