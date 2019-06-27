#include <ros/ros.h>
#include "bezier_quintic_segment.h"
#include "bezier_trajectory.h"

//#include<normal_toppra_traj_instant_1.h>
#include<normal_toppra_traj_instant_3.h>
//#include<test_trajectory_max_jrk.h>

#include <python2.7/Python.h>
#include "matplotlibcpp.h"
namespace plt = matplotlibcpp;
using namespace std;
const double max_pos=180, max_vel= 130, max_acc=250, max_jrk=1000, frq=125; // pos, vel, acc, jrk max limits


void generate_sync_durations( std::vector< std::vector<double> > traj_durations, std::vector<double> &sync_durations){
    int n_jts= traj_durations.size();
    int n_pts= traj_durations[0].size() +1;
    sync_durations.resize(n_pts-1);

    //takes max durations for each segment over different joints/traajectories, create synchronised durations
    for(int seg=0; seg<n_pts-1 ; seg++){
       sync_durations[seg] = traj_durations[0][seg];
       for(int jt=1; jt<n_jts ; jt++){
           if( sync_durations[seg] < traj_durations[jt][seg]  )
               sync_durations[seg]= traj_durations[jt][seg] ;
       }
    }
}


void compute_durations_from_times( std::vector<double> traj_times, std::vector<double>  &traj_durations){
    //create trajectories durations from times
    int n_pts=traj_times.size();
    traj_durations.resize( n_pts -1);
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[seg] = traj_times[seg+1] - traj_times[seg];
}


void compute_times_from_durations( double t_start, std::vector<double>  traj_durations,  std::vector<double>  &traj_times ){
    //create trajectories times from durations
    int n_pts=traj_durations.size()+1;
    traj_times.resize( n_pts );
    traj_times[0]= t_start;

    for (int pt=0; pt< n_pts-1; pt++)
            traj_times[pt+1] = traj_times[pt] + traj_durations[pt]; // t= last_time + duration of that segment

}


void compute_durations_from_times(std::vector< std::vector<double> > traj_times, std::vector< std::vector<double> > &traj_durations){
    //create trajectories durations from times
    int n_jts=traj_times.size();
    int n_pts=traj_times[0].size();
    traj_durations.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)  //n_segs = n_pts-1
            traj_durations[jt].push_back( traj_times[jt][seg+1] - traj_times[jt][seg]);
}


void compute_times_from_durations(double t_start, std::vector<std::vector<double>> traj_durations, std::vector< std::vector<double> > &traj_times ){
    //create trajectories times from durations
    int n_jts=traj_durations.size();
    int n_pts=traj_durations[0].size()+1;
    traj_times.resize( n_jts );

    for(int jt=0; jt<n_jts; jt++){
        traj_times[jt][0]= t_start;
        for (int pt=0; pt< n_pts-1; pt++)
            traj_times[jt][pt+1] = traj_times[jt][pt] + traj_durations[jt][pt]; // t= last_time + duration of that segment
     }
}


//void generate_sync_times(std::vector< std::vector<double> > traj_times, std::vector<double> &sync_T){
//    int n_jts=traj_times.size();
//    int n_pts=traj_times[0].size();
//    std::vector< std::vector<double> >  traj_durations;
//    std::vector<double> sync_duration; //it same for all trajectory
//    traj_durations.resize(n_jts);
//    sync_duration.resize(n_pts-1);
//    sync_T.resize(n_pts);


//    //takes max durations for each segment over different joints/traajectories, create synchronised durations
//    for(int seg=0; seg<n_pts-1 ; seg++){
//       sync_duration[seg] = traj_times[0][seg+1] - traj_times[0][seg];
//       for(int jt=1; jt<n_jts ; jt++){
//           if( sync_duration[seg] < (traj_times[jt][seg+1] - traj_times[jt][seg]) )
//               sync_duration[seg]= (traj_times[jt][seg+1] - traj_times[jt][seg]);
//       }
//    }

//    //fill trajectories times: synchronised times
//    sync_T[0]=traj_times[0][0];
//    for (int i=0; i< n_pts-1; i++)
//       sync_T[i+1] = sync_T[i] + sync_duration[i];

//}










int main(int argc, char **argv)
{
    ros::init(argc, argv, "bezier_segment_test");
    ros::NodeHandle nh;

    ROS_INFO("initializing path .... ");
    //============ read trajectory ==========
    trajectory_msgs::JointTrajectory  traj;
    traj = generate_traj();
    int n_jts = traj.joint_names.size();
    int n_pts = traj.points.size();
    // extract pos, vel ,acc
    std::vector< std::vector<double> > P_jt_wpt, V_jt_wpt, A_jt_wpt, T_jt_wpt, durations;
    P_jt_wpt.resize(n_jts);
    V_jt_wpt.resize(n_jts);
    A_jt_wpt.resize(n_jts);
    T_jt_wpt.resize(n_jts);
    durations.resize(n_jts);
    for(int jt=0; jt<n_jts; jt++){
        for(int pt=0; pt<n_pts; pt++){
            P_jt_wpt[jt].push_back( traj.points[pt].positions[jt] );
            V_jt_wpt[jt].push_back( traj.points[pt].velocities[jt] );
            A_jt_wpt[jt].push_back( traj.points[pt].accelerations[jt] );
            T_jt_wpt[jt].push_back(0);
        }
    }
    double t_start =T_jt_wpt[0][0];
    for(int jt=0; jt<n_jts; jt++)
        for(int seg=0; seg<n_pts-1; seg++)
            durations[jt].push_back( T_jt_wpt[jt][seg+1] - T_jt_wpt[jt][seg]);






    //create trajectory ------------------------------------------------------------
    for(int jt=0; jt<n_jts ; jt++){ //n_jts-1
        ROS_INFO_STREAM("========================================================");
        ROS_INFO_STREAM("jt: "<< jt );
        ROS_INFO_STREAM("========================================================");

        ROS_INFO_STREAM("###original times:  ");
        for (int i=0; i<T_jt_wpt[jt].size(); i++)
            ROS_INFO_STREAM("  "<< T_jt_wpt[jt][i]);

        bezier_trajectory traj(P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt], durations[jt], t_start );
        T_jt_wpt[jt] =traj.update_waypoints_times(max_pos, max_vel, max_acc, max_jrk);

        ROS_INFO_STREAM("###new iginal times:  ");
        for (int i=0; i<T_jt_wpt[jt].size(); i++)
            ROS_INFO_STREAM("  "<< T_jt_wpt[jt][i]);

        traj.print_attributes();
        std::vector<double> T_vec, POS, VEL,ACC, JRK;
        traj.trajectory_states( frq, T_vec, POS, VEL,ACC, JRK );

        std::cout<<"T_vec.size(): "<< T_vec.size() << std::endl;
        std::cout<<"POS.size(): "<< POS.size() << std::endl;
        std::cout<<"VEL.size(): "<< VEL.size() << std::endl;
        std::cout<<"ACC.size(): "<< ACC.size() << std::endl;
        std::cout<<"JRK.size(): "<< JRK.size() << std::endl;


        plt::figure(1);
        plt::subplot(2, 2, 1);
        plt::named_plot( "pos",T_vec, POS); plt::named_plot( "waypoints",T_jt_wpt[jt], P_jt_wpt[jt], "r*");
        plt::title("pos"); plt::grid(true); plt::title("pos"); plt::legend();
        plt::subplot(2, 2, 2);
        plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
        plt::subplot(2, 2, 3);
        plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
        plt::subplot(2, 2, 4);
        plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");



    }

     plt::show();




    //synchronization part
    std::vector<std::vector<double>> traj_durations;
    std::vector<double> sync_durations, sync_T;
    compute_durations_from_times(T_jt_wpt, traj_durations);
    generate_sync_durations( traj_durations, sync_durations);
    compute_times_from_durations( t_start, sync_durations, sync_T);
    ROS_INFO_STREAM("###new times sync_T:  ");
    for (int i=0; i<sync_T.size(); i++)
        ROS_INFO_STREAM("  "<< sync_T[i]);



for(int jt=0; jt<n_jts ; jt++){ //n_pts-1
    ROS_INFO_STREAM("========================================================");
    ROS_INFO_STREAM("jt: "<< jt );
    ROS_INFO_STREAM("========================================================");

    bezier_trajectory syn_traj(P_jt_wpt[jt], V_jt_wpt[jt], A_jt_wpt[jt], sync_durations, t_start );
    syn_traj.set_absolute_limits(max_pos, max_vel, max_acc, max_jrk);

    std::vector<double> T_vec, POS, VEL,ACC, JRK;
    syn_traj.trajectory_states( frq, T_vec, POS, VEL,ACC, JRK );
    syn_traj.print_attributes();

//    syn_traj.print_attributes();
    std::cout<<"T_vec.size(): "<< T_vec.size() << std::endl;
    std::cout<<"POS.size(): "<< POS.size() << std::endl;
    std::cout<<"VEL.size(): "<< VEL.size() << std::endl;
    std::cout<<"ACC.size(): "<< ACC.size() << std::endl;
    std::cout<<"JRK.size(): "<< JRK.size() << std::endl;


    plt::figure(1);
    plt::subplot(2, 2, 1);
    plt::named_plot( "pos",T_vec, POS); plt::named_plot( "waypoints", sync_T ,  P_jt_wpt[jt], "r*");
    plt::title("pos"); plt::grid(true); plt::legend();
    plt::subplot(2, 2, 2);
    plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
    plt::subplot(2, 2, 3);
    plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
    plt::subplot(2, 2, 4);
    plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");
}




    plt::show();



}



//        plt::figure(1);
//        plt::named_plot( "pos",T_vec, POS); plt::grid(true); plt::title("pos");
//        plt::figure(2);
//        plt::named_plot( "vel",T_vec, VEL); plt::grid(true); plt::title("vel");
//        plt::figure(3);
//        plt::named_plot( "acc",T_vec, ACC); plt::grid(true); plt::title("acc");
//        plt::figure(4);
//        plt::named_plot( "jrk",T_vec, JRK); plt::grid(true); plt::title("jrk");



















