/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*                                                      */
/* File:  rudder_controller.c                           */
/*                                                      */
/* Author: Automatically generated by Xfuzzy            */
/*                                                      */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <math.h>
#include "rudder_controller.h"

/*======================================================*/
/*  Common function to compute a fuzzy number           */
/*======================================================*/

static double compute(FuzzyNumber fn,double x) {
 int length = fn.length;
 int i;
 double imp = fn.imp(fn.degree[0],fn.conc[0].equal(x));
 double mu = imp

 for(i=1; i<length; i++) {
  imp = fn.imp(fn.degree[i],fn.conc[i].equal(x));
  mu = fn.also(mu,imp);
 }
 return mu;
}

/*======================================================*/
/*  MembershipFunction MF_xfl_singleton                 */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double MF_xfl_singleton_equal(double x, double min, double max, double step, double a) {
    return (x==a? 1 : 0); 

}

/*======================================================*/
/*  MembershipFunction MF_xfl_trapezoid                 */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double MF_xfl_trapezoid_equal(double x, double min, double max, double step, double a, double b, double c, double d) {
    return (x<a || x>d? 0: (x<b? (x-a)/(b-a) : (x<c?1 : (d-x)/(d-c)))); 

}

/*======================================================*/
/*  MembershipFunction MF_xfl_triangle                  */
/*======================================================*/

/*------------------------------------------------------*/
/* Function to compute an equal relationship            */
/*------------------------------------------------------*/

static double MF_xfl_triangle_equal(double x, double min, double max, double step, double a, double b, double c) {
    return (a<x && x<=b? (x-a)/(b-a) : (b<x && x<c? (c-x)/(c-b) : 0)); 

}

/*======================================================*/
/*  Operatorset OP__default_                            */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the operator ALSO                     */
/*------------------------------------------------------*/

static double OP__default__Also(double a, double b) {
    return (a>b? a : b); 

}

/*------------------------------------------------------*/
/* Description of the operator IMPLICATION              */
/*------------------------------------------------------*/

static double OP__default__Imp(double a, double b) {
    return (a<b? a : b); 

}

/*------------------------------------------------------*/
/* Description of the defuzzification method            */
/*------------------------------------------------------*/

static double OP__default__Defuz(FuzzyNumber mf) {
 double min = mf.min;
 double max = mf.max;
 double step = mf.step;
     double x, m, num=0, denom=0;
     for(x=min; x<=max; x+=step) {
      m = compute(mf,x);
      num += x*m;
      denom += m;
     }
     if(denom==0) return (min+max)/2;
     return num/denom;

}


/*======================================================*/
/*  Type TP_rudder                                      */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label full_port                   */
/*------------------------------------------------------*/

static double TP_rudder_full_port_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,0.0);
}

/*------------------------------------------------------*/
/* Description of the label port                        */
/*------------------------------------------------------*/

static double TP_rudder_port_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,25.0);
}

/*------------------------------------------------------*/
/* Description of the label centered                    */
/*------------------------------------------------------*/

static double TP_rudder_centered_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,50.0);
}

/*------------------------------------------------------*/
/* Description of the label starboard                   */
/*------------------------------------------------------*/

static double TP_rudder_starboard_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,75.0);
}

/*------------------------------------------------------*/
/* Description of the label full_starboard              */
/*------------------------------------------------------*/

static double TP_rudder_full_starboard_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,100.0);
}

/*======================================================*/
/*  Type TP_goal_alignment                              */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label far_left                    */
/*------------------------------------------------------*/

static double TP_goal_alignment_far_left_equal(double x){
   return MF_xfl_trapezoid_equal(x,-180.0,180.0,1.0,-210.0,-180.0,-90.0,-60.0);
}

/*------------------------------------------------------*/
/* Description of the label too_left                    */
/*------------------------------------------------------*/

static double TP_goal_alignment_too_left_equal(double x){
   return MF_xfl_triangle_equal(x,-180.0,180.0,1.0,-90.0,-60.0,-30.0);
}

/*------------------------------------------------------*/
/* Description of the label left                        */
/*------------------------------------------------------*/

static double TP_goal_alignment_left_equal(double x){
   return MF_xfl_triangle_equal(x,-180.0,180.0,1.0,-60.0,-30.0,30.0);
}

/*------------------------------------------------------*/
/* Description of the label right                       */
/*------------------------------------------------------*/

static double TP_goal_alignment_right_equal(double x){
   return MF_xfl_triangle_equal(x,-180.0,180.0,1.0,-30.0,30.0,60.0);
}

/*------------------------------------------------------*/
/* Description of the label too_right                   */
/*------------------------------------------------------*/

static double TP_goal_alignment_too_right_equal(double x){
   return MF_xfl_triangle_equal(x,-180.0,180.0,1.0,30.0,60.0,90.0);
}

/*------------------------------------------------------*/
/* Description of the label far_right                   */
/*------------------------------------------------------*/

static double TP_goal_alignment_far_right_equal(double x){
   return MF_xfl_trapezoid_equal(x,-180.0,180.0,1.0,60.0,90.0,180.0,210.0);
}

/*======================================================*/
/*  Rulebase RL_rudder_controller_rules                 */
/*======================================================*/

static void RL_rudder_controller_rules(double goal_alignment, double *rudder) {
 double _rl;

 double _rudder_degree[6];
 Consequent _rudder_conc[6];
 FuzzyNumber _rudder;
 _rudder.min = 0.0;
 _rudder.max = 100.0;
 _rudder.step = 1.0;
 _rudder.imp = OP__default__Imp;
 _rudder.also = OP__default__Also;
 _rudder.length = 6;
 _rudder.degree = _rudder_degree;
 _rudder.conc = _rudder_conc;
 int _rudder_i = 0;

 double _goal_alignment_eq[6];
 _goal_alignment_eq[0] = TP_goal_alignment_far_left_equal(goal_alignment);
 _goal_alignment_eq[1] = TP_goal_alignment_too_left_equal(goal_alignment);
 _goal_alignment_eq[2] = TP_goal_alignment_left_equal(goal_alignment);
 _goal_alignment_eq[3] = TP_goal_alignment_right_equal(goal_alignment);
 _goal_alignment_eq[4] = TP_goal_alignment_too_right_equal(goal_alignment);
 _goal_alignment_eq[5] = TP_goal_alignment_far_right_equal(goal_alignment);

 _rl = _goal_alignment_eq[0];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_full_port_equal;
 _rudder_i++;

 _rl = _goal_alignment_eq[1];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_port_equal;
 _rudder_i++;

 _rl = _goal_alignment_eq[2];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_centered_equal;
 _rudder_i++;

 _rl = _goal_alignment_eq[3];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_centered_equal;
 _rudder_i++;

 _rl = _goal_alignment_eq[4];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_starboard_equal;
 _rudder_i++;

 _rl = _goal_alignment_eq[5];
 _rudder_degree[_rudder_i] = _rl;
 _rudder_conc[_rudder_i].equal = TP_rudder_full_starboard_equal;
 _rudder_i++;

 *rudder = OP__default__Defuz(_rudder);
}


/*======================================================*/
/*                   Inference Engine                   */
/*======================================================*/

void rudder_controllerInferenceEngine(double goal_alignment, double *_d_rudder) {
 double rudder;
 RL_rudder_controller_rules(goal_alignment, &rudder);
 *_d_rudder = rudder;
}

