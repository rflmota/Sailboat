/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/
/*                                                      */
/* File:  sail_controller.c                             */
/*                                                      */
/* Author: Automatically generated by Xfuzzy            */
/*                                                      */
/*++++++++++++++++++++++++++++++++++++++++++++++++++++++*/

#include <stdio.h>
#include <math.h>
#include "sail_controller.h"

/*======================================================*/
/*  Common function to compute a fuzzy number           */
/*======================================================*/

static double compute(FuzzyNumber fn,double x) {
 int length = fn.length;
 int i;
 double imp = fn.imp(fn.degree[0],fn.conc[0].equal(x));
 double mu = imp;

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
/* Description of the operator AND                      */
/*------------------------------------------------------*/

static double OP__default__And(double a, double b) {
    return (a<b? a : b); 

}

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
/*  Type TP_sail                                        */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label full_tight                  */
/*------------------------------------------------------*/

static double TP_sail_full_tight_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,0.0);
}

/*------------------------------------------------------*/
/* Description of the label tight                       */
/*------------------------------------------------------*/

static double TP_sail_tight_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,25.0);
}

/*------------------------------------------------------*/
/* Description of the label half                        */
/*------------------------------------------------------*/

static double TP_sail_half_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,50.0);
}

/*------------------------------------------------------*/
/* Description of the label loose                       */
/*------------------------------------------------------*/

static double TP_sail_loose_equal(double x){
   return MF_xfl_singleton_equal(x,0.0,100.0,1.0,75.0);
}

/*------------------------------------------------------*/
/* Description of the label full_loose                  */
/*------------------------------------------------------*/

static double TP_sail_full_loose_equal(double x){
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
/*  Type TP_wind_alignment                              */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label bow                         */
/*------------------------------------------------------*/

static double TP_wind_alignment_bow_equal(double x){
   return MF_xfl_trapezoid_equal(x,0.0,180.0,1.0,-45.0,0.0,45.0,90.0);
}

/*------------------------------------------------------*/
/* Description of the label slant                       */
/*------------------------------------------------------*/

static double TP_wind_alignment_slant_equal(double x){
   return MF_xfl_triangle_equal(x,0.0,180.0,1.0,45.0,90.0,135.0);
}

/*------------------------------------------------------*/
/* Description of the label stern                       */
/*------------------------------------------------------*/

static double TP_wind_alignment_stern_equal(double x){
   return MF_xfl_trapezoid_equal(x,0.0,180.0,1.0,90.0,135.0,180.0,225.0);
}

/*======================================================*/
/*  Type TP_roll                                        */
/*======================================================*/

/*------------------------------------------------------*/
/* Description of the label right                       */
/*------------------------------------------------------*/

static double TP_roll_right_equal(double x){
   return MF_xfl_trapezoid_equal(x,0.0,90.0,1.0,-30.0,0.0,30.0,60.0);
}

/*------------------------------------------------------*/
/* Description of the label heel                        */
/*------------------------------------------------------*/

static double TP_roll_heel_equal(double x){
   return MF_xfl_trapezoid_equal(x,0.0,90.0,1.0,30.0,60.0,90.0,120.0);
}

/*======================================================*/
/*  Rulebase RL_sail_controller_rules                   */
/*======================================================*/

static void RL_sail_controller_rules(double roll, double goal_alignment, double wind_alignment, double *sail) {
 double _rl;

 double _sail_degree[36];
 Consequent _sail_conc[36];
 FuzzyNumber _sail;
 _sail.min = 0.0;
 _sail.max = 100.0;
 _sail.step = 1.0;
 _sail.imp = OP__default__Imp;
 _sail.also = OP__default__Also;
 _sail.length = 36;
 _sail.degree = _sail_degree;
 _sail.conc = _sail_conc;
 int _sail_i = 0;

 double _roll_eq[2];
 _roll_eq[0] = TP_roll_right_equal(roll);
 _roll_eq[1] = TP_roll_heel_equal(roll);

 double _goal_alignment_eq[6];
 _goal_alignment_eq[0] = TP_goal_alignment_far_left_equal(goal_alignment);
 _goal_alignment_eq[1] = TP_goal_alignment_too_left_equal(goal_alignment);
 _goal_alignment_eq[2] = TP_goal_alignment_left_equal(goal_alignment);
 _goal_alignment_eq[3] = TP_goal_alignment_right_equal(goal_alignment);
 _goal_alignment_eq[4] = TP_goal_alignment_too_right_equal(goal_alignment);
 _goal_alignment_eq[5] = TP_goal_alignment_far_right_equal(goal_alignment);

 double _wind_alignment_eq[3];
 _wind_alignment_eq[0] = TP_wind_alignment_bow_equal(wind_alignment);
 _wind_alignment_eq[1] = TP_wind_alignment_slant_equal(wind_alignment);
 _wind_alignment_eq[2] = TP_wind_alignment_stern_equal(wind_alignment);

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[0]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[1]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[2]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[3]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[4]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[5]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[0]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[1]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[2]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[3]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[4]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[5]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[0]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[1]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[2]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[3]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[4]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[0],_goal_alignment_eq[5]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_full_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[0]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[1]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[2]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[3]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[4]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[5]),_wind_alignment_eq[2]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[0]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[1]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[2]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[3]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_loose_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[4]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[5]),_wind_alignment_eq[1]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_half_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[0]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[1]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[2]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[3]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[4]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 _rl = OP__default__And(OP__default__And(_roll_eq[1],_goal_alignment_eq[5]),_wind_alignment_eq[0]);
 _sail_degree[_sail_i] = _rl;
 _sail_conc[_sail_i].equal = TP_sail_tight_equal;
 _sail_i++;

 *sail = OP__default__Defuz(_sail);
}


/*======================================================*/
/*                   Inference Engine                   */
/*======================================================*/

void sail_controllerInferenceEngine(double roll, double goal_alignment, double wind_alignment, double *_d_sail) {
 double sail;
 RL_sail_controller_rules(roll, goal_alignment, wind_alignment, &sail);
 *_d_sail = sail;
}
