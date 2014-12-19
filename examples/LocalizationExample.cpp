/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file LocalizationExample.cpp
 * @brief Simple robot localization example, with three "GPS-like" measurements
 * @author Frank Dellaert
 */

/**
 * A simple 2D pose slam example with "GPS" measurements
 *  - The robot moves forward 2 meter each iteration
 *  - The robot initially faces along the X axis (horizontal, to the right in 2D)
 *  - We have full odometry between pose
 *  - We have "GPS-like" measurements implemented with a custom factor
 */

// We will use Pose2 variables (x, y, theta) to represent the robot positions
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Moses3.h>




// We will use simple integer Keys to refer to the robot poses.
#include <gtsam/inference/Key.h>

// As in OdometryExample.cpp, we use a BetweenFactor to model odometry measurements.
#include <gtsam/slam/BetweenFactor.h>
 #include <gtsam/slam/PriorFactor.h>
 #include <gtsam/navigation/GPSFactor.h>

// We add all facors to a Nonlinear Factor Graph, as our factors are nonlinear.
#include <gtsam/nonlinear/NonlinearFactorGraph.h>

// The nonlinear solvers within GTSAM are iterative solvers, meaning they linearize the
// nonlinear functions around an initial linearization point, then solve the linear system
// to update the linearization point. This happens repeatedly until the solver converges
// to a consistent set of variable values. This requires us to specify an initial guess
// for each variable, held in a Values container.
#include <gtsam/nonlinear/Values.h>

// Finally, once all of the factors have been added to our factor graph, we will want to
// solve/optimize to graph to find the best (Maximum A Posteriori) set of variable values.
// GTSAM includes several nonlinear optimizers to perform this step. Here we will use the
// standard Levenberg-Marquardt solver
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>

// Once the optimized values have been calculated, we can also calculate the marginal covariance
// of desired variables
#include <gtsam/nonlinear/Marginals.h>

using namespace std;
using namespace Sophus;
using namespace gtsam;


// Before we begin the example, we must create a custom unary factor to implement a
// "GPS-like" functionality. Because standard GPS measurements provide information
// only on the position, and not on the orientation, we cannot use a simple prior to
// properly model this measurement.
//
// The factor will be a unary factor, affect only a single system variable. It will
// also use a standard Gaussian noise model. Hence, we will derive our new factor from
// the NoiseModelFactor1.
#include <gtsam/nonlinear/NonlinearFactor.h>

#define PI 3.14159265359f
#define useUnary 1
#define printCov 0


namespace gtsam{

class UnaryFactor: public NoiseModelFactor1<Moses3> {

  // The factor will hold a measurement consisting of an (X,Y) location
  // We could this with a Point2 but here we just use two doubles
  Point3 nT_;

public:
  /// shorthand for a smart pointer to a factor
  typedef boost::shared_ptr<UnaryFactor> shared_ptr;

  // The constructor requires the variable key, the (X, Y) measurement value, and the noise model
  UnaryFactor(Key j, const Point3& gpsIn, const SharedNoiseModel& model):
    NoiseModelFactor1<Moses3>(model, j), nT_(gpsIn) {}

  virtual ~UnaryFactor() {}

  // Using the NoiseModelFactor1 base class there are two functions that must be overridden.
  // The first is the 'evaluateError' function. This function implements the desired measurement
  // function, returning a vector of errors when evaluated at the provided variable value. It
  // must also calculate the Jacobians for this measurement function, if requested.

  Vector evaluateError(const Moses3& q, boost::optional<Matrix&> H = boost::none) const
  {
    // The measurement function for a GPS-like measurement is simple:
    // error_x = pose.x - measurement.x
    // error_y = pose.y - measurement.y
    // Consequently, the Jacobians are:
    // [ derror_x/dx  derror_x/dy  derror_x/dtheta ] = [1 0 0]
    // [ derror_y/dx  derror_y/dy  derror_y/dtheta ] = [0 1 0]
    //if (H) (*H) = (Matrix(2,3) << 1.0,0.0,0.0, 0.0,1.0,0.0); //this is crap for moses3
    //return (Vector(2) << q.x() - m_.x(), q.y() - m_.y());          //this is crap for moses3

    if (H) {
	    H->resize(3, 7);
	    H->block < 3, 3 > (0, 0) << q.Sim3::rotation_matrix();  //deriv. trans NOT TESTED
	    H->block < 3, 3 > (0, 3) << zeros(3,3);				//deriv. rot NOT TESTED
	    H->block < 3, 1 > (0, 6) << zeros(3, 1);	//6 not 4      		//deriv scale NOT TESTED


	  }

	Vector3 res =nT_.localCoordinates(Point3(q.Sim3::translation()));
	//cout << "****** "<<res.transpose() <<"   ";
	//cout << nT_.vector().transpose() << "    "<< q.Sim3::translation().transpose() <<" ]]" << endl;
  	return res;


  }

  // The second is a 'clone' function that allows the factor to be copied. Under most
  // circumstances, the following code that employs the default copy constructor should
  // work fine.
  virtual gtsam::NonlinearFactor::shared_ptr clone() const {
    return boost::static_pointer_cast<gtsam::NonlinearFactor>(
        gtsam::NonlinearFactor::shared_ptr(new UnaryFactor(*this))); }

  // Additionally, we encourage you the use of unit testing your custom factors,
  // (as all GTSAM factors are), in which you would need an equals and print, to satisfy the
  // GTSAM_CONCEPT_TESTABLE_INST(T) defined in Testable.h, but these are not needed below.

}; // UnaryFactor




}
int main(int argc, char** argv) {

  // 1. Create a factor graph container and add factors to it
  NonlinearFactorGraph graph;
  NonlinearFactorGraph graphWithGPS;



  

	//cout << Rot3::rodriguez(0.0, 0.0, PI/2)<<endl;


  //Intermediate constraints
  Rot3 R0 = Rot3(Rot3::rodriguez(0.0, 0.0, 0.0)); //Initial prior to origin
  Point3 Pp0(0.0, 0.0, 0.0);  
  

  int minus = +1;

  Rot3 R12 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp12(1.0, 0.0, 0.0);

  Rot3 R23 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp23(1.0, 0.0, 0.0);   

  Rot3 R34 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp34(1.0, 0.0, 0.0);



  Rot3 R45 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp45(1.0, 0.0, 0.0);

  Rot3 R56 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp56(1.0, 0.0, 0.0);   

  Rot3 R67 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp67(1.0, 0.0, 0.0);


  Rot3 R78 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp78(1.0, 0.0, 0.0);

  Rot3 R89 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp89(1.0, 0.0, 0.0);   

  Rot3 R90 = Rot3(Rot3::rodriguez(0.0, 0.0, minus*PI/9.0));
  Point3 Pp90(1.0, 0.0, 0.0);



  float s0,s12,s23,s34,s45,s56,s67,s78,s89,s90;
  s0=1; //Initial Prior to origin.

  s12=0.95;
  s23=0.9;
  s34=0.85;
  s45=0.8;
  s56=0.75;
  s67=0.7;
  s78=0.65;
  s89=0.6;
  s90=0.55;


  //Unary priors
  Point3 Up1(0, 0, 0.0);  
  Point3 Up2(1, -3.78165e-17, 0.0);  
  Point3 Up3(1.93969, 0.34202, 0.0);  
  Point3 Up4(2.70574, 0.984808, 0.0);  
  Point3 Up5(3.20574, 1.85083, 0.0);  
  Point3 Up6(3.37939, 2.83564, 0.0);  
  Point3 Up7(3.20574, 3.82045, 0.0);  
  Point3 Up8(2.70574, 4.68647, 0.0);  
  Point3 Up9(1.93969, 5.32926, 0.0);  
  Point3 Up10(1, 5.67128, 0.0);  


  //Initial Estimate values
  Point3 Pi1(0.0, 0.0, 0.0);
  Point3 Pi2(1.0, 1.0, 0.0);
  Point3 Pi3(2.0, 1.0, 0.0);
  Point3 Pi4(3.0, 1.0, 0.0);
  Point3 Pi5(4.0, 1.0, 0.0);
  Point3 Pi6(5.0, 1.0, 0.0);
  Point3 Pi7(6.0, 1.0, 0.0);
  Point3 Pi8(7.0, 1.0, 0.0);
  Point3 Pi9(8.0, 1.0, 0.0);
  Point3 Pi10(9.0, 1.0, 0.0);




  Rot3 Ri1 = Rot3::rodriguez(0.0, 0.0, PI/2);
  Rot3 Ri2 = Rot3::rodriguez(0.0, 0.0, PI/2);
  Rot3 Ri3 = Rot3::rodriguez(0.0, 0.0, PI/2);
  Rot3 Ri4 = Rot3::rodriguez(0.0, 0.0, PI/2);

  float si1,si2,si3,si4;
  si1=1;
  si2=1;
  si3=1;
  si4=1;




  // Add a prior on the first pose, setting it to the origin
  // A prior factor consists of a mean and a noise model (covariance matrix)

  Moses3 priorMean(ScSO3(s0*R0.matrix()),Pp0.vector()); // prior at origin
  noiseModel::Diagonal::shared_ptr priorNoise = noiseModel::Diagonal::Sigmas((Vector(7) << 0.01, 0.01, 0.01, 0.01, 0.01, 0.01, 0.01));
  graph.add(PriorFactor<Moses3>(1, priorMean, priorNoise));
  



  // 2a. Add odometry factors
  // For simplicity, we will use the same noise model for each odometry factor
  noiseModel::Diagonal::shared_ptr odometryNoise = noiseModel::Diagonal::Sigmas((Vector(7) << 0.1, 0.1, 0.1, 0.1, 0.1, 0.1, 1));
  // Create odometry (Between) factors between consecutive poses

  graph.add(BetweenFactor<Moses3>(1, 2, Moses3(ScSO3(s12*R12.matrix()),Pp12.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(2, 3, Moses3(ScSO3(s23*R23.matrix()),Pp23.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(3, 4, Moses3(ScSO3(s34*R34.matrix()),Pp34.vector()), odometryNoise));

  graph.add(BetweenFactor<Moses3>(4, 5, Moses3(ScSO3(s45*R45.matrix()),Pp45.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(5, 6, Moses3(ScSO3(s56*R56.matrix()),Pp56.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(6, 7, Moses3(ScSO3(s67*R67.matrix()),Pp67.vector()), odometryNoise));

  graph.add(BetweenFactor<Moses3>(7, 8, Moses3(ScSO3(s78*R78.matrix()),Pp78.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(8, 9, Moses3(ScSO3(s89*R89.matrix()),Pp89.vector()), odometryNoise));
  graph.add(BetweenFactor<Moses3>(9, 10, Moses3(ScSO3(s90*R90.matrix()),Pp90.vector()), odometryNoise));



  graphWithGPS = graph;

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas((Vector(3) << 10, 10, 10)); // 10cm std on x,y

if(useUnary){
	
  graphWithGPS.add(boost::make_shared<GPSFactor>(1, Up1, unaryNoise));  
  graphWithGPS.add(boost::make_shared<GPSFactor>(2, Up2, unaryNoise));
  graphWithGPS.add(boost::make_shared<GPSFactor>(3, Up3, unaryNoise));
  
  graphWithGPS.add(boost::make_shared<GPSFactor>(4, Up4, unaryNoise));

  graphWithGPS.add(boost::make_shared<GPSFactor>(5, Up5, unaryNoise));
  graphWithGPS.add(boost::make_shared<GPSFactor>(6, Up6, unaryNoise));
  graphWithGPS.add(boost::make_shared<GPSFactor>(7, Up7, unaryNoise));

  graphWithGPS.add(boost::make_shared<GPSFactor>(8, Up8, unaryNoise));
  graphWithGPS.add(boost::make_shared<GPSFactor>(9, Up9, unaryNoise));
  graphWithGPS.add(boost::make_shared<GPSFactor>(10, Up10, unaryNoise));

  cout << "GPS Points:"<<endl;
  cout << Up1<<endl;
  cout << Up2<<endl;
  cout << Up3<<endl;
  cout << Up4<<endl;
  cout << Up5<<endl;
  cout << Up6<<endl;
  cout << Up7<<endl;
  cout << Up8<<endl;
  cout << Up9<<endl;
  cout << Up10<<endl;
}

  //graph.add(BetweenFactor<Pose2>(1, 2, Pose2(2.0, 0.0, 0.0), odometryNoise));
  //graph.add(BetweenFactor<Pose2>(2, 3, Pose2(2.0, 0.0, 0.0), odometryNoise));

  // 2b. Add "GPS-like" measurements
  // We will use our custom UnaryFactor for this.
  //noiseModel::Diagonal::shared_ptr unaryNoise = noiseModel::Diagonal::Sigmas((Vector(2) << 0.1, 0.1)); // 10cm std on x,y
  //graph.add(boost::make_shared<UnaryFactor>(1, 0.0, 0.0, unaryNoise));
  //graph.add(boost::make_shared<UnaryFactor>(2, 2.0, 0.0, unaryNoise));
  //graph.add(boost::make_shared<UnaryFactor>(3, 4.0, 0.0, unaryNoise));
  
  //graph.print("\nFactor Graph:\n"); // print

  // 3. Create the data structure to hold the initialEstimate estimate to the solution
  // For illustrative purposes, these have been deliberately set to incorrect values
  Values initialEstimate;

  initialEstimate.insert(1, Moses3(ScSO3(si1*Ri1.matrix()),Pi1.vector()));
  initialEstimate.insert(2, Moses3(ScSO3(si2*Ri2.matrix()),Pi2.vector()));
  initialEstimate.insert(3, Moses3(ScSO3(si3*Ri3.matrix()),Pi3.vector()));
  initialEstimate.insert(4, Moses3(ScSO3(si4*Ri4.matrix()),Pi4.vector()));

  initialEstimate.insert(5, Moses3(ScSO3(si2*Ri2.matrix()),Pi5.vector()));
  initialEstimate.insert(6, Moses3(ScSO3(si3*Ri3.matrix()),Pi6.vector()));
  initialEstimate.insert(7, Moses3(ScSO3(si4*Ri4.matrix()),Pi7.vector()));
 
  initialEstimate.insert(8, Moses3(ScSO3(si2*Ri2.matrix()),Pi8.vector()));
  initialEstimate.insert(9, Moses3(ScSO3(si3*Ri3.matrix()),Pi9.vector()));
  initialEstimate.insert(10, Moses3(ScSO3(si4*Ri4.matrix()),Pi10.vector()));







  //initialEstimate.insert(1, Pose2(0.5, 0.0, 0.2));
  //initialEstimate.insert(2, Pose2(2.3, 0.1, -0.2));
  //initialEstimate.insert(3, Pose2(4.1, 0.1, 0.1));
  initialEstimate.print("\nInitial Estimate:\n"); // print

  // 4. Optimize using Levenberg-Marquardt optimization. The optimizer
  // accepts an optional set of configuration parameters, controlling
  // things like convergence criteria, the type of linear system solver
  // to use, and the amount of information displayed during optimization.
  // Here we will use the default set of parameters.  See the
  // documentation for the full set of parameters.
  LevenbergMarquardtOptimizer optimizer(graph, initialEstimate);
  Values result = optimizer.optimize();
  result.print("Final Result:\n");

  LevenbergMarquardtOptimizer optimizerWithGPS(graphWithGPS, initialEstimate);
  Values resultWithGPS = optimizerWithGPS.optimize();
  resultWithGPS.print("FinalWithGPS Result:\n");

  // 5. Calculate and print marginal covariances for all variables
  Marginals marginals(graph, result);
  Marginals marginalsWithGPS(graphWithGPS, resultWithGPS);


if(printCov){
  cout << "x1 covariance:\n" << marginals.marginalCovariance(1) << endl;
  cout << "x2 covariance:\n" << marginals.marginalCovariance(2) << endl;
  cout << "x3 covariance:\n" << marginals.marginalCovariance(3) << endl;
  cout << "x4 covariance:\n" << marginals.marginalCovariance(4) << endl;

  cout << "x5 covariance:\n" << marginals.marginalCovariance(5) << endl;
  cout << "x6 covariance:\n" << marginals.marginalCovariance(6) << endl;
  cout << "x7 covariance:\n" << marginals.marginalCovariance(7) << endl;
  cout << "x8 covariance:\n" << marginals.marginalCovariance(8) << endl;

  cout << "x9 covariance:\n" << marginals.marginalCovariance(9) << endl;
  cout << "x10 covariance:\n" << marginals.marginalCovariance(10) << endl;
}
  return 0;
}
