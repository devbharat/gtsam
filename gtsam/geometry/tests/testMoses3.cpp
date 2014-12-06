/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 * @file   testMoses3.cpp
 * @brief  Unit tests for Moses3 class
 */

#include <gtsam/geometry/Moses3.h>
#include <gtsam/geometry/Pose2.h>
#include <gtsam/geometry/Pose3.h> 
#include <gtsam/base/lieProxies.h>
#include <gtsam/base/Testable.h>
#include <gtsam/base/numericalDerivative.h>

//#include <gtsam/3rdparty/Eigen/unsupported/Eigen/MatrixFunctions> 

#include <boost/assign/std/vector.hpp> // for operator +=
using namespace boost::assign;

#include <CppUnitLite/TestHarness.h>
#include <cmath>

using namespace std;
using namespace Sophus;
using namespace gtsam;

GTSAM_CONCEPT_TESTABLE_INST(Moses3)
GTSAM_CONCEPT_LIE_INST(Moses3)

/*
static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::rodriguez(0,0,0);
static Point3 Pp(10.0,0.0,0.0);

static Rot3 R1 = Rot3(Point3(0.0, -1.0 , 0.0),Point3(1.0, 0.0 , 0.0),Point3(0.0, 0.0 , 1.0));
static Rot3 R2 = Rot3(Point3(0.0, 1.0 , 0.0),Point3(-1.0, 0.0 , 0.0),Point3(0.0, 0.0 , 1.0));

static Point3 Pp1(0.0,10.0,0.0);
static Point3 Pp2(10.1,0.0,0.0);


static Moses3 Tt(ScSO3(R.matrix()),Pp.vector());
static Sim3 Ss(ScSO3(R.matrix()),Pp.vector());
static Moses3 Ts(Ss);

static Moses3 Tt1(ScSO3(2*R.matrix()),Pp.vector());

static SE3 Se(R.matrix(),Pp.vector());
static SE3 Se1(R1.matrix(),Pp1.vector());

static Pose3 Po(R2,Pp);
static Pose3 Po1(R2,Pp2);

static Moses3 T(R,Point3(3.5,-8.2,4.2));
static Moses3 T2(Rot3::rodriguez(0.3,0.2,0.1),Point3(3.5,-8.2,4.2));
static Moses3 T3(Rot3::rodriguez(-90, 0, 0), Point3(1, 2, 3));
const double tol=1e-5;
*/
static Point3 P(0.2,0.7,-2);
static Rot3 R = Rot3::rodriguez(0.3,0,0);
static Moses3 T(R,Point3(3.5,-8.2,4.2));
static Moses3 T2(Rot3::rodriguez(0.3,0.2,0.1),Point3(3.5,-8.2,4.2));
static Moses3 T3(Rot3::rodriguez(-90, 0, 0), Point3(1, 2, 3));

static Moses3 T4(ScSO3(2*Rot3::rodriguez(-90, 0, 0).matrix()), Point3(1, 2, 3).vector());

const double tol=1e-5;



/* ************************************************************************* */
TEST( Moses3, stream)
{
  Moses3 T;
  std::ostringstream os;
  os << T;
  EXPECT(os.str() == "1 * 1 0 0\n0 1 0\n0 0 1\n\n[0, 0, 0]';\n");
}

/* ************************************************************************* */
TEST( Moses3, constructors)
{
  Moses3 expected(Rot3::rodriguez(0,0,3),Point3(1,2,0));
  Pose2 pose2(1,2,3);
  EXPECT(assert_equal(expected,Moses3(pose2)));
}

/* ************************************************************************* */
TEST( Moses3, retract_expmap)
{
  Moses3 id;
  Vector v = zero(7);
  Rot3 Ro = Rot3::rodriguez(0.0,0,0);
  v(6) = 0.0; //scale = e^this
  EXPECT(assert_equal(Moses3(Ro, Point3()), id.retract(v, Moses3::EXPMAP),1e-2));
  v(3) = 0.3; //rot1
  EXPECT(assert_equal(Moses3(R, Point3()), id.retract(v, Moses3::EXPMAP),1e-2));  
  v(0)=0.2;v(1)=0.394742;v(2)=-2.08998; //translation precalculated for P and R defined globle
  EXPECT(assert_equal(Moses3(R, P),id.retract(v, Moses3::EXPMAP),1e-2));
}

/* ************************************************************************* */
TEST( Moses3, expmap_a_full)
{
  Moses3 id;
  Vector v = zero(7);
  v(3) = 0.3;
  EXPECT(assert_equal(expmap_default<Moses3>(id, v), Moses3(R, Point3())));
  v(0)=0.2;v(1)=0.394742;v(2)=-2.08998;
  EXPECT(assert_equal(Moses3(R, P),expmap_default<Moses3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST( Moses3, expmap_a_full2)
{
  Moses3 id;
  Vector v = zero(7);
  v(3) = 0.3;
  EXPECT(assert_equal(expmap_default<Moses3>(id, v), Moses3(R, Point3())));
  v(0)=0.2;v(1)=0.394742;v(2)=-2.08998;
  EXPECT(assert_equal(Moses3(R, P),expmap_default<Moses3>(id, v),1e-5));
}

/* ************************************************************************* */
TEST(Moses3, expmap_b)
{
  Moses3 p1(Rot3(), Point3(100, 0, 0));
  Moses3 p2 = p1.retract((Vector(7) << 0.0, 0.0, 0.0, 0.0, 0.0, 0.1, 0.0));
  Moses3 expected(Rot3::rodriguez(0.0, 0.0, 0.1), Point3(100.0, 0.0, 0.0));
  EXPECT(assert_equal(expected, p2,1e-2));
}

/* ************************************************************************* */
// test case for screw motion in the plane
namespace screw {
  double a=0.3, c=cos(a), s=sin(a), w=0.3;
  Vector xi = (Vector(7) << w, 0.0, 1.0 ,0.0, 0.0, w, 0.0);
  Rot3 expectedR(c, -s, 0, s, c, 0, 0, 0, 1);
  Point3 expectedT(0.29552, 0.0446635, 1);
  Moses3 expected(expectedR, expectedT);
}


/* ************************************************************************* */
// Checks correct exponential map (Expmap) with brute force matrix exponential
TEST(Moses3, expmap_c_full)
{
  EXPECT(assert_equal(screw::expected, expm<Moses3>(screw::xi),1e-6));
  EXPECT(assert_equal(screw::expected, Moses3::Expmap(screw::xi),1e-6));
}

/* ************************************************************************* */
// assert that T*exp(xi)*T^-1 is equal to exp(Ad_T(xi))
TEST(Pose3, Adjoint_full)
{
  Moses3 expected = T * Moses3::Expmap(screw::xi) * T.inverse();
  Vector xiprime = T.Adjoint(screw::xi);
  EXPECT(assert_equal(expected, Moses3::Expmap(xiprime), 1e-6));

  Moses3 expected2 = T2 * Moses3::Expmap(screw::xi) * T2.inverse();
  Vector xiprime2 = T2.Adjoint(screw::xi);
  EXPECT(assert_equal(expected2, Moses3::Expmap(xiprime2), 1e-6));

  Moses3 expected3 = T3 * Moses3::Expmap(screw::xi) * T3.inverse();
  Vector xiprime3 = T3.Adjoint(screw::xi);
  EXPECT(assert_equal(expected3, Moses3::Expmap(xiprime3), 1e-6));
}


/* ************************************************************************* */
// Tested with scale
TEST(Pose3, expmaps_galore_full)
{
  Vector xi; Moses3 actual;
  xi = (Vector(7) << 0.4, 0.5, 0.6 ,0.1, 0.2, 0.3, 0.1);
  actual = Moses3::Expmap(xi);
  EXPECT(assert_equal(expm<Moses3>(xi), actual,1e-6));
  //EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-6));
  EXPECT(assert_equal(xi, Moses3::Logmap(actual),1e-6));

  xi = (Vector(7) << -0.4, 0.5, -0.6 ,0.1, -0.2, 0.3, 0.1);
  for (double theta=1.0;0.3*theta<=M_PI;theta*=2) {
    Vector txi = xi*theta;
    actual = Moses3::Expmap(txi);
    EXPECT(assert_equal(expm<Moses3>(txi,30), actual,1e-6));
    //EXPECT(assert_equal(Agrawal06iros(txi), actual,1e-6));
    Vector log = Moses3::Logmap(actual);
    EXPECT(assert_equal(actual, Moses3::Expmap(log),1e-6));
    EXPECT(assert_equal(txi,log,1e-6)); // not true once wraps
  }

  // Works with large v as well, but expm needs 10 iterations!
  xi = (Vector(7) << 100.0, 120.0, -60.0 ,0.2, 0.3, -0.8, 0.1);
  actual = Moses3::Expmap(xi);
  EXPECT(assert_equal(expm<Moses3>(xi,10), actual,1e-5));
  //EXPECT(assert_equal(Agrawal06iros(xi), actual,1e-6));
  EXPECT(assert_equal(xi, Moses3::Logmap(actual),1e-6));
}

/* ************************************************************************* */
// Tested with scale
TEST(Moses3, Adjoint_compose_full)
{
  // To debug derivatives of compose, assert that
  // T1*T2*exp(Adjoint(inv(T2),x) = T1*exp(x)*T2
  const Moses3& T1 = T;
  Vector x = (Vector(7) << 0.4, 0.2, 0.8 ,0.1, 0.1, 0.1, 0.0);
  Moses3 expected = T1 * Moses3::Expmap(x) * T2;
  Vector y = T2.inverse().Adjoint(x);
  Moses3 actual = T1 * T2 * Moses3::Expmap(y);
  EXPECT(assert_equal(expected, actual, 1e-6));
}

/* ************************************************************************* */
// Check compose and its pushforward
// NOTE: testing::compose<Pose3>(t1,t2) = t1.compose(t2)  (see lieProxies.h)
// Tested with scale
TEST( Moses3, compose )
{

  const Moses3& T6 = T2;
  gtsam::Matrix actual = (T6*T6).matrix();
  
  gtsam::Matrix expected = T6.matrix()*T6.matrix();
  EXPECT(assert_equal(actual,expected,1e-8));

  gtsam::Matrix actualDcompose1, actualDcompose2;
  T6.compose(T6, actualDcompose1, actualDcompose2);

  gtsam::Matrix numericalH1 = numericalDerivative21(testing::compose<Moses3>, T6, T6);
  EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
  EXPECT(assert_equal(T6.inverse().AdjointMap(),actualDcompose1,5e-3));

  gtsam::Matrix numericalH2 = numericalDerivative22(testing::compose<Moses3>, T6, T6);
  EXPECT(assert_equal(numericalH2,actualDcompose2,1e-4));
}

/* ************************************************************************* */
// Check compose and its pushforward, another case 
//Tested with scale
TEST( Moses3, compose2 )
{
  const Moses3& T1 = T;
  gtsam::Matrix actual = (T1*T2).matrix();
  gtsam::Matrix expected = T1.matrix()*T2.matrix();
  EXPECT(assert_equal(actual,expected,1e-8));

  gtsam::Matrix actualDcompose1, actualDcompose2;
  T1.compose(T2, actualDcompose1, actualDcompose2);

  gtsam::Matrix numericalH1 = numericalDerivative21(testing::compose<Moses3>, T1, T2);
  EXPECT(assert_equal(numericalH1,actualDcompose1,5e-3));
  EXPECT(assert_equal(T2.inverse().AdjointMap(),actualDcompose1,5e-3));

  gtsam::Matrix numericalH2 = numericalDerivative22(testing::compose<Moses3>, T1, T2);
  EXPECT(assert_equal(numericalH2,actualDcompose2,1e-5));
}


/* ************************************************************************* */
// Tested with scale
TEST( Moses3, inverse)
{
  const Moses3& T5 = T4;
  gtsam::Matrix actualDinverse;
  gtsam::Matrix actual = T5.inverse(actualDinverse).matrix();
  gtsam::Matrix expected = inverse(T5.matrix());
  EXPECT(assert_equal(actual,expected,1e-8));

  gtsam::Matrix numericalH = numericalDerivative11(testing::inverse<Moses3>, T5);
  EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
  EXPECT(assert_equal(-T5.AdjointMap(),actualDinverse,5e-3));
}


/* ************************************************************************* */
TEST( Moses3, inverseDerivatives2)
{
  Rot3 R = Rot3::rodriguez(0.3,0.4,-0.5);
  Point3 t(3.5,-8.2,4.2);
  Moses3 T(R,t);

  gtsam::Matrix numericalH = numericalDerivative11(testing::inverse<Moses3>, T);
  gtsam::Matrix actualDinverse;
  T.inverse(actualDinverse);
  EXPECT(assert_equal(numericalH,actualDinverse,5e-3));
  EXPECT(assert_equal(-T.AdjointMap(),actualDinverse,5e-3));
}

/* ************************************************************************* */
TEST( Moses3, compose_inverse)
{
  gtsam::Matrix actual = (T*T.inverse()).matrix();
  gtsam::Matrix expected = eye(4,4);
  EXPECT(assert_equal(actual,expected,1e-8));
}

/* ************************************************************************* */
// Jacobian Scale hacked. Not correct.
Point3 transform_from_(const Moses3& pose, const Point3& point) { return pose.transform_from(point); }


TEST( Moses3, Dtransform_from1_a)
{
  gtsam::Matrix actualDtransform_from1;
  T.transform_from(P, actualDtransform_from1, boost::none);
  gtsam::Matrix numerical = numericalDerivative21(transform_from_,T,P);
  EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Moses3, Dtransform_from1_b)
{
  Moses3 origin(ScSO3(2*Rot3::rodriguez(1.0,0.0,1.0).matrix()),Point3(0.0,1.0,0.0));
  Point3 Pee(1.0,10.0,1.0);
  gtsam::Matrix actualDtransform_from1;
  origin.transform_from(Pee, actualDtransform_from1, boost::none);
  gtsam::Matrix numerical = numericalDerivative21(transform_from_,origin,Pee);
  EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}

TEST( Moses3, Dtransform_from1_c)
{
  Point3 origin;
  Moses3 T0(R,origin);
  gtsam::Matrix actualDtransform_from1;
  T0.transform_from(P, actualDtransform_from1, boost::none);
  gtsam::Matrix numerical = numericalDerivative21(transform_from_,T0,P);
  EXPECT(assert_equal(numerical,actualDtransform_from1,1e-8));
}





















/* ************************************************************************* */
bool moses3bracket_tests()
{
  bool failed = false;
  vector<Vector7> vecs;
  Vector7 tmp;
  tmp << 0,0,0,0,0,0,0;
  vecs.push_back(tmp);
  tmp << 1,0,0,0,0,0,0;
  vecs.push_back(tmp);
  tmp << 0,1,0,1,0,0,0.1;
  vecs.push_back(tmp);
  tmp << 0,0,1,0,1,0,0.1;
  vecs.push_back(tmp);
  tmp << -1,1,0,0,0,1,-0.1;
  vecs.push_back(tmp);
  tmp << 20,-1,0,-1,1,0,-0.1;
  vecs.push_back(tmp);
  tmp << 30,5,-1,20,-1,0,2;
  vecs.push_back(tmp);
  for (size_t i=0; i<vecs.size(); ++i)
  {
    Vector7 resDiff = vecs[i] - Moses3::vee(Moses3::hat(vecs[i]));
    if (resDiff.norm()>SMALL_EPS)
    {
      cerr << "Hat-vee Test" << endl;
      cerr  << "Test case: " << i <<  endl;
      cerr << resDiff.transpose() << endl;
      cerr << endl;
      failed = true;
    }

    for (size_t j=0; j<vecs.size(); ++j)
    {
      Vector7 res1 = Moses3::lieBracket(vecs[i],vecs[j]);
      Matrix4 hati = Moses3::hat(vecs[i]);
      Matrix4 hatj = Moses3::hat(vecs[j]);

      Vector7 res2 = Moses3::vee(hati*hatj-hatj*hati);
      Vector7 resDiff = res1-res2;
      if (resDiff.norm()>SMALL_EPS)
      {
        cerr << "Sim3 Lie Bracket Test" << endl;
        cerr  << "Test case: " << i << ", " <<j<< endl;
        cerr << vecs[i].transpose() << endl;
        cerr << vecs[j].transpose() << endl;
        cerr << resDiff.transpose() << endl;
        cerr << endl;
        failed = true;
      }
    }


    /*
    Vector7 omega = vecs[i];
    Matrix4 exp_x = Moses3::exp(omega).matrix();
    Matrix4 expmap_hat_x = (Moses3::hat(omega)).exp();
    Matrix4 DiffR = exp_x-expmap_hat_x;
    double nrm = DiffR.norm();

    if (isnan(nrm) || nrm>SMALL_EPS)
    {
      cerr << "expmap(hat(x)) - exp(x)" << endl;
      cerr  << "Test case: " << i << endl;
      cerr << exp_x <<endl;
      cerr << expmap_hat_x <<endl;
      cerr << DiffR <<endl;
      cerr << endl;
      failed = true;
    }*/
  }
  return failed;
}

/* ********************************************************************** */

bool Moses3explog_tests()
{
  double pi = 3.14159265;
  vector<Moses3> omegas;
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0.2, 0.5, 0.0,1.)),Vector3(0,0,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0.2, 0.5, -1.0,1.1)),Vector3(10,0,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0., 0., 0.,1.1)),Vector3(0,100,5)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0., 0., 0.00001, 0.)),Vector3(0,0,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0., 0., 0.00001, 0.0000001)),Vector3(1,-1.00000001,2.0000000001)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0., 0., 0.00001, 0)),Vector3(0.01,0,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(pi, 0, 0,0.9)),Vector3(4,-5,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0.2, 0.5, 0.0,0)),Vector3(0,0,0))
                   *Moses3(ScSO3::exp(Vector4(pi, 0, 0,0)),Vector3(0,0,0))
                   *Moses3(ScSO3::exp(Vector4(-0.2, -0.5, -0.0,0)),Vector3(0,0,0)));
  omegas.push_back(Moses3(ScSO3::exp(Vector4(0.3, 0.5, 0.1,0)),Vector3(2,0,-7))
                   *Moses3(ScSO3::exp(Vector4(pi, 0, 0,0)),Vector3(0,0,0))
                   *Moses3(ScSO3::exp(Vector4(-0.3, -0.5, -0.1,0)),Vector3(0,6,0)));

  bool failed = false;

  for (size_t i=0; i<omegas.size(); ++i)
  {
    Matrix4 R1 = omegas[i].matrix();
    Matrix4 R2 = Moses3::Expmap(omegas[i].Logmap7()).matrix();
    Matrix4 DiffR = R1-R2;
    double nrm = DiffR.norm();

    // ToDO: Force Sim3 to be more accurate!
    if (isnan(nrm) || nrm>SMALL_EPS)
    {
      cerr << "Sim3 - exp(log(Sim3))" << endl;
      cerr  << "Test case: " << i << endl;
      cerr << DiffR <<endl;
      cerr << endl;
      failed = true;
    }
  }
  for (size_t i=0; i<omegas.size(); ++i)
  {
    Vector3 p(1,2,4);
    Matrix4 T = omegas[i].matrix();
    Vector3 res1 = static_cast<const Sim3 &>(omegas[i])*p;
    Vector3 res2 = T.topLeftCorner<3,3>()*p + T.topRightCorner<3,1>();

    double nrm = (res1-res2).norm();

    if (isnan(nrm) || nrm>SMALL_EPS)
    {
      cerr << "Transform vector" << endl;
      cerr  << "Test case: " << i << endl;
      cerr << (res1-res2) <<endl;
      cerr << endl;
      failed = true;
    }
  }

  for (size_t i=0; i<omegas.size(); ++i)
  {
    Matrix4 q = omegas[i].matrix();
    Matrix4 inv_q = omegas[i].inverse().matrix();
    Matrix4 res = q*inv_q ;
    Matrix4 I;
    I.setIdentity();

    double nrm = (res-I).norm();

    if (isnan(nrm) || nrm>SMALL_EPS)
    {
      cerr << "Inverse" << endl;
      cerr  << "Test case: " << i << endl;
      cerr << (res-I) <<endl;
      cerr << endl;
      failed = true;
    }
  }
  return failed;
}


/* ************************************************************************* */
int main(){ 
  TestResult tr;
/*
  Vector7d tmp;
  tmp << 0,0,0,0,0,0,0;


  cout << Tt;
  cout << Tt.to_Pose3();

  cout << Tt1;
  cout << Tt1.to_Pose3();

  cout << Tt1.inverse();
  cout << Tt1.inverse().to_Pose3();

  cout << Tt1*Tt1.inverse();
  cout << (Tt1*Tt1.inverse()).to_Pose3();



  cout << Se;
  cout << Se1;
  cout << (Se*Se1);

  cout << Po;
  cout << Po1;

  cout << Po1.between(Po);
  cout << Po1.compose(Po.inverse());

*/

  cout << T4.scale()<<endl;

  bool failed = Moses3explog_tests();
  cout << failed<<endl;

  failed = moses3bracket_tests();
  cout << failed<<endl;

  return TestRegistry::runAllTests(tr);}
/* ************************************************************************* */
