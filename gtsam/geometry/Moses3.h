/* ----------------------------------------------------------------------------

 * GTSAM Copyright 2010, Georgia Tech Research Corporation, 
 * Atlanta, Georgia 30332-0415
 * All Rights Reserved
 * Authors: Frank Dellaert, et al. (see THANKS for the full author list)

 * See LICENSE for the license information

 * -------------------------------------------------------------------------- */

/**
 *@file  Moses3.h
 *@brief 3D Pose
 */

// \callgraph
#pragma once

#include <gtsam/config.h>

#define Moses3_DEFAULT_COORDINATES_MODE Moses3::EXPMAP

#include <gtsam/geometry/Pose3.h>
#include <gtsam/base/DerivedValue.h>
#include <gtsam/geometry/Point3.h>
#include <gtsam/geometry/Rot3.h>
#include <gtsam/3rdparty/Sophus/sophus/sim3.h> 

using namespace Sophus;

namespace gtsam {

//class Pose2;
// forward declare

/**
 * A 3D similarity (R,t,s) : (Rot3,Point3, double)
 * Vector7 Sim3 retract input (t1,t2,t3,r1,r2,r3,lambda)...scale is exp(lambda) 
 * @addtogroup geometry
 * \nosubgrouping
 */
class GTSAM_EXPORT Moses3: public Sim3, public DerivedValue<Moses3> {
public:
  static const size_t dimension = Sim3::DoF; // 7
 
  /** Pose Concept requirements */
  typedef Rot3 Rotation;
  typedef Point3 Translation;

private:

  Rot3 R_;   ///< Rotation gRp, between global and pose frame
  Point3 t_; ///< Translation gTp, from global origin to pose frame origin
  //double s_; /// scale
  
  //ScSO3 scso3_;
  //Vector3d translation_;
  //ScSO3 scso3_;  //inherited from Sim3
  //Vector3d translation_; //inherited from Sim3
  int ppp;

public:

  /// @name Standard Constructors
  /// @{

  /** Default constructor is origin */
  Moses3() {
  }

  /** Copy constructor from Sim3 
   *  Calls Sim3 base class constructor.
   */
  Moses3(const Sim3& pose) : 
  		Sim3(pose.scso3(), pose.Sim3::translation()){
  }


  /** Copy constructor from ScSO3 and Vector3 
   *  Calls Sim3 base class constructor.
   */
  Moses3(const ScSO3& pose, const Vector3& trans) : 
  		Sim3(pose, trans){
  }

  /** Copy constructor from ScSO3 and Vector3 
   *  Calls Sim3 base class constructor.
   */
  Moses3(const ScSO3& pose, const Point3& trans) : 
  		Sim3(pose, trans.vector()){
  }


  /** Copy constructor 
   *  Calls Sim3 base class constructor.
   */
  Moses3(const Moses3& pose) : 
  		Sim3(pose.scso3(), pose.Sim3::translation()){
  }

  /** Construct from R,t */
  Moses3(const Rot3& R, const Point3& t) :
      Sim3(ScSO3(R.matrix()),t.vector()) {
  }

  /** Construct from s, R,t */
  Moses3(double s, const Rot3& R, const Point3& t) :
       Sim3(s, R.matrix(), t.vector()){
  }


  /** Construct from Pose2 */
  explicit Moses3(const Pose2& pose2);

  /** Constructor from 4*4 matrix 
	* 1st 3 x 3 block is s*R (scso3), 3rd colm is t, last element 4x4 is 1
  	*/
  Moses3(const Matrix &T) :
    	Sim3( ScSO3(T.block(0,0,3,3)), T.col(3).head(3)){
  }

  /// @}
  /// @name Testable
  /// @{
  /// print with optional string
  void print(const std::string& s = "") const;

  /// assert equality up to a tolerance
  bool equals(const Moses3& pose, double tol = 1e-9) const;

  /// @}
  /// @name Group
  /// @{

  /// identity for group operation
  // TODO CHECK what it creates
  static Moses3 identity() {
    return Moses3();
  }

  Pose3 to_Pose3() const{
  	SE3 ans(this->Sim3::to_SE3());
  	Pose3 pose(Rot3(ans.rotation_matrix()), Point3(ans.translation()));
  	return pose;
  }

  /// inverse transformation with derivatives
  // TODO: Test
  //	   Implement Derivatives
  Moses3 inverse(boost::optional<Matrix&> H1 = boost::none) const;


  ///compose this transformation onto another (first *this and then p2)
  // TODO: Verify the jakobians(?) H1 and H2. Is this just the * operator in Sim3 ? Yes.
  Moses3 compose(const Moses3& p2, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const;

  /// compose syntactic sugar
  // call Sim3's * operator.
  Moses3 operator*(const Moses3& T) const {
    return Moses3(static_cast<const Sim3&>(*this)*static_cast<const Sim3&>(T));
  }

  /**
   * Return relative pose between p1 and p2, in p1 coordinate frame
   * as well as optionally the derivatives
   * TODO Test. Derivatives. // between = compose(p2,inverse(p1)); Verify if orger is correct.
   */
  Moses3 between(const Moses3& p2, boost::optional<Matrix&> H1 = boost::none,
      boost::optional<Matrix&> H2 = boost::none) const;

  /// @}
  /// @name Manifold
  /// @{

  /** Enum to indicate which method should be used in Moses3::retract() and
   * Moses3::localCoordinates()
   */
  enum CoordinatesMode {
    EXPMAP, ///< The correct exponential map, computationally expensive.
    FIRST_ORDER ///< A fast first-order approximation to the exponential map.
  };

  /// Dimensionality of tangent space = 6 DOF - used to autodetect sizes
  static size_t Dim() {
    return dimension;
  }

  /// Dimensionality of the tangent space = 6 DOF
  size_t dim() const {
    return dimension;
  }


  /// Retraction from R^6 \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ from R^ with fast first-order approximation to the exponential map
  // TODO Write
  // Moses3 retractFirstOrder(const Vector& d) const;

  /// Retraction from R^6 \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ to Moses3 manifold neighborhood around current pose
  // TODO Test compose(Expmap(xi));
  Moses3 retract(const Vector& d, Moses3::CoordinatesMode mode =
      Moses3_DEFAULT_COORDINATES_MODE) const;



  /// Local 7D coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of Moses3 manifold neighborhood around current pose
  Vector7 localCoordinates(const Moses3& T2, Moses3::CoordinatesMode mode =Moses3_DEFAULT_COORDINATES_MODE) const;

	/// @}
	/// @name Lie Group
	/// @{

	/// Exponential map at identity - create a rotation from canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$
	static Moses3 Expmap(const Vector& xi);

	/// Log map at identity - return the canonical coordinates \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$ of this rotation
	// TODO Remove
	static Vector7 Logmap(const Moses3& p);

	static Vector7 Logmap7(const Moses3& p);

	Vector7 Logmap7() const
	{
	return Moses3::Logmap7(*this);
	}

	Vector7 Logmap() const
	{
	return Moses3::Logmap(*this);
	}


	/**
	* Calculate Adjoint map, transforming a twist in the this pose's (i.e, body) frame to the world spatial frame
	* Ad_pose is 6*6 matrix that when applied to twist xi \f$ [R_x,R_y,R_z,T_x,T_y,T_z] \f$, returns Ad_pose(xi)
	*/
	// TODO Remove
	Matrix7 AdjointMap() const; /// FIXME Not tested - marked as incorrect

	// Why not static ? Coz depends on private members
	Matrix7 AdjointMap7() const;

	/**
	* Apply this pose's AdjointMap Ad_g to a twist \f$ \xi_b \f$, i.e. a body-fixed velocity, transforming it to the spatial frame
	* \f$ \xi^s = g*\xi^b*g^{-1} = Ad_g * \xi^b \f$
	*/
	// Why is this not static ? Coz depends on private members
	Vector Adjoint(const Vector& xi_b) const {return AdjointMap7()*xi_b; } /// FIXME Not tested - marked as incorrect




    /**
     * Compute the [ad(w,v)] operator as defined in [Kobilarov09siggraph], pg 11
     * [ad(w,v)] = [w^, zero3; v^, w^]
     * Note that this is the matrix representation of the adjoint operator for se3 Lie algebra,
     * aka the Lie bracket, and also the derivative of Adjoint map for the Lie group SE3.
     *
     * Let \f$ \hat{\xi}_i \f$ be the se3 Lie algebra, and \f$ \hat{\xi}_i^\vee = \xi_i = [\omega_i,v_i] \in \mathbb{R}^6\f$ be its
     * vector representation.
     * We have the following relationship:
     * \f$ [\hat{\xi}_1,\hat{\xi}_2]^\vee = ad_{\xi_1}(\xi_2) = [ad_{(\omega_1,v_1)}]*\xi_2 \f$
     *
     * We use this to compute the discrete version of the inverse right-trivialized tangent map,
     * and its inverse transpose in the discrete Euler Poincare' (DEP) operator.
     *
     */
     // TODO WRITE. Remove This is the lie bracket.
    static Matrix7 adjointMap(const Vector& xi); //points to adjointMap7
	static Matrix7 adjointMap7(const Vector& xi);
    /**
     * Action of the adjointMap on a Lie-algebra vector y, with optional derivatives
     */
     // TODO Test Derivatives
    static Vector adjoint(const Vector& xi, const Vector& y, boost::optional<Matrix&> H = boost::none); // TODO!! Sign conflict with [xi1,xi2] = - adj(xi1).xi2.

    /**
     * The dual version of adjoint action, acting on the dual space of the Lie-algebra vector space.
     */
     // TODO Test derivatives
    static Vector adjointTranspose(const Vector& xi, const Vector& y, boost::optional<Matrix&> H = boost::none);


    /**
     * Compute the inverse right-trivialized tangent (derivative) map of the exponential map,
     * as detailed in [Kobilarov09siggraph] eq. (15)
     * The full formula is documented in [Celledoni99cmame]
     *    Elena Celledoni and Brynjulf Owren. Lie group methods for rigid body dynamics and
     *    time integration on manifolds. Comput. meth. in Appl. Mech. and Eng., 19(3,4):421� 438, 2003.
     * and in [Hairer06book] in formula (4.5), pg. 84, Lemma 4.2
     *    Ernst Hairer, et al., Geometric Numerical Integration,
     *      Structure-Preserving Algorithms for Ordinary Differential Equations, 2nd edition, Springer-Verlag, 2006.
     */
     //TODO Remove and calculate for Sim3
    static Matrix7 dExpInv_exp(const Vector&  xi);










    /**
     * wedge for Moses3:
     * @param xi 7-dim twist (v,omega,s) where
     *  omega = (wx,wy,wz) 3D angular velocity
     *  v (vx,vy,vz) = 3D velocity
     * @return xihat, 4*4 element of Lie algebra that can be exponentiated
     
    static Matrix wedge(double wx, double wy, double wz, double vx, double vy, double vz) {
      return (Matrix(4,4) <<
          0.,-wz,  wy,  vx,
          wz,  0.,-wx,  vy,
          -wy, wx,   0., vz,
          0.,  0.,  0.,  0.);
    }*/
    static Matrix wedge(const Vector7& xi) {
      return Sim3::hat(xi);
    }


    /// @}
    /// @name Group Action on Point3. There you go.
    /// @{

    /**
     * @brief takes point in Pose coordinates and transforms it to world coordinates
     * @param p point in Pose coordinates
     * @param Dpose optional 3*7 Jacobian wrpt this pose
     * @param Dpoint optional 3*3 Jacobian wrpt point
     * @return point in world coordinates
     */
    Point3 transform_from(const Point3& p,
        boost::optional<Matrix&> Dpose=boost::none, boost::optional<Matrix&> Dpoint=boost::none) const;

    /** syntactic sugar for transform_from */
    inline Point3 operator*(const Point3& p) const { return transform_from(p); }

    /**
     * @brief takes point in world coordinates and transforms it to Pose coordinates
     * @param p point in world coordinates
     * @param Dpose optional 3*7 Jacobian wrpt this pose
     * @param Dpoint optional 3*3 Jacobian wrpt point
     * @return point in Pose coordinates
     */
    Point3 transform_to(const Point3& p,
        boost::optional<Matrix&> Dpose=boost::none, boost::optional<Matrix&> Dpoint=boost::none) const;

    /// @}
    /// @name Standard Interface
    /// @{

    /// get rotation
    const Rot3& rotation() const { 
    	Rot3 rot(this->rotation_matrix());
    	return rot; 
    }

    /// get translation
    
    const Point3& translation() const { 
    	Point3 trans(this->Sim3::translation());
    	return trans; 
    }    

    // This crashes PC with out of memory error. const Point3& translation() const { return Point3(this->translation()); }

    /// get x
    double x() const { return Point3(translation_).x(); }

    /// get y
    double y() const { return Point3(translation_).y(); }

    /// get z
    double z() const { return Point3(translation_).z(); }

    /// get z
    double s() const { return this->scale(); }

    /** convert to 4*4 matrix */
    // TODO Test
    Matrix4 matrix() const;

    /** receives a pose in world coordinates and transforms it to local coordinates */
    Moses3 transform_to(const Moses3& pose) const;

    /**
     * Calculate range to a landmark
     * @param point 3D location of landmark
     * @return range (double)
     */
    double range(const Point3& point,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /**
     * Calculate range to another pose
     * @param pose Other SO(3) pose
     * @return range (double)
     */
    double range(const Moses3& pose,
        boost::optional<Matrix&> H1=boost::none,
        boost::optional<Matrix&> H2=boost::none) const;

    /// @}
    /// @name Advanced Interface
    /// @{

    /**
     * Return the start and end indices (inclusive) of the translation component of the
     * exponential map parameterization
     * @return a pair of [start, end] indices into the tangent space vector
     */
    inline static std::pair<size_t, size_t> translationInterval() { return std::make_pair(0, 2); }

    /**
     * Return the start and end indices (inclusive) of the rotation component of the
     * exponential map parameterization
     * @return a pair of [start, end] indices into the tangent space vector
     */
    static std::pair<size_t, size_t> rotationInterval() { return std::make_pair(3, 5); }

    /// Output stream operator
    GTSAM_EXPORT friend std::ostream &operator<<(std::ostream &os, const Moses3& p);

  private:
    /** Serialization function */
    friend class boost::serialization::access;
    template<class Archive>
    void serialize(Archive & ar, const unsigned int version) {
      ar & boost::serialization::make_nvp("Moses3",
              boost::serialization::base_object<Value>(*this));
      ar & BOOST_SERIALIZATION_NVP(Rot3(this->rotation_matrix()));
      ar & BOOST_SERIALIZATION_NVP(Point3(translation_));//add scale
    }
    /// @}

  };// Moses3 class

  /**
   * wedge for Moses3:
   * @param xi 6-dim twist (omega,v) where
   *  omega = 3D angular velocity
   *  v = 3D velocity
   * @return xihat, 4*4 element of Lie algebra that can be exponentiated
   */
template<>
inline Matrix wedge<Moses3>(const Vector& xi) {
  return Sim3::hat(xi);
}

/**
 * Calculate pose between a vector of 3D point correspondences (p,q)
 * where q = Moses3::transform_from(p) = t + R*p
 */
 /*
typedef std::pair<Point3, Point3> Point3Pair;
GTSAM_EXPORT boost::optional<Moses3> align(const std::vector<Point3Pair>& pairs);
*/
} // namespace gtsam
