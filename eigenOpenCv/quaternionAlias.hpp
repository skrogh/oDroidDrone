#ifndef _QUATERNION_ALIAS_H_
#define _QUATERNION_ALIAS_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen {

template<typename Scalar, int Options = AutoAlign> class QuaternionAlias;

namespace internal {
template<typename _Scalar,int _Options>
struct traits<QuaternionAlias<_Scalar,_Options> >
{
	typedef QuaternionAlias<_Scalar,_Options> PlainObject;
	typedef _Scalar Scalar;
	typedef Matrix<_Scalar,4,1,_Options> Coefficients;
	enum{
		IsAligned = internal::traits<Coefficients>::Flags & AlignedBit,
		Flags = IsAligned ? (AlignedBit | LvalueBit) : LvalueBit
	};
};
}

template<typename _Scalar, int _Options>
class QuaternionAlias : public Quaternion<_Scalar,_Options>
{

	typedef Quaternion<_Scalar,_Options> Base;
	enum { IsAligned = internal::traits<QuaternionAlias>::IsAligned };

public:
	typedef _Scalar Scalar;
	typedef Matrix<Scalar,3,1> Vector3;
	typedef Matrix<Scalar,3,3> Matrix3;

	typedef typename internal::traits<QuaternionAlias>::Coefficients Coefficients;
	typedef typename Base::AngleAxisType AngleAxisType;

	inline QuaternionAlias : Quaternion<Scalar, _Options>();
/*
	inline QuaternionAlias : Quaternion<_Scalar,_Options>() {}

	inline QuaternionAlias(const Scalar& w, const Scalar& x, const Scalar& y, const Scalar& z) : Quaternion(w, x, y, z);

	inline QuaternionAlias(const Scalar* data) : Quaternion(data);

	template<class Derived> EIGEN_STRONG_INLINE QuaternionAlias(const QuaternionBase<Derived>& other) : Quaternion(other);

	explicit inline QuaternionAlias(const AngleAxisType& aa) : Quaternion(aa);

	template<typename Derived>
	explicit inline QuaternionAlias(const MatrixBase<Derived>& other) : Quaternion(other);

	template<typename OtherScalar, int OtherOptions>
	explicit inline QuaternionAlias(const QuaternionAlias<OtherScalar, OtherOptions>& other)
	{ m_coeffs = other.coeffs().template cast<Scalar>(); }
*/
	template<class OtherDerived> EIGEN_STRONG_INLINE QuaternionAlias<Scalar> operator* (const QuaternionAlias<OtherDerived>& other) const
	{
	
	
	EIGEN_STATIC_ASSERT((internal::is_same<typename Scalar::Scalar, typename OtherDerived::Scalar>::value),
	YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
	
	return internal::quat_product<Architecture::Target, Scalar, OtherDerived,
			typename internal::traits<Scalar>::Scalar,
			internal::traits<Scalar>::IsAligned && internal::traits<OtherDerived>::IsAligned>::run(other, *this);
	}
	
	//template<class OtherDerived> EIGEN_STRONG_INLINE Derived& operator*= (const QuaternionBase<OtherDerived>& q) 
	Matrix3 toRotationMatrix() const 
	{
		return this->conjugate().Base::toRotationMatrix();
	}
	EIGEN_STRONG_INLINE Vector3 _transformVector(const Vector3& v) const
	{
		return this->conjugate()._transformVector( v );
	}

};

/*

template <class Derived>
template <class OtherDerived>
EIGEN_STRONG_INLINE QuaternionAlias<typename internal::traits<Derived>::Scalar>
QuaternionAlias<Derived>::operator* (const QuaternionAlias<OtherDerived>& other) const
{
	EIGEN_STATIC_ASSERT((internal::is_same<typename Derived::Scalar, typename OtherDerived::Scalar>::value),
	YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)

	return internal::quat_product<Architecture::Target, Derived, OtherDerived,
			typename internal::traits<Derived>::Scalar,
			internal::traits<Derived>::IsAligned && internal::traits<OtherDerived>::IsAligned>::run(other, *this);
}*/
}
#endif//_QUATERNION_ALIAS_H_