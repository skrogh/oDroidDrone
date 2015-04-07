#ifndef _QUATERNION_ALIAS_H_
#define _QUATERNION_ALIAS_H_
#include <Eigen/Dense>
#include <Eigen/Geometry>

namespace Eigen {

template<typename _Scalar, int _Options>
class QuaternionAlias : public Quaternion<_Scalar,_Options>
{
/*
	typedef Quaternion<_Scalar,_Options> Base;
	enum { IsAligned = internal::traits<QuaternionAlias>::IsAligned };
public:
	template<class OtherDerived> EIGEN_STRONG_INLINE QuaternionAlias<Scalar> operator* (const QuaternionAlias<OtherDerived>& q) const
	{
		EIGEN_STATIC_ASSERT((internal::is_same<typename Derived::Scalar, typename OtherDerived::Scalar>::value),
		YOU_MIXED_DIFFERENT_NUMERIC_TYPES__YOU_NEED_TO_USE_THE_CAST_METHOD_OF_MATRIXBASE_TO_CAST_NUMERIC_TYPES_EXPLICITLY)
		return internal::quat_product<Architecture::Target, Derived, OtherDerived,
				typename internal::traits<Derived>::Scalar,
				internal::traits<Derived>::IsAligned && internal::traits<OtherDerived>::IsAligned>::run(other, *this);
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
*/
};

}
#endif//_QUATERNION_ALIAS_H_