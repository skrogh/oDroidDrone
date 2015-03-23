Matrix3d crossMat( const Vector3d& v ){
	Matrix3f m;
	m <<     0, -v(2),  v(1)
	      v(2),     0, -v(0)
	     -v(1),  v(0),     0;
	return m;
}


void MSCKF::propagateState( double a_m[3], double g_m[3] ) {

}
