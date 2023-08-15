#ifndef JMATH_H
#define JMATH_H


#define _EPSILON_	0.0001
#define _PI_		3.1415926535897932384626433832795028
#define NULL	0

#ifndef SWAPARGS_FUNCTION
#define SWAPARGS_FUNCTION

template <class TYPE> void jm_Swap(TYPE& a, TYPE& b)
{
	TYPE temp;
	temp = a;
	a = b;
	b = temp;
}

#endif SWAPARGS_FUNCTION

class JVector;
class JMatrix;
class JQuaternion;


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JMath Vector
#ifndef JVECTOR_H
#define JVECTOR_H

class JVector
{
public:
	double m_p[3];

public:
	JVector(void);
	JVector(double x, double y, double z);
	JVector(const JVector& Cpy);
	~JVector(void);

	void Set(double x, double y, double z);
	bool IsZero(double epsilon = _EPSILON_) const;
	double distance(const JVector& v1) const;
	double dotProduct(const JVector& v1) const;
	JVector crossProduct(const JVector& v1) const;
	void normalize(void);
	double magnitude(void) const;
	double getdistance(void);
	double norm(void) const;
	double norm_1(void) const;
	double norm_infinite(void) const;
	int getLongestAxisComponent(void) const;
	int getSecondLongestAxisComponent(void) const;
	int getShortestAxisComponent(void) const;

public:
	const double& operator[](int n) const;
	double& operator[](int n);

	operator const double* () const { return &m_p[0]; }
	operator double* () { return &m_p[0]; }

	JVector& operator = (const JVector& v);
	JVector& operator = (const JMatrix& m);
	JVector operator + (const JVector& v) const;
	JVector operator - (const JVector& v) const;
	JVector operator * (const JVector& v) const;
	JVector operator / (const JVector& v) const;

	JVector operator + (const double& m) const;
	JVector operator - (const double& m) const;
	JVector operator * (const double& m) const;
	JVector operator / (const double& m) const;

	JVector& operator -= (const JVector& v1);
	JVector& operator += (const JVector& v1);
	JVector& operator *= (const JVector& v1);
	JVector& operator /= (const JVector& v1);

	JVector& operator += (double s);
	JVector& operator -= (double s);
	JVector& operator *= (double s);
	JVector& operator /= (double s);

	friend JVector operator + (const double& m, JVector& v);
	friend JVector operator - (const double& m, JVector& v);
	friend JVector operator * (const double& m, JVector& v);
	friend JVector operator / (const double& m, JVector& v);

	bool operator == (const JVector& v);
	bool operator != (const JVector& v);
	bool operator <= (const JVector& v);
	bool operator >= (const JVector& v);
	bool operator > (const JVector& v);
	bool operator < (const JVector& v);
};

bool jm_Vec_IsZero(const JVector& v1, double epsilon = 0.0);
double jm_Vec_distance(const JVector& v1, const JVector& v2);
double jm_Vec_dotProduct(const JVector& v1, const JVector& v2);
JVector jm_Vec_crossProduct(const JVector& v1, const JVector& v2);
JVector jm_Vec_normalize(const JVector& v1);
double jm_Vec_magnitude(const JVector& v1);
double jm_Vec_norm(const JVector& v1);
double jm_Vec_norm_1(const JVector& v1);
double jm_Vec_norm_infinite(const JVector& v1);

#endif
// JMath Vector
//////////////////////////////////////////////////////////////////////////////////////////////////////////////



//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JMath Matrix
#ifndef JMATRIX_H
#define JMATRIX_H

class JMatrix {
public:
	double* m_p;
private:
	int m_nRow;
	int m_nCol;
	char m_cType;

public:
	JMatrix(void);
	JMatrix(int nRow, int nCol);
	JMatrix(const JMatrix& Cpy);
	~JMatrix(void);

private:
	void MakeMatrix(void);
	void ReleaseMatrix(void);

public:
	char GetMType(void);
	int GetNRow(void) const;
	int GetNCol(void) const;

	bool IsError(void);
	bool IsIdentity(void);

	JMatrix Transpose(void);
	JMatrix& Inverse(void);

	JMatrix GetTranspose(void);
	JMatrix GetInverse(void);

	JMatrix cross(JMatrix m);

	void setZero(void);
	void setIdentity(void);


public:
	const double* operator[](int n) const;
	double* operator[](int n);

	operator const double* () const { return &m_p[0]; }
	operator double* () { return &m_p[0]; }

	double& operator()(const int i, const int j);
	const double& operator()(const int i, const int j) const;
	double& operator()(const int i);
	const double& operator()(const int i) const;


	JMatrix& operator = (JMatrix& m);
	JMatrix operator + (JMatrix& m) const;
	JMatrix operator - (JMatrix& m) const;
	JMatrix operator * (JMatrix& m) const;
	//JMatrix & operator * (JMatrix & m) const;
	JMatrix operator / (JMatrix& m) const;
	JVector operator * (JVector& v) const;

	JMatrix operator + (const double& n) const;
	JMatrix operator - (const double& n) const;
	JMatrix operator * (const double& n) const;
	JMatrix operator / (const double& n) const;

	friend JMatrix operator + (double& n, const JMatrix& m);
	friend JMatrix operator - (double& n, const JMatrix& m);
	friend JMatrix operator * (double& n, const JMatrix& m);
	friend JMatrix operator / (double& n, const JMatrix& m);

	JMatrix& operator += (JMatrix& m);
	JMatrix& operator -= (JMatrix& m);
	JMatrix& operator *= (JMatrix& m);
	JMatrix& operator /= (JMatrix& m);

	JMatrix& operator += (const double& n);
	JMatrix& operator -= (const double& n);
	JMatrix& operator *= (const double& n);
	JMatrix& operator /= (const double& n);

	//////////////////////////////////////////////////////////////////////////
	// R Matrix or H Matrix Mode Function Set
	JMatrix(char nType);

	bool SetDegree(double Rx, double Ry, double Rz);
	bool SetRadian(double Rx, double Ry, double Rz);

	bool GetDegree(double& Rx, double& Ry, double& Rz);
	bool GetRadian(double& Rx, double& Ry, double& Rz);

	bool Set_nsa(JVector& n, JVector& s, JVector& a);
	bool Set_nsa(double* n, double* s, double* a);
	bool Get_nsa(JVector& n, JVector& s, JVector& a);
	bool Get_nsa(double* n, double* s, double* a);

	bool SkewSymetric(JVector& v);
	bool SkewSymetric(double v[3]);
	void InitializeRMatrix(void);

	JVector operator * (const JVector& v);

	void InitializeHMatrix(void);
	bool SetTranslate(double x, double y, double z);
	bool SetTranslate(JVector& v);
	bool GetTranslate(double& x, double& y, double& z);
	bool GetTranslate(JVector& v);

	bool SetRMatrix(JMatrix& R);
	bool GetRMatrix(JMatrix& R);

	bool SetMatrix(int nRow, int nCol);
	bool SetHomer(JVector& v, JMatrix& R);
	bool SetHomer(double x, double y, double z, JMatrix& R);
	bool SetHomerDegree(JVector& v, double Rx, double Ry, double Rz);
	bool SetHomerRadian(JVector& v, double Rx, double Ry, double Rz);
	bool SetHomerDegree(double x, double y, double z, double Rx, double Ry, double Rz);
	bool SetHomerRadian(double x, double y, double z, double Rx, double Ry, double Rz);

	bool GetHomer(JVector& v, JMatrix& R);
	bool GetHomer(double& x, double& y, double& z, JMatrix& R);
	bool GetHomerDegree(JVector& v, double& Rx, double& Ry, double& Rz);
	bool GetHomerRadian(JVector& v, double& Rx, double& Ry, double& Rz);
	bool GetHomerDegree(double& x, double& y, double& z, double& Rx, double& Ry, double& Rz);
	bool SetHomerRadian(double& x, double& y, double& z, double& Rx, double& Ry, double& Rz);
	//////////////////////////////////////////////////////////////////////////
	JMatrix GetTopLeftCorner(int row, int col);
	JMatrix GetTopRightCorner(int row, int col);
	JMatrix GetBottomLeftCorner(int row, int col);
	JMatrix GetBottomRightCorner(int row, int col);
	JMatrix GetRightCols(int col);
	JMatrix GetLeftCols(int col);
	JMatrix GetTopRows(int row);
	JMatrix GetBottomRows(int row);
	JMatrix GetCol(int ncol);
	JMatrix GetRow(int nrow);
	bool SetTopLeftCorner(JMatrix & A);
	bool SetTopRightCorner(JMatrix & A);
	bool SetBottomLeftCorner(JMatrix & A);
	bool SetBottomRightCorner(JMatrix & A);
	bool SetRightCols(JMatrix & A);
	bool SetLeftCols(JMatrix & A);
	bool SetTopRows(JMatrix & A);
	bool SetBottomRows(JMatrix & A);
	bool SetCol(int ncol, JMatrix & A);
	bool SetRow(int nrow, JMatrix & A);
	double norm(void);
	//////////////////////////////////////////////////////////////////////////
};


#endif JMATRIX_H
// JMath Matrix
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


//////////////////////////////////////////////////////////////////////////////////////////////////////////////
// JMath Vector
#ifndef JQUATERNION_H
#define JQUATERNION_H

class JQuaternion
{
	double m_s;
	JMatrix m_v;

public:
	JQuaternion(void);
	JQuaternion(double r, JMatrix v);
	JQuaternion(JMatrix R);
	//JQuaternion(const JQuaternion &Cpy);
	~JQuaternion(void);

public:
	void SetQuaternion(double theta, JMatrix axis);

public:
	// 	operator const double*() const { return &m_s; }
	//     operator double*() { return &m_s; }


	JMatrix& v() { return m_v; }
	const JMatrix& v() const { return m_v; }

	double& s() { return m_s; }
	const double s() const { return m_s; }

	void scale(double s);
	void normalize();

	double norm(void);

	void GetRotationMatrix(double R[3][3]) const;
	void SetRotationMatrix(const double R[3][3]);
	void GetRotationMatrix(JMatrix& rotMat) const;
	void SetRotationMatrix(const JMatrix& rotMat);

	// Converts to/from axis/angle representation and quaternion.
	// Angle is specified in radians
	void GetAxisAngle(JMatrix& axis, double& angle);
	void SetAxisAngle(const JMatrix& axis, double angle);


	JQuaternion& operator = (JQuaternion& q);
	JQuaternion operator * (const JQuaternion& q);
	JQuaternion operator * (const JMatrix& v);
	JQuaternion operator * (const double& n);
	friend JQuaternion operator * (const JMatrix& v, JQuaternion& q);
	friend JQuaternion operator * (const double& n, JQuaternion& q);
};

#endif
// JMath Vector
//////////////////////////////////////////////////////////////////////////////////////////////////////////////


#endif JQUATERNION_H