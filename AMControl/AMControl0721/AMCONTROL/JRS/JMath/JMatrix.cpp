
#include "TcPch.h"
#pragma hdrstop

#include "../include/JRobotSystemDefine.h"


#include "JMath.h"

bool gauss_inverse(int n, double *a, double *x);



JMatrix::JMatrix(void)
	: m_p(NULL)
	, m_nRow(0)
	, m_nCol(0)
	, m_cType('N')
{

}

JMatrix::JMatrix(int nRow, int nCol)
	: m_nRow(nRow)
	, m_nCol(nCol)
	, m_cType('N')
{
	MakeMatrix();
}

JMatrix::JMatrix(const JMatrix& Cpy)
{
	m_nCol = Cpy.m_nCol;
	m_nRow = Cpy.m_nRow;
	m_cType = Cpy.m_cType;
	MakeMatrix();

	int i, j;
	for (i = 0; i < m_nRow; i++) {
		for (j = 0; j < m_nCol; j++) {
			(*this)[i][j] = Cpy[i][j];
		}
	}
}

JMatrix::~JMatrix(void)
{
	ReleaseMatrix();
}


void JMatrix::MakeMatrix(void)
{
	m_p = new double[m_nCol * m_nRow];
	setZero();
}

void JMatrix::ReleaseMatrix(void)
{
	m_nCol = 0;
	m_nRow = 0;
	m_cType = NULL;
	if (m_p)
		delete[] m_p;
	m_p = NULL;
}

char JMatrix::GetMType(void)
{
	return m_cType;
}

int JMatrix::GetNRow(void) const
{
	return m_nRow;
}

int JMatrix::GetNCol(void) const
{
	return m_nCol;
}

bool JMatrix::IsError(void)
{
	return (m_p == NULL);
}

bool JMatrix::IsIdentity(void)
{
	if (m_nCol != m_nRow)
		return false;

	bool bCheck = false;
	int i;
	for (i = 0; i < m_nRow; i++) {
		if (fabs_((*this)[i][i] - 1.0) >= _EPSILON_)
			return false;
	}
	return true;
}

//JMatrix & JMatrix::Transpose(void)
//{
//	int i, j;
//
//	if( m_nRow == 1 || m_nCol == 1 ) 
//	{
//		// 1 Dimension Matrix
//		jm_Swap(m_nRow, m_nCol);
//	}
//	else if( m_nRow == m_nCol ) 
//	{
//		// Square Matrix
//		for( i = 0; i < m_nRow; i++ ) {
//			for( j = i; j < m_nCol; j++ ) {
//				if( i == j )
//					continue;
//				jm_Swap((*this)[i][j], (*this)[j][i]);
//			}
//		}
//	} 
//	else 
//	{
//		// No Square Matrix
//		double * pTemp;
//		pTemp = new double[m_nCol * m_nRow];
//		for( i = 0; i < m_nRow; i++ ) {
//			for( j = 0; j < m_nCol; j++ ) {
//				pTemp[i + j * m_nRow] = (*this)[i][j];
//			}
//		}
//		jm_Swap(m_p, pTemp);
//		jm_Swap(m_nCol, m_nRow);
//		delete [] pTemp;
//	}
//
//	return (*this);
//}

JMatrix JMatrix::Transpose(void)
{
	int i, j;

	JMatrix M_Swap(m_nCol, m_nRow);

	for (i = 0; i < m_nCol; i++)
	{
		for (j = 0; j < m_nRow; j++)
		{
			M_Swap(i, j) = (*this)(j, i);
		}
	}

	return M_Swap;
}



JMatrix& JMatrix::Inverse(void)
{
	// 	if( m_cType == 'R' ) {
	// 		Transpose();
	// 		return (*this);
	// 	}

	if (m_nCol != m_nRow)
		return (*this);

	JMatrix Temp(*this);
	JMatrix original(*this);
	JMatrix inverse(*this);

	if (gauss_inverse(m_nCol, original.m_p, inverse.m_p) == false)
		return (*this);


	Temp = (*this) * inverse;
	if (Temp.IsIdentity() == false)
		return (*this);

	for (int i = 0; i < m_nRow; i++)
	{
		for (int j = 0; j < m_nCol; j++)
		{
			(*this)[i][j] = inverse[i][j];
		}
	}

	return (*this);
}

JMatrix JMatrix::GetTranspose(void)
{
	JMatrix result(*this);
	return result.Transpose();
}

JMatrix JMatrix::GetInverse(void)
{
	JMatrix result(*this);
	return result.Inverse();
}

JMatrix JMatrix::cross(JMatrix m)
{
	JMatrix result(3, 1);
	result.setZero();

	if (m_nRow == 3 && m.GetNRow() == 3)
	{
		result(0, 0) = (*this)(1) * m(2) - (*this)(2) * m(1);
		result(1, 0) = (*this)(2) * m(0) - (*this)(0) * m(2);
		result(2, 0) = (*this)(0) * m(1) - (*this)(1) * m(0);
	}
	return result;
}

void JMatrix::setZero(void)
{
	for (int i = 0; i < (m_nCol * m_nRow); i++)
	{
		m_p[i] = 0.0;
	}
	//memset(m_p, NULL, sizeof(double) * (m_nCol * m_nRow));
}

void JMatrix::setIdentity(void)
{
	if (m_nCol != m_nRow)
		return;

	setZero();

	int i;
	for (i = 0; i < m_nRow; i++) {
		(*this)[i][i] = 1.0;
	}
}


const double* JMatrix::operator[](int n) const
{
	if (m_nRow == 1 || m_nCol == 1)
		return &m_p[n];
	return &m_p[n * m_nCol];
}

double* JMatrix::operator[](int n)
{
	if (m_nRow == 1 || m_nCol == 1)
		return &m_p[n];
	return &m_p[n * m_nCol];
}


double& JMatrix::operator()(const int i, const int j)
{
	return m_p[i * m_nCol + j];
}

const double& JMatrix::operator()(const int i, const int j) const
{
	return m_p[i * m_nCol + j];
}
double& JMatrix::operator()(const int i)
{
	return m_p[i];
}

const double& JMatrix::operator()(const int i) const
{
	return m_p[i];
}

JMatrix& JMatrix::operator = (JMatrix& m)
{
	ReleaseMatrix();

	m_nRow = m.GetNRow();
	m_nCol = m.GetNCol();
	m_cType = m.m_cType;
	MakeMatrix();

	for (int i = 0; i < m_nRow * m_nCol; i++)
	{
		m_p[i] = m.m_p[i];
	}

	return *this;
}

JMatrix JMatrix::operator + (JMatrix& m) const
{
	JMatrix result(*this);

	int nRow = m.GetNRow();
	int nCol = m.GetNCol();

	if (result.GetNRow() != nRow || result.GetNCol() != nCol) {
		result.setZero();
		return result;
	}

	for (int i = 0; i < nRow; i++)
	{
		for (int j = 0; j < nCol; j++)
		{
			result[i][j] += m[i][j];
		}
	}

	return result;
}

JMatrix JMatrix::operator - (JMatrix& m) const
{
	JMatrix result(*this);
	int nRow = m.GetNRow();
	int nCol = m.GetNCol();

	if (result.GetNRow() != nRow || result.GetNCol() != nCol) {
		result.setZero();
		return result;
	}

	for (int i = 0; i < nRow; i++)
	{
		for (int j = 0; j < nCol; j++)
		{
			result[i][j] -= m[i][j];
		}
	}

	return result;
}

JMatrix JMatrix::operator * (JMatrix& m) const
{
	int nRow1 = m_nRow;
	int nCol1 = m_nCol;
	int nRow2 = m.GetNRow();
	int nCol2 = m.GetNCol();
	JMatrix result(nRow1, nCol2);
	result.setZero();
	result.m_cType = m.m_cType;

	if (m_nCol != m.GetNRow())
		return result;

	//int i, j, k;
	int i;
	int j;
	int k;
	for (i = 0; i < nRow1; i++)
	{
		for (j = 0; j < nCol2; j++)
		{
			for (k = 0; k < nCol1; k++)
				result[i][j] += (*this)[i][k] * m[k][j];
		}
	}
	return result;
}

JVector JMatrix::operator * (JVector& v) const
{
	int nRow1 = m_nRow;
	int nCol1 = m_nCol;
	JVector result;

	if (m_nCol != 3)
		return result;

	int i, j;
	for (i = 0; i < nRow1; i++)
	{
		result.m_p[i] = 0;
		for (j = 0; j < 3; j++)
		{
			result.m_p[i] = result.m_p[i] + this->m_p[0 * i + j] * v.m_p[j];
		}
	}

	return result;
}
JMatrix JMatrix::operator / (JMatrix& m) const
{
	int nRow1 = m_nRow;
	int nCol1 = m_nCol;
	int nRow2 = m.GetNRow();
	int nCol2 = m.GetNCol();
	JMatrix result(nRow1, nCol2);
	result.setZero();

	if (m_nCol != m.GetNRow())
		return result;

	if (m.GetNCol() != m.GetNRow())
		return result;

	JMatrix m2 = m.GetInverse();

	int i, j, k;
	for (i = 0; i < nRow1; i++)
	{
		for (j = 0; j < nCol2; j++)
		{
			for (k = 0; k < nCol1; k++)
				result[i][j] += (*this)[i][k] * m2[k][j];
		}
	}

	return result;
}

JMatrix JMatrix::operator + (const double& n) const
{
	JMatrix result(*this);

	for (int i = 0; i < m_nRow; i++)
	{
		for (int j = 0; j < m_nCol; j++)
		{
			result[i][j] += n;
		}
	}

	return result;
}

JMatrix JMatrix::operator - (const double& n) const
{
	JMatrix result(*this);

	for (int i = 0; i < m_nRow; i++)
	{
		for (int j = 0; j < m_nCol; j++)
		{
			result[i][j] -= n;
		}
	}

	return result;
}

JMatrix JMatrix::operator * (const double& n) const
{
	JMatrix result(*this);

	for (int i = 0; i < m_nRow; i++)
	{
		for (int j = 0; j < m_nCol; j++)
		{
			result[i][j] *= n;
		}
	}

	return result;
}

JMatrix JMatrix::operator / (const double& n) const
{
	JMatrix result(*this);

	for (int i = 0; i < m_nRow; i++)
	{
		for (int j = 0; j < m_nCol; j++)
		{
			result[i][j] /= n;
		}
	}

	return result;
}

JMatrix operator + (double& n, const JMatrix& m)
{
	return m + n;
}

JMatrix operator - (double& n, const JMatrix& m)
{
	return m - n;
}

JMatrix operator * (double& n, const JMatrix& m)
{
	return m * n;
}

JMatrix operator / (double& n, const JMatrix& m)
{
	return m / n;
}

JMatrix& JMatrix::operator += (JMatrix& m)
{
	if (m_nCol != m.GetNCol() || m_nRow != m.GetNRow())
		return *this;

	int i, j;
	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < m_nCol; j++)
		{
			(*this)[i][j] += m[i][j];
		}
	}

	return (*this);
}

JMatrix& JMatrix::operator -= (JMatrix& m)
{
	if (m_nCol != m.GetNCol() || m_nRow != m.GetNRow())
		return *this;

	int i, j;
	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < m_nCol; j++)
		{
			(*this)[i][j] -= m[i][j];
		}
	}

	return (*this);
}

JMatrix& JMatrix::operator *= (JMatrix& m)
{
	int nRow1 = m_nRow;
	int nCol1 = m_nCol;
	int nRow2 = m.GetNRow();
	int nCol2 = m.GetNCol();
	JMatrix result(nRow1, nCol2);
	result.setZero();

	if (m_nCol != m.GetNRow())
		return (*this);

	if (m.GetNCol() != m.GetNRow())
		return (*this);

	int i, j, k;
	for (i = 0; i < nRow1; i++)
	{
		for (j = 0; j < nCol2; j++)
		{
			for (k = 0; k < nCol1; k++)
				result[i][j] += (*this)[i][k] * m[k][j];
		}
	}

	for (i = 0; i < m_nRow; i++) {
		for (j = 0; j < m_nCol; j++) {
			(*this)[i][j] = result[i][j];
		}
	}

	return (*this);
}

JMatrix& JMatrix::operator /= (JMatrix& m)
{
	int nRow1 = m_nRow;
	int nCol1 = m_nCol;
	int nRow2 = m.GetNRow();
	int nCol2 = m.GetNCol();
	JMatrix result(nRow1, nCol2);
	result.setZero();

	if (m_nCol != m.GetNRow())
		return (*this);

	if (m.GetNCol() != m.GetNRow())
		return (*this);

	JMatrix m2 = m.GetInverse();

	int i, j, k;
	for (i = 0; i < nRow1; i++)
	{
		for (j = 0; j < nCol2; j++)
		{
			for (k = 0; k < nCol1; k++)
				result[i][j] += (*this)[i][k] * m2[k][j];
		}
	}

	for (i = 0; i < m_nRow; i++) {
		for (j = 0; j < m_nCol; j++) {
			(*this)[i][j] = result[i][j];
		}
	}

	return (*this);
}

JMatrix& JMatrix::operator += (const double& n)
{
	int i;
	for (i = 0; i < m_nCol * m_nRow; i++)
		m_p[i] += n;

	return (*this);
}

JMatrix& JMatrix::operator -= (const double& n)
{
	int i;
	for (i = 0; i < m_nCol * m_nRow; i++)
		m_p[i] -= n;

	return (*this);
}

JMatrix& JMatrix::operator *= (const double& n)
{
	int i;
	for (i = 0; i < m_nCol * m_nRow; i++)
		m_p[i] *= n;

	return (*this);
}

JMatrix& JMatrix::operator /= (const double& n)
{
	int i;
	for (i = 0; i < m_nCol * m_nRow; i++)
		m_p[i] /= n;

	return (*this);
}


//////////////////////////////////////////////////////////////////////////
// R Matrix or H Matrix Mode Function Set
JMatrix::JMatrix(char nType)
	: m_p(NULL)
	, m_nCol(0)
	, m_nRow(0)
{
	JMatrix();
	if (nType == 'R') {
		InitializeRMatrix();
	}
	else if (nType == 'H') {
		InitializeHMatrix();
	}
	else
		JMatrix();
}

bool JMatrix::SetDegree(double Rx, double Ry, double Rz)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	double A = Rz * _PI_ / 180.0;
	double B = Ry * _PI_ / 180.0;
	double G = Rx * _PI_ / 180.0;
	SetRadian(G, B, A);

	return true;
}

bool JMatrix::SetRadian(double Rx, double Ry, double Rz)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	(*this)[0][0] = COS(Rz) * COS(Ry);
	(*this)[0][1] = COS(Rz) * SIN(Ry) * SIN(Rx) - SIN(Rz) * COS(Rx);
	(*this)[0][2] = COS(Rz) * SIN(Ry) * COS(Rx) + SIN(Rz) * SIN(Rx);

	(*this)[1][0] = SIN(Rz) * COS(Ry);
	(*this)[1][1] = SIN(Rz) * SIN(Ry) * SIN(Rx) + COS(Rz) * COS(Rx);
	(*this)[1][2] = SIN(Rz) * SIN(Ry) * COS(Rx) - COS(Rz) * SIN(Rx);

	(*this)[2][0] = -SIN(Ry);
	(*this)[2][1] = COS(Ry) * SIN(Rx);
	(*this)[2][2] = COS(Ry) * COS(Rx);
	return true;
}

bool JMatrix::GetDegree(double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	GetRadian(Rx, Ry, Rz);
	Rx *= 180.0 / _PI_;
	Ry *= 180.0 / _PI_;
	Rz *= 180.0 / _PI_;
	return true;
}

bool JMatrix::GetRadian(double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	int i, j;
	for (i = 0; i < m_nRow; i++) {
		for (j = 0; j < m_nCol; j++) {
			if (fabs_((*this)[i][j]) < _EPSILON_) {
				(*this)[i][j] = 0.0;
			}
		}
	}

	Rz = ATAN2((*this)[1][0], (*this)[0][0]);
	Ry = ATAN2(-(*this)[2][0], SQRT(POW((*this)[2][1], 2) + POW((*this)[2][2], 2)));
	Rx = ATAN2((*this)[2][1], (*this)[2][2]);
	if (fabs_(Rz) < _EPSILON_)
		Rz = 0.0;
	if (fabs_(Ry) < _EPSILON_)
		Ry = 0.0;
	if (fabs_(Rx) < _EPSILON_)
		Rx = 0.0;
	return true;
}

bool JMatrix::Set_nsa(JVector& n, JVector& s, JVector& a)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	(*this)[0][0] = n[0];
	(*this)[1][0] = n[1];
	(*this)[2][0] = n[2];

	(*this)[0][1] = s[0];
	(*this)[1][1] = s[1];
	(*this)[2][1] = s[2];

	(*this)[0][2] = a[0];
	(*this)[1][2] = a[1];
	(*this)[2][2] = a[2];
	return true;
}

bool JMatrix::Set_nsa(double* n, double* s, double* a)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	(*this)[0][0] = n[0];
	(*this)[1][0] = n[1];
	(*this)[2][0] = n[2];

	(*this)[0][1] = s[0];
	(*this)[1][1] = s[1];
	(*this)[2][1] = s[2];

	(*this)[0][2] = a[0];
	(*this)[1][2] = a[1];
	(*this)[2][2] = a[2];
	return true;
}

bool JMatrix::Get_nsa(JVector& n, JVector& s, JVector& a)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	n[0] = (*this)[0][0];
	n[1] = (*this)[1][0];
	n[2] = (*this)[2][0];

	s[0] = (*this)[0][1];
	s[1] = (*this)[1][1];
	s[2] = (*this)[2][1];

	a[0] = (*this)[0][2];
	a[1] = (*this)[1][2];
	a[2] = (*this)[2][2];

	return true;
}

bool JMatrix::Get_nsa(double* n, double* s, double* a)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	n[0] = (*this)[0][0];
	n[1] = (*this)[1][0];
	n[2] = (*this)[2][0];

	s[0] = (*this)[0][1];
	s[1] = (*this)[1][1];
	s[2] = (*this)[2][1];

	a[0] = (*this)[0][2];
	a[1] = (*this)[1][2];
	a[2] = (*this)[2][2];

	return true;
}

bool JMatrix::SkewSymetric(JVector& v)
{
	if (m_cType != 'R')
		return false;

	(*this)[0][0] = 0.0;
	(*this)[0][1] = -v[2];
	(*this)[0][2] = v[1];

	(*this)[1][0] = v[2];
	(*this)[1][1] = 0.0;
	(*this)[1][2] = -v[0];

	(*this)[2][0] = -v[1];
	(*this)[2][1] = v[0];
	(*this)[2][2] = 0.0;

	return true;
}

bool JMatrix::SkewSymetric(double v[3])
{
	JVector v2(v[0], v[1], v[2]);
	return SkewSymetric(v2);
}

void JMatrix::InitializeRMatrix(void)
{
	if (m_cType != 'R') {
		ReleaseMatrix();
		m_nCol = 3;
		m_nRow = 3;
		m_cType = 'R';
		MakeMatrix();
	}
	m_nCol = 3;
	m_nRow = 3;
	m_cType = 'R';
	setIdentity();
}

JVector JMatrix::operator * (const JVector& v)
{
	JVector result;

	if (m_cType != 'R')
		return result;

	result[0] = ((*this)[0][0] * v[0]) + ((*this)[0][1] * v[1]) + ((*this)[0][2] * v[2]);
	result[1] = ((*this)[1][0] * v[0]) + ((*this)[1][1] * v[1]) + ((*this)[1][2] * v[2]);
	result[2] = ((*this)[2][0] * v[0]) + ((*this)[2][1] * v[1]) + ((*this)[2][2] * v[2]);

	return result;
}

void JMatrix::InitializeHMatrix(void)
{
	if (m_cType != 'H') {
		ReleaseMatrix();
		m_nCol = 4;
		m_nRow = 4;
		m_cType = 'H';
		MakeMatrix();
	}
	m_nCol = 4;
	m_nRow = 4;
	m_cType = 'H';
	setIdentity();
}

bool JMatrix::SetTranslate(double x, double y, double z)
{
	if (m_cType != 'H')
		return false;

	(*this)[0][3] = x;
	(*this)[1][3] = y;
	(*this)[2][3] = z;
	return true;
}

bool JMatrix::SetTranslate(JVector& v)
{
	return SetTranslate(v[0], v[1], v[2]);
}

bool JMatrix::GetTranslate(double& x, double& y, double& z)
{
	if (m_cType != 'H')
		return false;

	x = (*this)[0][3];
	y = (*this)[1][3];
	z = (*this)[2][3];
	return true;
}

bool JMatrix::GetTranslate(JVector& v)
{
	if (m_cType != 'H')
		return false;

	v[0] = (*this)[0][3];
	v[1] = (*this)[1][3];
	v[2] = (*this)[2][3];
	return true;
}

bool JMatrix::SetRMatrix(JMatrix& R)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			(*this)[i][j] = R[i][j];
		}
	}

	return true;
}

bool JMatrix::GetRMatrix(JMatrix& R)
{
	if (m_cType != 'R' && m_cType != 'H')
		return false;

	int i, j;
	for (i = 0; i < 3; i++) {
		for (j = 0; j < 3; j++) {
			R[i][j] = (*this)[i][j];
		}
	}

	return true;
}

bool JMatrix::SetMatrix(int nRow, int nCol)
{
	if (nRow < 1 && nCol)
		return false;

	m_nRow = nRow;
	m_nCol = nCol;
	MakeMatrix();
	return true;
}

bool JMatrix::SetHomer(JVector& v, JMatrix& R)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(v);
	SetRMatrix(R);
	return true;
}

bool JMatrix::SetHomerDegree(JVector& v, double Rx, double Ry, double Rz)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(v);
	SetDegree(Rx, Ry, Rz);
	return true;
}

bool JMatrix::SetHomerRadian(JVector& v, double Rx, double Ry, double Rz)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(v);
	SetRadian(Rx, Ry, Rz);
	return true;
}

bool JMatrix::SetHomer(double x, double y, double z, JMatrix& R)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(x, y, z);
	SetRMatrix(R);
	return true;
}

bool JMatrix::SetHomerDegree(double x, double y, double z, double Rx, double Ry, double Rz)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(x, y, z);
	SetDegree(Rx, Ry, Rz);
	return true;
}

bool JMatrix::SetHomerRadian(double x, double y, double z, double Rx, double Ry, double Rz)
{
	if (m_cType != 'H')
		return false;

	SetTranslate(x, y, z);
	SetRadian(Rx, Ry, Rz);
	return true;
}

bool JMatrix::GetHomer(JVector& v, JMatrix& R)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(v);
	GetRMatrix(R);
	return true;
}

bool JMatrix::GetHomer(double& x, double& y, double& z, JMatrix& R)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(x, y, z);
	GetRMatrix(R);
	return true;
}

bool JMatrix::GetHomerDegree(JVector& v, double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(v);
	GetDegree(Rx, Ry, Rz);
	return true;
}

bool JMatrix::GetHomerRadian(JVector& v, double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(v);
	GetRadian(Rx, Ry, Rz);
	return true;
}

bool JMatrix::GetHomerDegree(double& x, double& y, double& z, double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(x, y, z);
	GetDegree(Rx, Ry, Rz);
	return true;
}

bool JMatrix::SetHomerRadian(double& x, double& y, double& z, double& Rx, double& Ry, double& Rz)
{
	if (m_cType != 'H')
		return false;

	GetTranslate(x, y, z);
	GetRadian(Rx, Ry, Rz);
	return true;
}
//////////////////////////////////////////////////////////////////////////

JMatrix	JMatrix::GetTopLeftCorner(int row, int col)
{
	JMatrix	RESULT(row, col);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[i][j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetTopRightCorner(int row, int col)
{
	JMatrix	RESULT(row, col);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[i][m_nCol - col + j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetBottomLeftCorner(int row, int col)
{
	JMatrix	RESULT(row, col);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[m_nRow - row + i][j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetBottomRightCorner(int row, int col)
{
	JMatrix	RESULT(row, col);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[m_nRow - row + i][m_nCol - col + j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetLeftCols(int col)
{
	JMatrix	RESULT(m_nRow, col);
	int i, j;
	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[i][j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetRightCols(int col)
{
	JMatrix	RESULT(m_nRow, col);
	int i, j;
	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < col; j++)
			RESULT(i, j) = (*this)[i][m_nCol - col + j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetTopRows(int row)
{
	JMatrix	RESULT(row, m_nCol);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < m_nCol; j++)
			RESULT(i, j) = (*this)[i][j];
	}
	return RESULT;
}

JMatrix	JMatrix::GetBottomRows(int row)
{
	JMatrix	RESULT(row, m_nCol);
	int i, j;
	for (i = 0; i < row; i++)
	{
		for (j = 0; j < m_nCol; j++)
			RESULT(i, j) = (*this)[m_nRow - row + i][j];
	}
	return RESULT;
}

JMatrix JMatrix::GetCol(int ncol)
{
	int i;
	JMatrix		RESULT(m_nRow, 1);
	for (i = 0; i < m_nRow; i++)
		RESULT(i, 0) = (*this)(i, ncol);
	return RESULT;
}

JMatrix JMatrix::GetRow(int nrow)
{
	int i;
	JMatrix		RESULT(1, m_nCol);
	for (i = 0; i < m_nCol; i++)
		RESULT(0, i) = (*this)(nrow, i);
	return RESULT;
}

bool	JMatrix::SetTopLeftCorner(JMatrix & A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[i][j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetTopRightCorner(JMatrix & A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[i][m_nCol - A.GetNCol() + j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetBottomLeftCorner(JMatrix & A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[m_nRow - A.GetNRow() + i][j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetBottomRightCorner(JMatrix & A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[m_nRow - A.GetNRow() + i][m_nCol - A.GetNCol() + j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetLeftCols(JMatrix & A)
{
	if (A.GetNRow() != m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;

	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[i][j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetRightCols(JMatrix & A)
{
	if (A.GetNRow() != m_nRow || A.GetNCol() > m_nCol)
		return false;

	int i, j;
	for (i = 0; i < m_nRow; i++)
	{
		for (j = 0; j < A.GetNCol(); j++)
			(*this)[i][m_nRow - A.GetNCol() + j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetTopRows(JMatrix& A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() != m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < m_nCol; j++)
			(*this)[i][j] = A(i, j);
	}
	return true;
}

bool	JMatrix::SetBottomRows(JMatrix& A)
{
	if (A.GetNRow() > m_nRow || A.GetNCol() != m_nCol)
		return false;

	int i, j;
	for (i = 0; i < A.GetNRow(); i++)
	{
		for (j = 0; j < m_nCol; j++)
			(*this)[m_nRow - A.GetNRow() + i][j] = A(i, j);
	}
	return true;
}

bool JMatrix::SetCol(int ncol, JMatrix& A)
{
	if (A.GetNRow() != m_nRow || A.GetNCol() != 1)
		return false;
	int i;
	for (i = 0; i < m_nRow; i++)
		(*this)[i][ncol] = A(i, 0);

	return true;
}

bool JMatrix::SetRow(int nrow, JMatrix& A)
{
	if (A.GetNRow() != 1 || A.GetNCol() != m_nCol)
		return false;

	int i;
	for (i = 0; i < m_nCol; i++)
		(*this)[nrow][i] = A(0, i);

	return true;
}

double JMatrix::norm(void)
{
	double sol = 0.0;
	for (int i = 0; i < GetNRow(); i++)
		sol = sol + (*this)[i][0] * (*this)[i][0];
	return SQRT(sol);
}


//////////////////////////////////////////////////////////////////////////
bool gauss_inverse(int n, double a[], double x[])
{
	//double g[N][N+N];
	int mi;
	double** g;
	g = new double* [n];
	for (mi = 0; mi < n; mi++)
		g[mi] = new double[n + n];


	int i, j, k;
	double temp, gii, gik, gki;

	//   if (n > N) {
	//     fprintf(stderr,"gauss_inverse: dimension can't over %d\n",N);
	//     return(2);
	//   }

	/*[g]=[a I] */
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) {
			g[i][j] = a[i * n + j];
			g[i][n + j] = 0;
		}
		g[i][n + i] = 1;
	}

	/*Form upper triangle matrix */
	for (i = 0; i < n; i++) {

		/*  Find the row whose i-th element is not zero */
		for (k = i; k < n; k++) if (g[k][i] != 0) break;

		/*  If i-th elements is all zero, it is singular maxtrix! */
		if (k == n) {
			//fprintf(stderr,"gauss_inverse: can't solve singular singular!\n");
			for (mi = 0; mi < n; mi++)
				delete[] g[mi];
			delete[] g;

			return false;
		}

		/*  If the row is not the i-th, exchange the two rows */
		if (k != i) {
			for (j = i; j < n + n; j++) {
				temp = g[i][j]; g[i][j] = g[k][j]; g[k][j] = temp;
			}
		}

		/*  Normalize i-th row to Gii=1 */
		for (gii = g[i][i], j = i; j < n + n; j++) g[i][j] /= gii;

		/*  Making the other i-th elements to zero */
		for (k = i + 1; k < n; k++) {
			for (gki = g[k][i], j = i; j < n + n; j++) {
				g[k][j] -= g[i][j] * gki;
			}
		}
	}

	/*Form diagonal matrix */
	for (i = n - 1; i >= 0; i--) {

		/*  Making the other elements except the diagonal element to zero */
		for (k = i + 1; k < n; k++) {
			gik = g[i][k];
			g[i][k] = 0;
			for (j = n; j < n + n; j++) g[i][j] -= g[k][j] * gik;
		}
	}

	/*Inverse Matrix is obtained from the right half matrix */
	for (i = 0; i < n; i++) {
		for (j = 0; j < n; j++) x[i * n + j] = g[i][n + j];
	}

	for (mi = 0; mi < n; mi++)
		delete[] g[mi];
	delete[] g;

	return true;;
}
