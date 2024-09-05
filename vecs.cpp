// dl_vec.cpp
// 2d and 3d vector stuff

#include "vecs.h"

//void dl3v<float>::rotateByQuaternion(const Quaternion & Q)
//{
//	const float qx = Q.x, qy = Q.y, qz = Q.z, qw = Q.w;
//
//	const float ix = qw * x + qy * z - qz * y;
//	const float iy = qw * y + qz * x - qx * z;
//	const float iz = qw * z + qx * y - qy * x;
//	const float iw = - qx * x - qy * y - qz * z;
//
//	// calculate result * inverse quat
//	x = ix * qw + iw * - qx + iy * - qz - iz * - qy;
//	y = iy * qw + iw * - qy + iz * - qx - ix * - qz;
//	z = iz * qw + iw * - qz + ix * - qy - iy * - qx;
//}


//////////////////////////////////////////////////////////////////////////////////
// Point Line Helpers
//////////////////////////////////////////////////////////////////////////////////

#if 0
/* returns point on line defined by P1 and P2 nearest point Q */
Pm2V NearestPointOnLine(
	const Pm2V& P1,
	const Pm2V& P2,
	const Pm2V& Q)
{
	Pm2V line_vec = P2 - P1;

	double t = Dot(line_vec, Q - P1);

	/* geometrically, t is now the signed distance along the line P1->P2
		times the length of the line from P1->P2.
		So, t divided by the length of P1-P2 squared is the proportion
		of the distance from P1 to P2 at which the nearest point is located */
	t /= line_vec.LengthSq();

	return P1 + t * line_vec;
}

/* returns false if lines are parallel */
bool LineIntersect(
	const Pm2V P0,		// endpoints
	const Pm2V P1,
	const Pm2V Q0,
	const Pm2V Q1,
	Pm2V& intersection_point)
{
	return PmLine2d(P0, P1).Intersection(PmLine2d(Q0, Q1), &intersection_point);
}

bool isPointCloseToLineSq(
	const Pm2V& Q,
	const Pm2V& P1,
	const Pm2V& P2,
	double dist_sq)
{
	Pm2V line_vec = P2 - P1;
	double t = Dot(line_vec, Q - P1);
	t /= line_vec.LengthSq();
	Pm2V nearest_pt_on_line = P1 + t * line_vec;
	return isClosePointSq(Q, nearest_pt_on_line, dist_sq);
}

/* determine if points a and b are within sqrt(dist_sq) of each other */
bool isClosePointSq(Pm2V* a, Pm2V* b, double dist_sq)
{
	double dx = a->x - b->x;
	double dy = a->y - b->y;
	return (dx * dx + dy * dy <= dist_sq);
}

bool isClosePointSq(const Pm2V& a, const Pm2V& b, double dist_sq)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	return (dx * dx + dy * dy <= dist_sq);
}

bool isClosePointSq(const Pm2Vf& a, const Pm2Vf& b, float dist_sq)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	return (dx * dx + dy * dy <= dist_sq);
}


/* faster than GetAngle */
bool GetSinCos(
	Pm2V start,
	Pm2V end,
	double& sine,
	double& cosine)
{
	if (!start.normalize() || !end.normalize())
		return false;

	Pm3V sine_vec(Pm3V(start) * Pm3V(end));
	sine = sine_vec.Dot(Pm3V(0, 0, 1));
	cosine = start.Dot(end);

	return true;
}

/* result is between -Pi to +Pi.
	In a right-hand coordinate system,
	when end_vec is counterclockwise from start_vec, the angle is positive */
double GetAngle(
	const Pm2V& start_vec,
	const Pm2V& end_vec)
{
	// CCWAngleBetween suffers from over agressive snap-to-zero. don't use it
	double sine, cosine;
	if (!GetSinCos(start_vec, end_vec, sine, cosine))
	{
		assert(0);
		return 0.0;
	}
	return atan2(sine, cosine);
}

#endif


// this stuff is very old and not currently used
// but let's keep it around in the code base for a while
// because it may be useful
// there are some very fast methods here for the special
// case of 2d vectors

#if 0
#include "Pm2V.h"
static const double coincidist = 1e-5;

C2dVect::C2dVect(const Pm3V& v3d)
{
	x = v3d.x;
	y = v3d.y;
}

C2dVect::C2dVect(const IwVector3d& v3d)
{
	x = v3d.x;
	y = v3d.y;
}

bool isClosePoint(double x1, double x2)
{
	return abs(x1 - x2) <= coincidist;
}

bool isClosePoint(const C2dVect& v1, const C2dVect& v2)
{
	return(isClosePoint(v1.x, v2.x) && isClosePoint(v1.y, v2.y));
}

C2dVect NearestPointToLine(const C2dVect& P, const C2dVect& v1, const C2dVect& v2)
{
	double dx, dy, t, x1, y1, x2, y2;
	x1 = v1.x;
	x2 = v2.x;
	y1 = v1.y;
	y2 = v2.y;
	dy = y2 - y1;
	dx = x2 - x1;
	t = (P.x * dx + P.y * dy - x1 * dx - y1 * dy) / (dx * dx + dy * dy);
	return C2dVect(x1 + dx * t, y1 + dy * t);
}
{ return a.x * b.x + a.y * b.y; }

double DistanceOfPointToLine(const C2dVect& P, const C2dVect& v1, const C2dVect& v2)
{
	C2dVect q;
	q = NearestPointToLine(P, v1, v2);
	q -= P;
	return q.Length();
}

int PointLeftOrRightOfLine(C2dVect& P, C2dVect& v1, C2dVect& v2)
{
	C2dVect Q = NearestPointToLine(P, v1, v2);
	double cm = CrossMag(P - Q, v2 - v1);
	if (cm > 0.0f)
		return 1;         // P is to the Right of the line
	else if (cm < 0.0f)
		return -1;        // Left
	else
		return 0;         // P is on the line
}

C2dVectArray::C2dVectArray()
{
}

C2dVectArray::~C2dVectArray()
{
}

C2dVectList::C2dVectList()
{
}

C2dVectList::~C2dVectList()
{
}

void C2dVectList::AsciiDump(bool append, CString fn) const
{
	ofstream f;

	if (append)
		f.open(fn, ios::ate);
	else
		f.open(fn);

	for (POSITION pos = GetHeadPosition(); pos; GetNext(pos))
	{
		C2dVect v = GetAt(pos);
		f << v.x << " " << v.y << "\n";
	}

	f << "\n";

	f.close();
}

void AFXAPI ConstructElements(C2dVect* p, int n)
{
	for (int i = 0; i < n; i++)
		p[i].C2dVect::C2dVect();
}

void AFXAPI DestructElements(C2dVect* p, int n)
{
	for (int i = 0; i < n; i++)
		p[i].C2dVect::~C2dVect();
}

C2dLine::C2dLine()
{
}

C2dLine::C2dLine(const C2dVect& v0, const C2dVect& v1)
{
	v[0] = v0;
	v[1] = v1;
}

C2dLine::~C2dLine()
{
}

C2dVect NearestPointToLine(const C2dVect& P, const C2dLine& L)
{
	return NearestPointToLine(P, L.v[0], L.v[1]);
}

double DistanceOfPointToLine(const C2dVect& P, const C2dLine& L)
{
	return DistanceOfPointToLine(P, L.v[0], L.v[1]);
}

bool Line2dIntersection(const C2dLine& L, C2dLine M, C2dVect& intersection_pt)
{
	// surely way more efficient algorithms in Graphic Gems
	double x;
	C2dTMatrix t;
	C2dVect v = Unitize(L.v[1] - L.v[0]);
	double sine = v.y;
	double cosine = v.x;

	/* what is the matrix that would transform L to start at
		the origin and align with the positive x axis */
	t.Translate(-L.v[0].x, -L.v[0].y, TRUE);
	t.Rotate(-sine, cosine);
	/* get rid of this later */
	{
		C2dVect test = t * L.v[0];
		test = t * L.v[1];
		assert(abs(test.y) < 0.00001);
	}

	/* now transform line M with this transform */
	t.TransformPoints(M.v, 2);

	/* where does line temp cross the x axis? this is the intersection point
		in the transformed space */
	{
		double x1 = M.v[0].x;
		double y1 = M.v[0].y;
		double x2 = M.v[1].x;
		double y2 = M.v[1].y;
		double dy = y2 - y1;
		double dx = x2 - x1;
		if (abs(dy) < 0.00001)
		{
			/* lines are parallel, no intersection, return false */
			return 0;
		}
		x = x1 - dx / dy * y1;
	}

	/* now transform the point (x, 0) back to the original space.
		the result is the point of intersection */
	intersection_pt.x = x;
	intersection_pt.y = 0;
	t.Invert();

	t.TransformPoints(&intersection_pt);
	return 1;
}

bool LineSegment2dIntersection(const C2dLine& L, const C2dLine& M, C2dVect& intersection_point)
{
	// there is a way more efficient implementation in Graphic Gems 2, p.7
	// (especially for the case where the lines usually don't intersect
	double dotL, dotM;
	if (Line2dIntersection(L, M, intersection_point))
	{
		dotL = Dot(intersection_point - L.v[0], intersection_point - L.v[1]);
		dotM = Dot(intersection_point - M.v[0], intersection_point - M.v[1]);
		return dotL <= 0.0 && dotM <= 0.0;
	}
	else
	{
		return 0;
	}
}

C2dTriangle::C2dTriangle()
{
}

C2dTriangle::C2dTriangle(C2dVect& v1, C2dVect& v2, C2dVect& v3)
{
	v[0] = v1;
	v[1] = v2;
	v[2] = v3;
}

C2dTriangle::~C2dTriangle()
{
}



bool C2dTriangle::Inside(C2dVect& P)
{
	int a = PointLeftOrRightOfLine(P, v[0], v[1]);
	int b = PointLeftOrRightOfLine(P, v[1], v[2]);
	int c = PointLeftOrRightOfLine(P, v[2], v[0]);
	/* for point to be inside, it must be to the same side of each of
		the edges of the triangle. on a line counts for inside */
	return !(a * b < 0 || a * c < 0);
}

C2dTriangleArray::C2dTriangleArray()
{
}

C2dTriangleArray::~C2dTriangleArray()
{
}

C2dTriangleList::C2dTriangleList()
{
}

C2dTriangleList::~C2dTriangleList()
{
}

void _stdcall ConstructElements(C2dTriangle* p, int n)
{
	for (int i = 0; i < n; i++)
		p[i].C2dTriangle::C2dTriangle();
}

void _stdcall DestructElements(C2dTriangle* p, int n)
{
	for (int i = 0; i < n; i++)
		p[i].C2dTriangle::~C2dTriangle();
}

static bool Convex(C2dVect& v1, C2dVect& v2, C2dVect& v3)
{
	/* turing left turn is concave */
	C2dVect q, r;
	double s;

	q = v2 - v1;
	r = v3 - v2;

	s = CrossMag(q, r);

	return s < 0.0f;
}

C2dTriangleArray* Facetize2DPolygon(C2dVectList& InputVertList, bool invert)
{
	POSITION pos, tpos, prev_pos, curr_pos, next_pos;
	C2dVect prev, curr, next;
	C2dTriangleArray* pTriArray = new C2dTriangleArray;
	C2dTriangle T;
	bool found_tri;
	C2dVectList VertList;

	/* create local vert list in the order which defines the inside to
		be on the right */
	for (pos = InputVertList.GetHeadPosition(); pos != NULL; )
	{
		next = InputVertList.GetNext(pos);
		if (invert)
			VertList.AddHead(next);
		else
			VertList.AddTail(next);
	}

	do
	{
		/* start out with prev and current set to last 2 verts in the list
			and next set to the first vert in the list */
		curr_pos = VertList.GetTailPosition();
		prev_pos = curr_pos;
		VertList.GetPrev(prev_pos);
		next_pos = VertList.GetHeadPosition();
		found_tri = 0;

		while (next_pos != NULL)
		{
			prev = VertList.GetAt(prev_pos);
			curr = VertList.GetAt(curr_pos);
			next = VertList.GetAt(next_pos);

			if (Convex(prev, curr, next))
			{
				int vert_inside;
				/* we have a convex vertex of the polygon.
					we'd like to put a line across and call it a triangle
					and remove the convex vertex from the polygon.
					But we must check to see that no verts of the polygon
					would fall inside this triangle. If they do, then
					we can't take hack off this convex vert at this time. */
				T.v[0] = prev;
				T.v[1] = curr;
				T.v[2] = next; // order so that right hand rule points down (-z)

				for (tpos = VertList.GetHeadPosition();
					tpos != NULL; )

				{
					C2dVect P = VertList.GetNext(tpos);
					vert_inside = P != T.v[0]
						&& P != T.v[1]
						&& P != T.v[2]
						&& T.Inside(P);
					if (vert_inside)
						break;
				}

				if (!vert_inside)
				{
					pTriArray->Add(T);
					found_tri = 1;

					/* remove the curr vertex from the list */
					VertList.RemoveAt(curr_pos);

					break;
				}
			}

			/* update positions */
			prev_pos = curr_pos;
			curr_pos = next_pos;
			VertList.GetNext(next_pos);

		} // while(pos != NULL)
		assert(found_tri);

	} while (VertList.GetCount() > 2);

	return pTriArray;
}

/* global constant 2d identity matrix */
const C2dTMatrix IdentityMat2d(1);

C2dTMatrix::C2dTMatrix(bool set_identity)
{
	if (set_identity)
	{
		a = 1;
		b = 0;
		tx = 0;
		c = 0;
		d = 1;
		ty = 0;
	}
}

void C2dTMatrix::SetIdentity()
{
	a = 1;
	b = 0;
	tx = 0;
	c = 0;
	d = 1;
	ty = 0;
}

C2dTMatrix C2dTMatrix::operator*(const C2dTMatrix& right) const
{
	C2dTMatrix result;
	/*
		int i,j;
		for (col = 0; col < 3; col++)
		for (row = 0; row < 3; row++)
		{
			result.mat[col][row] = (*this).mat[col][0]*right.mat[0][row] + (*this).mat[col][1]*right.mat[1][row] + (*this).mat[col][2]*right.mat[2][row];
		}
	*/
	result.a = this->a * right.a + this->b * right.c;
	result.b = this->a * right.b + this->b * right.d;
	result.c = this->c * right.a + this->d * right.c;
	result.d = this->c * right.b + this->d * right.d;
	result.tx = this->tx * right.a + this->ty * right.c + 1.0 * right.tx;
	result.ty = this->tx * right.b + this->ty * right.d + 1.0 * right.ty;
	return result;
}

void C2dTMatrix::PreMult(const C2dTMatrix& left)
{
	C2dTMatrix right = *this;
	a = left.a * right.a + left.b * right.c;
	b = left.a * right.b + left.b * right.d;
	tx = left.a * right.tx + left.b * right.ty + left.tx * 1.0;
	c = left.c * right.a + left.d * right.c;
	d = left.c * right.b + left.d * right.d;
	ty = left.c * right.tx + left.d * right.ty + left.ty * 1.0;
}

void C2dTMatrix::PostMult(const C2dTMatrix& right)
{
	C2dTMatrix left = *this;
	a = left.a * right.a + left.b * right.c;
	b = left.a * right.b + left.b * right.d;
	tx = left.a * right.tx + left.b * right.ty + left.tx * 1.0;
	c = left.c * right.a + left.d * right.c;
	d = left.c * right.b + left.d * right.d;
	ty = left.c * right.tx + left.d * right.ty + left.ty * 1.0;
}

void C2dTMatrix::Invert()
{
	double aa, bb, cc, dd, ttx, tty;
	double determinant = a * d - b * c;
	C2dTMatrix test = *this;           // debug
	assert(abs(determinant) > 1e-8);
	aa = d / determinant;
	bb = -c / determinant;
	cc = -b / determinant;
	dd = a / determinant;
	ttx = (b * ty - d * tx) / determinant;
	tty = (-a * ty + c * tx) / determinant;
	a = aa;
	b = cc;
	c = bb;
	d = dd;
	tx = ttx;
	ty = tty;
	test.PostMult(*this);              // debug
	assert(abs(test.a - 1) < 1e-8);
	assert(abs(test.d - 1) < 1e-8);
	assert(abs(test.b) < 1e-8);
	assert(abs(test.c) < 1e-8);
	assert(abs(test.tx) < 1e-8);
	assert(abs(test.ty) < 1e-8);
}

void C2dTMatrix::Scale(double mx, double my, Bool init)
{
	if (init)
	{
		// set this->mat to the scale matrix
		SetIdentity();
		a = mx;
		c = my;
	}
	else
	{
		C2dTMatrix scalemat(1);
		scalemat.a = mx;
		scalemat.d = my;
		PreMult(scalemat);
	}
}

void C2dTMatrix::Translate(double dx, double dy, Bool init)
{
	if (init)
	{
		// set this->mat to the scale matrix
		SetIdentity();
		tx = dx;
		ty = dy;
	}
	else
	{
		C2dTMatrix transmat(1);
		tx = dx;
		ty = dy;
		PreMult(transmat);
	}
}

void C2dTMatrix::Translate(const C2dVect& v, Bool init)
{
	if (init)
	{
		// set this->mat to the scale matrix
		SetIdentity();
		tx = v.x;
		ty = v.y;
	}
	else
	{
		C2dTMatrix transmat(1);
		transmat.tx = v.x;
		transmat.ty = v.y;
		PreMult(transmat);
	}
}

void C2dTMatrix::Rotate(double sin, double cos, Bool init)
{
	if (init)
	{
		// set this->mat to the scale matrix
		SetIdentity();
		a = cos;
		b = -sin;
		c = sin;
		d = cos;
	}
	else
	{
		C2dTMatrix rotmat(1);
		rotmat.a = cos;
		rotmat.b = -sin;
		rotmat.c = sin;
		rotmat.d = cos;
		PreMult(rotmat);
	}
}

void C2dTMatrix::Rotate(double counterclockwise_radians, Bool init)
{
	Rotate(sin(counterclockwise_radians), cos(counterclockwise_radians), init);
}

C2dVect C2dTMatrix::operator*(const C2dVect& v) const
{
	C2dVect v1;
	v1.x = a * v.x + b * v.y + tx;
	v1.y = c * v.x + d * v.y + ty;
	return v1;
}

void C2dTMatrix::TransformPoints(C2dVect p[], int num_points) const
{
	for (int i = 0; i < num_points; i++)
	{
		double const xnew = a * p[i].x + b * p[i].y + tx;
		p[i].y = c * p[i].x + d * p[i].y + ty;
		p[i].x = xnew;
	}
}



C2dArc::C2dArc()
{
}

C2dArc::~C2dArc()
{
}

void C2dArc::Set(const C2dVect& center, const C2dVect& p1, const C2dVect& p2, double radius)
{
	m_center = center;
	m_norm1 = p1 - center;
	if (radius > 0)
	{
		m_radius = radius;
	}
	else
	{
		m_radius = m_norm1.Length();
	}
	m_norm1.Unitize();
	m_norm2 = p2 - center;
	m_norm2.Unitize();
}

void C2dArc::SetP1(const C2dVect& p, double radius)
{
	m_norm1 = p - m_center;
	if (radius > 0)
	{
		m_radius = radius;
	}
	else
	{
		m_radius = m_norm1.Length();
	}
	m_norm1.Unitize();
}

void C2dArc::SetP2(const C2dVect& p, double radius)
{
	m_norm2 = p - m_center;
	if (radius > 0)
	{
		m_radius = radius;
	}
	else
	{
		m_radius = m_norm2.Length();
	}
	m_norm2.Unitize();
}

C2dVect C2dArc::GetCenter()
{
	return m_center;
}

double  C2dArc::GetRadius()
{
	return m_radius;
}

C2dVect C2dArc::GetP1()
{
	return m_center + m_radius * m_norm1;
}

C2dVect C2dArc::GetP2()
{
	return m_center + m_radius * m_norm2;
}

bool C2dArc::Subtended(C2dVect v)
{
	/* v defines a direction. is this direction within the
		arc? */
	double sin1, sin2, cos1, cos2;

	v.Unitize();

	/* angle 1 is from norm1 to v */
	sin1 = CrossMag(m_norm1, v);
	cos1 = Dot(m_norm1, v);
	/* angle 2 is from norm1 to norm2 */
	sin2 = CrossMag(m_norm1, m_norm2);
	cos2 = Dot(m_norm1, m_norm2);

	/* now if angle 1 <= angle2, then v falls within the arc */
	if (sin1 >= 0) // angle 1 <= 180 deg
	{
		if (sin2 >= 0) // angle 2 <= 180 deg
		{
			/* both angles in 0..180, cos monotonically decreasing in this domain */
			return cos1 >= cos2;
		}
		else
		{
			return 0;
		}
	}
	else // angle 1 > 180 deg
	{
		if (sin2 >= 0) // angle 2 <= 180 deg
		{
			return 0;
		}
		else // angle 2 > 180 deg
		{
			/* both angles in 180..360, cos monotonically increasing in this domain */
			return cos1 <= cos2;
		}
	}
}

int C2dArc::ArcLine2dIntersection(C2dLine line,
	C2dVect& intersection_pt0,
	C2dVect& intersection_pt1)
{
	//C2dTMatrix t;
	/* point on the line nearest to center */
	C2dVect P = NearestPointToLine(m_center, line);
	/* distance from center to nearest point on the line */
	double d = (P - m_center).Length();
	int num_intersections = 0;
	double cord_dist;
	C2dVect l;

	if (m_radius < d)
		return 0;

	if (abs(m_radius - d) < 1e-6)
	{
		/* the line is tangent to the circle. if the tangent point
			is within the arc we have 1 intersection, else none */
		intersection_pt0 = intersection_pt1 = P;
		if (Subtended(P - m_center))
			num_intersections++;
		return num_intersections;
	}

	/* the line intersects the circle in 2 places, find them, and test
		each to see if they fall in the arc */
	l = line.v[1] - line.v[0];
	l.Unitize(); // unit vector along line
	cord_dist = sqrt(m_radius * m_radius - d * d);
	intersection_pt0 = P + cord_dist * l;
	intersection_pt1 = P - cord_dist * l;

	/* now test to see if subtented by the arc */
	if (Subtended(intersection_pt0 - m_center))
	{
		num_intersections++;
		if (Subtended(intersection_pt1 - m_center))
			num_intersections++;
	}
	else
	{
		if (Subtended(intersection_pt1 - m_center))
		{
			/* only point 1 is valid, move it to the point 0 spot */
			intersection_pt0 = intersection_pt1;
			num_intersections = 1;
		}
	}

	return num_intersections;
}

#endif // Very old unused stuff

// 3d stuff

//////////////////////////////////////////////////////////////////////////////////
// Point Helpers
//////////////////////////////////////////////////////////////////////////////////

#if 0
/* determine if points a and b are within sqrt(dist_sq) of each other */
bool isClosePointSq(const ur3v* a, const ur3v* b, double dist_sq)
{
	double dx = a->x - b->x;
	double dy = a->y - b->y;
	double dz = a->z - b->z;
	return (dx * dx + dy * dy + dz * dz <= dist_sq);
}

bool isClosePointSq(const ur3v& a, const ur3v& b, double dist_sq)
{
	double dx = a.x - b.x;
	double dy = a.y - b.y;
	double dz = a.z - b.z;
	return (dx * dx + dy * dy + dz * dz <= dist_sq);
}

bool isClosePointSq(const ur3Vf& a, const ur3Vf& b, float dist_sq)
{
	float dx = a.x - b.x;
	float dy = a.y - b.y;
	float dz = a.z - b.z;
	return (dx * dx + dy * dy + dz * dz <= dist_sq);
}

//////////////////////////////////////////////////////////////////////////////////
// Line Helpers
//////////////////////////////////////////////////////////////////////////////////

/* returns point on line defined by P1 and P2 nearest point Q */
//ur3v NearestPointOnLine(
//	const ur3v & P1,
//	const ur3v & P2,
//	const ur3v & Q)
//{
//	return urLine(P1, P2).NearestPoint(Q);
//}

bool isPointCloseToLineSq(
	const ur3v& Q,
	const ur3v& P1,
	const ur3v& P2,
	double dist_sq)
{
	ur3v line_vec = P2 - P1;
	double t = dot(line_vec, Q - P1);
	t /= line_vec.lengthSq();
	ur3v nearest_pt_on_line = P1 + t * line_vec;
	return isClosePointSq(Q, nearest_pt_on_line, dist_sq);
}

bool isPointCloseToLineSegmentSq(
	const ur3v& Q,
	const ur3v& P1,
	const ur3v& P2,
	double dist_sq)
{
	ur3v line_vec = P2 - P1;
	double t = dot(line_vec, Q - P1);
	double dsq = line_vec.lengthSq();
	bool nearest_pt_is_on_segment = t >= 0.0 && t <= dsq;

	ur3v nearest_pt_on_line = P1 + t / dsq * line_vec;

	/* if Q is further from the infinite line, than test dist, then false */
	if (!isClosePointSq(Q, nearest_pt_on_line, dist_sq))
		return false;

	/* Q is within test dist of the infinite line. if the nearest point on
		the line is on the line segment, then true */
	if (nearest_pt_is_on_segment)
		return true;

	/* nearest pt is not on line segment. Finally we just check to see if
		Q is within test dist of end points */
	return isClosePointSq(Q, P1, dist_sq) || isClosePointSq(Q, P2, dist_sq);
}

//////////////////////////////////////////////////////////////////////////////////
// 3D Angle Helpers
//////////////////////////////////////////////////////////////////////////////////

/* faster than GetAngle */
bool GetSinCos(
	const ur3v& toward_viewer, // unit length, pointing toward viewer
	const ur3v& start_vec,
	const ur3v& end_vec,
	double& sine,
	double& cosine)
{
	assert(toward_viewer.isUnitized());

	/* project start and end vectors onto viewer plane */
	ur3v start_proj = toward_viewer * start_vec * toward_viewer;
	ur3v end_proj = toward_viewer * end_vec * toward_viewer;

	/* make sure the projected vectors have meaningful length */
	double start_len_sq = start_proj.lengthSq();
	double end_len_sq = end_proj.lengthSq();
	if (start_len_sq < DLV_EFF_ZERO_SQ || end_len_sq < DLV_EFF_ZERO_SQ)
		return false;

	/* unitize */
	start_proj /= sqrt(start_len_sq);
	end_proj /= sqrt(end_len_sq);

	ur3v sine_vec = start_proj * end_proj;
	sine = sine_vec.dot(toward_viewer);
	cosine = start_proj.dot(end_proj);

	return true;
}

double GetAngle(const ur3v& start_vec, const ur3v& end_vec)
{
	ur3v toward_viewer = start_vec * end_vec;
	if (!toward_viewer.normalize())
	{
		if (start_vec.dot(end_vec) > 0)
		{
			/* Given vectors are in the same direction. */
			return 0;
		}
		else
		{
			/* Given vectors are in the opposite direction. */
			return dPI;
		}
	}
	return GetAngle(toward_viewer, start_vec, end_vec);
}

/* result is between -Pi to +Pi.
	In a right-hand coordinate system,
	when end_vec is counterclockwise from start_vec, the angle is positive */
double GetAngle(
	const ur3v& toward_viewer, // unit length, pointing toward viewer
	const ur3v& start_vec,
	const ur3v& end_vec,
	bool* result_ok/*=NULL*/)
{
	// CCWAngleBetween suffers from over agressive snap-to-zero. don't use it
	double sine, cosine;
	if (!GetSinCos(toward_viewer, start_vec, end_vec, sine, cosine))
	{
		if (result_ok)
			*result_ok = false;
		else
			assert(0);
		return 0.0;
	}
	if (result_ok)
		*result_ok = true;
	return atan2(sine, cosine);
}


/* a,b and c are lengths of the sides of a triangle
	a and b are lengths of adjacent sides
	c is length of opposite side
	aa is a*a, bb is b*b, cc is c*c
	returns the cosine of the angle between the adjacent sides */
double LawOfCosines(double a, double b, double cc)
{
	double aa = a * a;
	double bb = b * b;
	if (aa == 0.0 || bb == 0.0 || cc == 0.0)
		return 1.0; // call the angle 0, cosine = 1

	assert(a > 0.0 && b > 0.0);
	double cosine = (aa + bb - cc) / (2.0 * a * b);
	if (cosine > 1.0)
	{
		assert(cosine < 1.001);
		cosine = 1.0;
	}
	else if (cosine < -1.0)
	{
		assert(cosine > -1.001);
		cosine = -1.0;
	}
	return cosine;
}


ur3v Perpendicular(const ur3v& input)
{
	ur3v cross_x(input * ur3v(1, 0, 0));
	ur3v cross_y(input * ur3v(0, 1, 0));
	if (cross_x.length() > cross_y.length())
		return cross_x;
	else
		return cross_y;
}




// dl3dMatrix *****************************************************

dl3dMatrix::dl3dMatrix(int init)
{
	switch (init)
	{
	default: assert(0);
	case 0: *this = Zero; break;
	case 1: *this = Identity; break;
	}
}

dl3dMatrix::dl3dMatrix(double m00, double m01, double m02, double m03,
	double m10, double m11, double m12, double m13,
	double m20, double m21, double m22, double m23,
	double m30, double m31, double m32, double m33)
{
	m_matrix[0][0] = m00;	m_matrix[0][1] = m01;	m_matrix[0][2] = m02;	m_matrix[0][3] = m03;
	m_matrix[1][0] = m10;	m_matrix[1][1] = m11;	m_matrix[1][2] = m12;	m_matrix[1][3] = m13;
	m_matrix[2][0] = m20;	m_matrix[2][1] = m21;	m_matrix[2][2] = m22;	m_matrix[2][3] = m23;
	m_matrix[3][0] = m30;	m_matrix[3][1] = m31;	m_matrix[3][2] = m32;	m_matrix[3][3] = m33;
}

dl3dMatrix::dl3dMatrix(const urRotation& rotation)
{
	ur3v sX = rotation.GetXAxis();
	ur3v sY = rotation.GetYAxis();
	ur3v sZ = rotation.GetZAxis();

	m_matrix[0][0] = sX.x;
	m_matrix[0][1] = sY.x;
	m_matrix[0][2] = sZ.x;
	m_matrix[0][3] = 0.0;

	m_matrix[1][0] = sX.y;
	m_matrix[1][1] = sY.y;
	m_matrix[1][2] = sZ.y;
	m_matrix[1][3] = 0.0;

	m_matrix[2][0] = sX.z;
	m_matrix[2][1] = sY.z;
	m_matrix[2][2] = sZ.z;
	m_matrix[2][3] = 0.0;

	m_matrix[3][0] = 0.0;
	m_matrix[3][1] = 0.0;
	m_matrix[3][2] = 0.0;
	m_matrix[3][3] = 1.0;
}

#if 0
dl3dMatrix::dl3dMatrix(const IwAxis2Placement& a2p)
{
	mpgl::array2d<double, 4> x(4, 4);
	a2p.GetMatrix(x.pointers());
	memcpy(m_matrix, x.block(), sizeof(m_matrix));
}
#endif

dl3dMatrix::~dl3dMatrix()
{
}

dl3dMatrix dl3dMatrix::Identity(1, 0, 0, 0,
	0, 1, 0, 0,
	0, 0, 1, 0,
	0, 0, 0, 1);

dl3dMatrix dl3dMatrix::Zero(0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0,
	0, 0, 0, 0);

dl3dMatrix& dl3dMatrix::SetZero()
{
	*this = Zero;
	return *this;
}

dl3dMatrix& dl3dMatrix::SetIdentity()
{
	*this = Identity;
	return *this;
}

dl3dMatrix dl3dMatrix::operator*(const dl3dMatrix& mat2) const
{
	dl3dMatrix m;
	Mult(*this, mat2, m);
	return m;
}

dl3dMatrix& dl3dMatrix::PreMult(const dl3dMatrix& left)
{
	Mult(left, *this, *this);
	return *this;
}

dl3dMatrix& dl3dMatrix::Invert()
{
	// for implementation method see Graphic Gems I, p.470
	assert(0);
	return *this;
}

dl3dMatrix& dl3dMatrix::Scale(const ur3v& s, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = s.x;
		m_matrix[1][1] = s.y;
		m_matrix[2][2] = s.z;
	}
	else
	{
		dl3dMatrix m;
		m.Scale(s, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::Scale(double mx, double my, double mz, bool Initialize /* = false */)
{
	return Scale(ur3v(mx, my, mz), Initialize);
}

dl3dMatrix& dl3dMatrix::Translate(double dx, double dy, double dz, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][3] = dx;
		m_matrix[1][3] = dy;
		m_matrix[2][3] = dz;
	}
	else
	{
		m_matrix[0][3] += dx;
		m_matrix[1][3] += dy;
		m_matrix[2][3] += dz;
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::Translate(const ur3v& v, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][3] = v.x;
		m_matrix[1][3] = v.y;
		m_matrix[2][3] = v.z;
	}
	else
	{
		m_matrix[0][3] += v.x;
		m_matrix[1][3] += v.y;
		m_matrix[2][3] += v.z;
	}
	return *this;
}

ur3v dl3dMatrix::GetTranslation() const
{
	return ur3v(m_matrix[0][3], m_matrix[1][3], m_matrix[2][3]);
}

dl3dMatrix& dl3dMatrix::RotateX(double sine, double cosine, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[1][1] = cosine;
		m_matrix[2][2] = cosine;
		m_matrix[2][1] = sine;
		m_matrix[1][2] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateX(sine, cosine, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::RotateX(double Angle, bool Initialize /* = false */)
{
	if (Initialize)
	{
		double sine = sin(Angle), cosine = cos(Angle);
		SetIdentity();
		m_matrix[1][1] = cosine;
		m_matrix[2][2] = cosine;
		m_matrix[2][1] = sine;
		m_matrix[1][2] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateX(Angle, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::FlipX(bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[1][1] = -1;
		m_matrix[2][2] = -1;
	}
	else
	{
		dl3dMatrix m;
		m.FlipX(true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::RotateY(double sine, double cosine, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = cosine;
		m_matrix[2][2] = cosine;
		m_matrix[0][2] = sine;
		m_matrix[2][0] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateY(sine, cosine, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::RotateY(double Angle, bool Initialize /* = false */)
{
	if (Initialize)
	{
		double sine = sin(Angle), cosine = cos(Angle);
		SetIdentity();
		m_matrix[0][0] = cosine;
		m_matrix[2][2] = cosine;
		m_matrix[0][2] = sine;
		m_matrix[2][0] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateY(Angle, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::FlipY(bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = -1;
		m_matrix[2][2] = -1;
	}
	else
	{
		dl3dMatrix m;
		m.FlipY(true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::RotateZ(double sine, double cosine, bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = cosine;
		m_matrix[1][1] = cosine;
		m_matrix[1][0] = sine;
		m_matrix[0][1] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateZ(sine, cosine, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::RotateZ(double Angle, bool Initialize /* = false */)
{
	if (Initialize)
	{
		double sine = sin(Angle), cosine = cos(Angle);
		SetIdentity();
		m_matrix[0][0] = cosine;
		m_matrix[1][1] = cosine;
		m_matrix[1][0] = sine;
		m_matrix[0][1] = -sine;
	}
	else
	{
		dl3dMatrix m;
		m.RotateZ(Angle, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::FlipZ(bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = -1;
		m_matrix[1][1] = -1;
	}
	else
	{
		dl3dMatrix m;
		m.FlipZ(true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::Rotate(double sine, double cosine, const ur3v& RotVect, bool Initialize /* = false */)
{
	if (Initialize)
	{
		double OneMcos, x, y, z, xsin, ysin, zsin;
		double xy1mcos, xz1mcos, yz1mcos;
		ur3v rv = RotVect;
		rv.unitize(); // in case it wasn't
		/* here set m_matrix to the rotation matrix */
		x = rv.x;
		y = rv.y;
		z = rv.z;
		xsin = x * sine;
		ysin = y * sine;
		zsin = z * sine;
		OneMcos = 1.0f - cosine;
		xy1mcos = x * y * OneMcos;
		xz1mcos = x * z * OneMcos;
		yz1mcos = y * z * OneMcos;
		m_matrix[0][0] = x * x + cosine * (1.0 - x * x);
		m_matrix[0][1] = xy1mcos - zsin;
		m_matrix[0][2] = xz1mcos + ysin;
		m_matrix[0][3] = 0.0;
		m_matrix[1][0] = xy1mcos + zsin;
		m_matrix[1][1] = y * y + cosine * (1.0 - y * y);
		m_matrix[1][2] = yz1mcos - xsin;
		m_matrix[1][3] = 0.0;
		m_matrix[2][0] = xz1mcos - ysin;
		m_matrix[2][1] = yz1mcos + xsin;
		m_matrix[2][2] = z * z + cosine * (1.0 - z * z);
		m_matrix[2][3] = 0.0;
		m_matrix[3][0] = 0.0;
		m_matrix[3][1] = 0.0;
		m_matrix[3][2] = 0.0;
		m_matrix[3][3] = 1.0;
	}
	else
	{
		dl3dMatrix m;
		m.Rotate(sine, cosine, RotVect, true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::Rotate(double Angle, const ur3v& RotVect, bool Initialize /* = false */)
{
	Rotate(sin(Angle), cos(Angle), RotVect, Initialize);
	return *this;
}

dl3dMatrix& dl3dMatrix::Rotate(const ur3v& from, const ur3v& to, bool Initialize /* = false */)
{
	const ur3v crs(cross(from, to));
	const double dt = dot(from, to);
	const double magprod = from.length() * to.length();
	Rotate(crs.length() / magprod, dt / magprod, crs, Initialize);
	return *this;
}

dl3dMatrix& dl3dMatrix::MirrorX(bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[0][0] = -1;
	}
	else
	{
		dl3dMatrix m;
		m.MirrorX(true);
		PreMult(m);
	}
	return *this;
}

dl3dMatrix& dl3dMatrix::MirrorY(bool Initialize /* = false */)
{
	if (Initialize)
	{
		SetIdentity();
		m_matrix[1][1] = -1;
	}
	else
	{
		dl3dMatrix m;
		m.MirrorY(true);
		PreMult(m);
	}
	return *this;
}

void dl3dMatrix::TransformPoint(ur3v* p) const
{
	double newx, newy;
	newx = m_matrix[0][0] * p->x + m_matrix[0][1] * p->y + m_matrix[0][2] * p->z + m_matrix[0][3];
	newy = m_matrix[1][0] * p->x + m_matrix[1][1] * p->y + m_matrix[1][2] * p->z + m_matrix[1][3];
	p->z = m_matrix[2][0] * p->x + m_matrix[2][1] * p->y + m_matrix[2][2] * p->z + m_matrix[2][3];
	p->x = newx;
	p->y = newy;
}

ur3v dl3dMatrix::TransformPoint(const ur3v& p) const
{
	ur3v new_point;
	new_point.x = m_matrix[0][0] * p.x + m_matrix[0][1] * p.y + m_matrix[0][2] * p.z + m_matrix[0][3];
	new_point.y = m_matrix[1][0] * p.x + m_matrix[1][1] * p.y + m_matrix[1][2] * p.z + m_matrix[1][3];
	new_point.z = m_matrix[2][0] * p.x + m_matrix[2][1] * p.y + m_matrix[2][2] * p.z + m_matrix[2][3];
	return new_point;
}

void dl3dMatrix::TransformPoints(int NumPoints, ur3v* p, int byte_pitch) const
{
	double newx, newy;
	for (int i = 0; i < NumPoints; i++)
	{
		newx = m_matrix[0][0] * p->x + m_matrix[0][1] * p->y + m_matrix[0][2] * p->z + m_matrix[0][3];
		newy = m_matrix[1][0] * p->x + m_matrix[1][1] * p->y + m_matrix[1][2] * p->z + m_matrix[1][3];
		p->z = m_matrix[2][0] * p->x + m_matrix[2][1] * p->y + m_matrix[2][2] * p->z + m_matrix[2][3];
		p->x = newx;
		p->y = newy;
		p = (ur3v*)((u8*)p + byte_pitch);
	}
}

void dl3dMatrix::TransformPoints(urArray<ur3v>* points) const
{
	for (int i = 0; i < points->length(); i++)
		TransformPoint(&(*points)[i]);
}

//void dl3dMatrix::Trace()
//{
//	for (uint i = 0; i < 4; i++)
//	{
//		trace("[%.9f], [%.9f], [%.9f], [%.9f]\n", m_matrix[i][0],m_matrix[i][1],m_matrix[i][2],m_matrix[i][3]);
//	}
//}

std::ostream& operator<<(std::ostream& os, const dl3dMatrix& m)
{
	for (uint i = 0; i < 4; i++)
		os << m.GetElement(i, 0) << " " << m.GetElement(i, 1) << " " << m.GetElement(i, 2) << " " << m.GetElement(i, 3) << " " << std::endl;
	return os;
}

// static
void dl3dMatrix::Mult(const dl3dMatrix& left, const dl3dMatrix& right, dl3dMatrix& result)
{
	if (&left != &result && &right != &result)
	{
		for (int j = 0; j < 4; j++)
			for (int i = 0; i < 4; i++)
			{
				result.m_matrix[j][i] = left.m_matrix[j][0] * right.m_matrix[0][i]
					+ left.m_matrix[j][1] * right.m_matrix[1][i]
					+ left.m_matrix[j][2] * right.m_matrix[2][i]
					+ left.m_matrix[j][3] * right.m_matrix[3][i];
			}
	}
	else
	{
		dl3dMatrix RESULT;
		for (int j = 0; j < 4; j++)
			for (int i = 0; i < 4; i++)
			{
				RESULT.m_matrix[j][i] = left.m_matrix[j][0] * right.m_matrix[0][i]
					+ left.m_matrix[j][1] * right.m_matrix[1][i]
					+ left.m_matrix[j][2] * right.m_matrix[2][i]
					+ left.m_matrix[j][3] * right.m_matrix[3][i];
			}
		result = RESULT;
	}
}

ur3v operator*(const dl3dMatrix& m, const ur3v& v)
{
	ur3v t;
	t.x = m.m_matrix[0][0] * v.x + m.m_matrix[0][1] * v.y + m.m_matrix[0][2] * v.z + m.m_matrix[0][3];
	t.y = m.m_matrix[1][0] * v.x + m.m_matrix[1][1] * v.y + m.m_matrix[1][2] * v.z + m.m_matrix[1][3];
	t.z = m.m_matrix[2][0] * v.x + m.m_matrix[2][1] * v.y + m.m_matrix[2][2] * v.z + m.m_matrix[2][3];
	return t;
}

void urRotation::Rotate(double radians, const ur3v& rot_axis)
{
	assert(rot_axis.isUnitized());
	double s = sin(radians);
	double c = cos(radians);
	double t = 1 - c;

	ur3v sXAxis;
	sXAxis.x = t * rot_axis.x * rot_axis.x + c;
	sXAxis.y = t * rot_axis.x * rot_axis.y + s * rot_axis.z;
	sXAxis.z = t * rot_axis.x * rot_axis.z - s * rot_axis.y;

	ur3v sYAxis;
	sYAxis.x = t * rot_axis.x * rot_axis.y - s * rot_axis.z;
	sYAxis.y = t * rot_axis.y * rot_axis.y + c;
	sYAxis.z = t * rot_axis.y * rot_axis.z + s * rot_axis.x;

	sXAxis.unitize();
	sYAxis.unitize();
	sYAxis = sXAxis * sYAxis * sXAxis;
	sYAxis.unitize();

	Rotate(urRotation(sXAxis, sYAxis));
}

void urRotation::Rotate(const urRotation& other)
{
	ur3v sXAxis;
	ur3v sYAxis;
	ur3v sInZAxis = other.GetZAxis();

	sXAxis.x = m_xaxis.x * other.m_xaxis.x + m_xaxis.y * other.m_yaxis.x + m_xaxis.z * sInZAxis.x;
	sXAxis.y = m_xaxis.x * other.m_xaxis.y + m_xaxis.y * other.m_yaxis.y + m_xaxis.z * sInZAxis.y;
	sXAxis.z = m_xaxis.x * other.m_xaxis.z + m_xaxis.y * other.m_yaxis.z + m_xaxis.z * sInZAxis.z;

	sYAxis.x = m_yaxis.x * other.m_xaxis.x + m_yaxis.y * other.m_yaxis.x + m_yaxis.z * sInZAxis.x;
	sYAxis.y = m_yaxis.x * other.m_xaxis.y + m_yaxis.y * other.m_yaxis.y + m_yaxis.z * sInZAxis.y;
	sYAxis.z = m_yaxis.x * other.m_xaxis.z + m_yaxis.y * other.m_yaxis.z + m_yaxis.z * sInZAxis.z;

	m_xaxis = sXAxis;
	m_yaxis = sYAxis;
}

/***********************************************************************
PURPOSE --- Find the axis placement which is the inversion of the given
	 placement.  The resulting placement takes points from the global
	 coordinate system and transforms them into the local coordinate
	 system of the original placement.

USAGE NOTES ---
***********************************************************************/
void urRotation::Invert()
{
	ur3v sMyZAxis = GetZAxis();
	ur3v sXAxis(m_xaxis.x, m_yaxis.x, sMyZAxis.x);
	ur3v sYAxis(m_xaxis.y, m_yaxis.y, sMyZAxis.y);
	ur3v sZAxis(m_xaxis.z, m_yaxis.z, sMyZAxis.z);

	m_xaxis = sXAxis;
	m_yaxis = sYAxis;
}
#if 0
void urRotation::Invert(urRotation& result) const
{
	ur3v sMyZAxis = GetZAxis();
	ur3v sXAxis(m_xaxis.x, m_yaxis.x, sMyZAxis.x);
	ur3v sYAxis(m_xaxis.y, m_yaxis.y, sMyZAxis.y);
	ur3v sZAxis(m_xaxis.z, m_yaxis.z, sMyZAxis.z);

	result.m_xaxis = sXAxis;
	result.m_yaxis = sYAxis;
}
#endif

void urRotation::TransformPoint(ur3v* P) const
{
	ur3v& crInput = *P;
	ur3v Q;
	ur3v sZAxis = GetZAxis();
	Q.x = crInput.x * m_xaxis.x + crInput.y * m_yaxis.x + crInput.z * sZAxis.x;
	Q.y = crInput.x * m_xaxis.y + crInput.y * m_yaxis.y + crInput.z * sZAxis.y;
	Q.z = crInput.x * m_xaxis.z + crInput.y * m_yaxis.z + crInput.z * sZAxis.z;
	*P = Q;
}

ur3v urRotation::TransformPoint(const ur3v& P) const
{
	ur3v Q;
	ur3v sZAxis = GetZAxis();
	Q.x = P.x * m_xaxis.x + P.y * m_yaxis.x + P.z * sZAxis.x;
	Q.y = P.x * m_xaxis.y + P.y * m_yaxis.y + P.z * sZAxis.y;
	Q.z = P.x * m_xaxis.z + P.y * m_yaxis.z + P.z * sZAxis.z;
	return Q;
}
#endif
