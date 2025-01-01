function smf_dq_create_opt(radians, ax, ay, az, x, y, z) 
{
	//Creates a dual quaternion from axis angle and a translation vector
	//Source: http://en.wikipedia.org/wiki/Dual_quaternion
	radians *= .5;
	var c = cos(radians);
	var s = sin(radians);
	ax *= s;
	ay *= s;
	az *= s;

	return [ax, ay, az, c,
            .5 * dot_product_3d(x, y, -z, c, az, ax),
			.5 * dot_product_3d(y, z, -x, c, ax, az),
			.5 * dot_product_3d(z, x, -y, c, ay, ax),
			.5 * dot_product_3d(-x, -y, -z, ax, ay, az)];
}

/// @func smf_dq_create_from_matrix(M, targetDQ)
function smf_dq_create_from_matrix_opt(M, DQ) 
{
	//---------------Create dual quaternion from a matrix
	//Source: http://www.euclideanspace.com/maths/geometry/rotations/conversions/matrixToQuaternion/
	//Creates a dual quaternion from a matrix
	var T = 1 + M[0] + M[5] + M[10]
	if (T > 0.)
	{
	    var S = sqrt(T) * 2;
	    DQ[@ 0] = (M[9] - M[6]) / S;
	    DQ[@ 1] = (M[2] - M[8]) / S;
	    DQ[@ 2] = (M[4] - M[1]) / S;
	    DQ[@ 3] = -0.25 * S;  //I have modified this
	}
	else if (M[0] > M[5] && M[0] > M[10])
	{// Column 0: 
	   var S = sqrt(max(0., 1.0 + M[0] - M[5] - M[10])) * 2;
	    DQ[@ 0] = 0.25 * S;
	    DQ[@ 1] = (M[4] + M[1]) / S;
	    DQ[@ 2] = (M[2] + M[8]) / S;
	    DQ[@ 3] = (M[9] - M[6]) / S;
	} 
	else if (M[5] > M[10])
	{// Column 1: 
	    var S = sqrt(max(0., 1.0 + M[5] - M[0] - M[10])) * 2;
	    DQ[@ 0] = (M[4] + M[1]) / S;
	    DQ[@ 1] = 0.25 * S;
	    DQ[@ 2] = (M[9] + M[6]) / S;
	    DQ[@ 3] = (M[2] - M[8]) / S;
	} 
	else 
	{// Column 2:
		var S  = sqrt(max(0., 1.0 + M[10] - M[0] - M[5])) * 2;
	    DQ[@ 0] = (M[2] + M[8]) / S;
	    DQ[@ 1] = (M[9] + M[6]) / S;
	    DQ[@ 2] = 0.25 * S;
	    DQ[@ 3] = (M[4] - M[1]) / S;
	}
    var m12 = M[12], m13 = M[13], m14 = M[14];
    var dq0 = DQ[0], dq1 = DQ[1], dq2 = DQ[2], dq3 = DQ[3];
    DQ[@ 4] = .5 * dot_product_3d(m12, m13, -m14, dq3, dq2, dq1);
    DQ[@ 5] = .5 * dot_product_3d(m13, m14, -m12, dq3, dq0, dq2);
    DQ[@ 6] = .5 * dot_product_3d(m14, m12, -m13, dq3, dq1, dq0);
    DQ[@ 7] = .5 * dot_product_3d(m12, m13,  m14, dq0, dq1, dq2);
	return DQ;
}

#macro smf_dq_duplicate_opt variable_clone

// no changes...
function smf_dq_get_conjugate_opt(DQ, targetDQ = array_create(8))
{
	targetDQ[@ 0] = -DQ[0];
	targetDQ[@ 1] = -DQ[1];
	targetDQ[@ 2] = -DQ[2];
	targetDQ[@ 3] =  DQ[3];
	targetDQ[@ 4] = -DQ[4];
	targetDQ[@ 5] = -DQ[5];
	targetDQ[@ 6] = -DQ[6];
	targetDQ[@ 7] =  DQ[7];
	return targetDQ;
}
function smf_dq_get_translation_opt(DQ) 
{//Returns the translation of a given dual quaternion
	gml_pragma("forceinline");
	var q0 = DQ[0], q1 = DQ[1], q2 = DQ[2], q3 = DQ[3], q4 = DQ[4], q5 = DQ[5], q6 = DQ[6], q7 = DQ[7];
	return [2 * (dot_product_3d(-q7, q4, q6, q0, q3, q1) - q5 * q2),
			2 * (dot_product_3d(-q7, q5, q4, q1, q3, q2) - q6 * q0),
			2 * (dot_product_3d(-q7, q6, q5, q2, q3, q0) - q4 * q1)];
}

function smf_dq_get_x_opt(DQ) {
	//Returns the x component of the translation of a given dual quaternion
	gml_pragma("forceinline");
	return 2 * (dot_product_3d(-DQ[7], DQ[4], DQ[6], DQ[0], DQ[3], DQ[1]) - DQ[5] * DQ[2]);
}
function smf_dq_get_y_opt(DQ) 
{	//Returns the y component of the translation of a given dual quaternion
	gml_pragma("forceinline");
    return 2 * (dot_product_3d(-DQ[7], DQ[5], DQ[4], DQ[1], DQ[3], DQ[2]) - DQ[6] * DQ[0]);
}
function smf_dq_get_z_opt(DQ) 
{	//Returns the z component of the translation of a given dual quaternion
	gml_pragma("forceinline");
    return 2 * (dot_product_3d(-DQ[7], DQ[6], DQ[5], DQ[2], DQ[3], DQ[0]) - DQ[4] * DQ[1]);
}

// no changes...
function smf_dq_negate_opt(DQ, targetDQ = DQ) 
{
	targetDQ[@ 0] = -DQ[0];
	targetDQ[@ 1] = -DQ[1];
	targetDQ[@ 2] = -DQ[2];
	targetDQ[@ 3] = -DQ[3];
	targetDQ[@ 4] = -DQ[4];
	targetDQ[@ 5] = -DQ[5];
	targetDQ[@ 6] = -DQ[6];
	targetDQ[@ 7] = -DQ[7];
	return targetDQ;
}
// no changes...
function smf_dq_lerp_opt(DQ1, DQ2, amount, targetDQ = array_create(8)) 
{
	targetDQ[@ 0] = lerp(DQ1[0], DQ2[0], amount);
	targetDQ[@ 1] = lerp(DQ1[1], DQ2[1], amount);
	targetDQ[@ 2] = lerp(DQ1[2], DQ2[2], amount);
	targetDQ[@ 3] = lerp(DQ1[3], DQ2[3], amount);
	targetDQ[@ 4] = lerp(DQ1[4], DQ2[4], amount);
	targetDQ[@ 5] = lerp(DQ1[5], DQ2[5], amount);
	targetDQ[@ 6] = lerp(DQ1[6], DQ2[6], amount);
	targetDQ[@ 7] = lerp(DQ1[7], DQ2[7], amount);
	return targetDQ;
}
function smf_dq_multiply_opt(R, S, targetDQ = array_create(8)) 
{
	//Multiplies two dual quaternions and outputs the result to target
	//R * S = (A * C, A * D + B * C)
	var r0 = R[0], r1 = R[1], r2 = R[2], r3 = R[3], r4 = R[4], r5 = R[5], r6 = R[6], r7 = R[7];
	var s0 = S[0], s1 = S[1], s2 = S[2], s3 = S[3], s4 = S[4], s5 = S[5], s6 = S[6], s7 = S[7];
	targetDQ[@ 0] = dot_product_3d(r3,  r0,  r1, s0, s3, s2) - r2 * s1;
	targetDQ[@ 1] = dot_product_3d(r3,  r1,  r2, s1, s3, s0) - r0 * s2;
	targetDQ[@ 2] = dot_product_3d(r3,  r2,  r0, s2, s3, s1) - r1 * s0;
	targetDQ[@ 3] = dot_product_3d(r3, -r0, -r1, s3, s0, s1) - r2 * s2;
	targetDQ[@ 4] = dot_product_3d(r3,  r0,  r1, s4, s7, s6) + dot_product_3d(-r2, r7,  r4, s5, s0, s3) + dot_product( r5, -r6, s2, s1);
	targetDQ[@ 5] = dot_product_3d(r3,  r1,  r2, s5, s7, s4) + dot_product_3d(-r0, r7,  r5, s6, s1, s3) + dot_product( r6, -r4, s0, s2);
	targetDQ[@ 6] = dot_product_3d(r3,  r2,  r0, s6, s7, s5) + dot_product_3d(-r1, r7,  r6, s4, s2, s3) + dot_product( r4, -r5, s1, s0);
	targetDQ[@ 7] = dot_product_3d(r3, -r0, -r1, s7, s4, s5) + dot_product_3d(-r2, r7, -r4, s6, s3, s0) + dot_product(-r5, -r6, s1, s2);
	return targetDQ;
}

// no changes...
function smf_dq_normalize_opt(DQ, targetDQ = DQ)
{
	var q0 = DQ[0], q1 = DQ[1], q2 = DQ[2], q3 = DQ[3], q4 = DQ[4], q5 = DQ[5], q6 = DQ[6], q7 = DQ[7];
	var l = 1 / sqrt(q0 * q0 + q1 * q1 + q2 * q2 + q3 * q3);
	targetDQ[@ 0] = q0 * l
	targetDQ[@ 1] = q1 * l
	targetDQ[@ 2] = q2 * l
	targetDQ[@ 3] = q3 * l
	var d = l * (q0 * q4 + q1 * q5 + q2 * q6 + q3 * q7);
    var dl = d * l;
	targetDQ[@ 4] = dot_product(q4, q0, l, dl);
	targetDQ[@ 5] = dot_product(q5, q1, l, dl);
	targetDQ[@ 6] = dot_product(q6, q2, l, dl);
	targetDQ[@ 7] = dot_product(q7, q3, l, dl);
	return targetDQ;
}

function smf_dq_quadratic_interpolate_opt(A, B, C, amount, targetDQ = array_create(8)) 
{
	var t0 = .5 * sqr(1 - amount);
	var t1 = .5 * amount * amount;
	var t2 = 2 * amount * (1 - amount);
	var b0 = B[0], b1 = B[1], b2 = B[2], b3 = B[3], b4 = B[4], b5 = B[5], b6 = B[6], b7 = B[7];
	targetDQ[@ 0] = dot_product_3d(t0, t1, t2, A[0] + b0, b0 + C[0], b0);
	targetDQ[@ 1] = dot_product_3d(t0, t1, t2, A[1] + b1, b1 + C[1], b1);
	targetDQ[@ 2] = dot_product_3d(t0, t1, t2, A[2] + b2, b2 + C[2], b2);
	targetDQ[@ 3] = dot_product_3d(t0, t1, t2, A[3] + b3, b3 + C[3], b3);
	targetDQ[@ 4] = dot_product_3d(t0, t1, t2, A[4] + b4, b4 + C[4], b4);
	targetDQ[@ 5] = dot_product_3d(t0, t1, t2, A[5] + b5, b5 + C[5], b5);
	targetDQ[@ 6] = dot_product_3d(t0, t1, t2, A[6] + b6, b6 + C[6], b6);
	targetDQ[@ 7] = dot_product_3d(t0, t1, t2, A[7] + b7, b7 + C[7], b7);
	return targetDQ;
}

function smf_dq_set_translation_opt(DQ, x, y, z) 
{
	DQ[@ 4] = .5 * dot_product_3d(x, y, -z, DQ[3], DQ[2], DQ[1]); 
	DQ[@ 5] = .5 * dot_product_3d(y, z, -x, DQ[3], DQ[0], DQ[2]);
	DQ[@ 6] = .5 * dot_product_3d(z, x, -y, DQ[3], DQ[1], DQ[0]); 
	DQ[@ 7] =-.5 * dot_product_3d(x, y,  z, DQ[0], DQ[1], DQ[2]);
}
