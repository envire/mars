/** dMlsfield Collider
 * \file mlsfield.h
 * \author Yong-Ho Yoo
 * \brief mlsfield is based on the heightfield grid structure. 
 *        Heightfield triangles have been replaced by rectangulars and boxs. 
 *
 */


#include <ode/common.h>
#include <ode/collision.h>
#include <ode/rotation.h>
#include "config.h"
#include <ode/matrix.h>
#include <ode/odemath.h>
#include "collision_kernel.h"
#include "collision_std.h"
#include "collision_util.h"
#include "mlsfield.h"
#include <iostream>


#define dMIN(A,B)  ((A)>(B) ? (B) : (A))
#define dMAX(A,B)  ((A)>(B) ? (A) : (B))


// Three-way MIN and MAX
#define dMIN3(A,B,C)	( (A)<(B) ? dMIN((A),(C)) : dMIN((B),(C)) )
#define dMAX3(A,B,C)	( (A)>(B) ? dMAX((A),(C)) : dMAX((B),(C)) )

#define dOPESIGN(a, op1, op2,b) \
    (a)[0] op1 op2 ((b)[0]); \
    (a)[1] op1 op2 ((b)[1]); \
    (a)[2] op1 op2 ((b)[2]);


dxMlsfieldData::dxMlsfieldData():
    m_fWidth( 0 ),
    m_fDepth( 0 ),
    m_fSampleWidth( 0 ),
    m_fSampleDepth( 0 ),
    m_fSampleZXAspect( 0 ),
    m_fInvSampleWidth( 0 ),
    m_fInvSampleDepth( 0 ),

    m_fHalfWidth( 0 ),
    m_fHalfDepth( 0 ),

    m_fMinHeight( 0 ),
    m_fMaxHeight( 0 ),
    m_fThickness( 0 ),
    m_fScale( 0 ),
    m_fOffset( 0 ),

    m_nWidthSamples( 0 ),
    m_nDepthSamples( 0 ),
    m_bCopyHeightData( 0 ),
    m_bWrapMode( 0 ),
    m_nGetHeightMode( 0 ),

    m_pHeightData( NULL ),
    m_pUserData( NULL ),

    m_pGetHeightCallback( NULL )
{
    memset( m_contacts, 0, sizeof( m_contacts ) );
}

void dxMlsfieldData::SetData( int nWidthSamples, int nDepthSamples,
                                dReal fWidth, dReal fDepth,
                                dReal fScale, dReal fOffset, dReal fThickness,
                                int bWrapMode )
{
    dIASSERT( fWidth > REAL( 0.0 ) );
    dIASSERT( fDepth > REAL( 0.0 ) );
    dIASSERT( nWidthSamples > 0 );
    dIASSERT( nDepthSamples > 0 );

    // x,z bounds
    m_fWidth = fWidth;
    m_fDepth = fDepth;

    // cache half x,z bounds
    m_fHalfWidth = fWidth / REAL( 2.0 );
    m_fHalfDepth = fDepth / REAL( 2.0 );

    // scale and offset
    m_fScale = fScale;
    m_fOffset = fOffset;

    // infinite min height bounds
    m_fThickness = fThickness;

    // number of vertices per side
    m_nWidthSamples = nWidthSamples;
    m_nDepthSamples = nDepthSamples;

    m_fSampleWidth = m_fWidth / ( m_nWidthSamples - REAL( 1.0 ) );
    m_fSampleDepth = m_fDepth / ( m_nDepthSamples - REAL( 1.0 ) );

    m_fSampleZXAspect = m_fSampleDepth / m_fSampleWidth;

    m_fInvSampleWidth = REAL( 1.0 ) / m_fSampleWidth;
    m_fInvSampleDepth = REAL( 1.0 ) / m_fSampleDepth;

    // finite or repeated terrain?
    m_bWrapMode = bWrapMode;
}


// recomputes heights bounds
void dxMlsfieldData::ComputeHeightBounds()
{
    int i;
    dReal h;
    unsigned char *data_byte;
    short *data_short;
    float *data_float;
    double *data_double;

    switch ( m_nGetHeightMode )  
    {

        // callback
    case 0:
        // change nothing, keep using default or user specified bounds
        return;

        // byte
    case 1:
        data_byte = (unsigned char*)m_pHeightData;
        m_fMinHeight = dInfinity;
        m_fMaxHeight = -dInfinity;

        for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
        {
            h = data_byte[i];
            if (h < m_fMinHeight)	m_fMinHeight = h;
            if (h > m_fMaxHeight)	m_fMaxHeight = h;
        }

        break;

        // short
    case 2:
        data_short = (short*)m_pHeightData;
        m_fMinHeight = dInfinity;
        m_fMaxHeight = -dInfinity;

        for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
        {
            h = data_short[i];
            if (h < m_fMinHeight)	m_fMinHeight = h;
            if (h > m_fMaxHeight)	m_fMaxHeight = h;
        }

        break;

        // float
    case 3:
        data_float = (float*)m_pHeightData;
        m_fMinHeight = dInfinity;
        m_fMaxHeight = -dInfinity;

        for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
        {
            h = data_float[i];
            if (h < m_fMinHeight)	m_fMinHeight = h;
            if (h > m_fMaxHeight)	m_fMaxHeight = h;
        }

        break;

        // double
    case 4:
        data_double = (double*)m_pHeightData;
        m_fMinHeight = dInfinity;
        m_fMaxHeight = -dInfinity;

        for (i=0; i<m_nWidthSamples*m_nDepthSamples; i++)
        {
            h = static_cast< dReal >( data_double[i] );
            if (h < m_fMinHeight)	m_fMinHeight = h;
            if (h > m_fMaxHeight)	m_fMaxHeight = h;
        }

        break;

    }

    // scale and offset
    m_fMinHeight *= m_fScale;
    m_fMaxHeight *= m_fScale;
    m_fMinHeight += m_fOffset;
    m_fMaxHeight += m_fOffset;

    // add thickness
    m_fMinHeight -= m_fThickness;
}


// returns height at given sample coordinates
dReal dxMlsfieldData::GetHeight( int x, int z )
{
    dReal h=0;
    unsigned char *data_byte;
    short *data_short;
    float *data_float;
    double *data_double;

    if ( m_bWrapMode == 0 )
    {
        // Finite
        if ( x < 0 ) x = 0;
        if ( z < 0 ) z = 0;
        if ( x > m_nWidthSamples - 1 ) x = m_nWidthSamples - 1;
        if ( z > m_nDepthSamples - 1 ) z = m_nDepthSamples - 1;
    }
    else
    {
        // Infinite
        x %= m_nWidthSamples - 1;
        z %= m_nDepthSamples - 1;
        if ( x < 0 ) x += m_nWidthSamples - 1;
        if ( z < 0 ) z += m_nDepthSamples - 1;
    }

    switch ( m_nGetHeightMode )
    {

        // callback (dReal)
    case 0:
        h = (*m_pGetHeightCallback)(m_pUserData, x, z);
        break;

        // byte
    case 1:
        data_byte = (unsigned char*)m_pHeightData;
        h = data_byte[x+(z * m_nWidthSamples)];
        break;

        // short
    case 2:
        data_short = (short*)m_pHeightData;
        h = data_short[x+(z * m_nWidthSamples)];
        break;

        // float
    case 3:
        data_float = (float*)m_pHeightData;
        h = data_float[x+(z * m_nWidthSamples)];
        break;

        // double
    case 4:
        data_double = (double*)m_pHeightData;
        h = (dReal)( data_double[x+(z * m_nWidthSamples)] );
        break;
    }

    return (h * m_fScale) + m_fOffset;
}


// returns height at given coordinates
dReal dxMlsfieldData::GetHeight( dReal x, dReal z )
{
    dReal dnX = dFloor( x * m_fInvSampleWidth );
    dReal dnZ = dFloor( z * m_fInvSampleDepth );

    dReal dx = ( x - ( dnX * m_fSampleWidth ) ) * m_fInvSampleWidth;
    dReal dz = ( z - ( dnZ * m_fSampleDepth ) ) * m_fInvSampleDepth;
   
    int nX = int( dnX );
    int nZ = int( dnZ );

    dReal y, y0;

    if ( dx + dz <= REAL( 1.0 ) ) // Use <= comparison to prefer simpler branch
    {
        y0 = GetHeight( nX, nZ );

        y = y0 + ( GetHeight( nX + 1, nZ ) - y0 ) * dx
            + ( GetHeight( nX, nZ + 1 ) - y0 ) * dz;
    }
    else
    {
        y0 = GetHeight( nX + 1, nZ + 1 );

        y = y0	+ ( GetHeight( nX + 1, nZ ) - y0 ) * ( REAL(1.0) - dz ) +
            ( GetHeight( nX, nZ + 1 ) - y0 ) * ( REAL(1.0) - dx );
    }

    return y;
}

dxMlsfieldData::~dxMlsfieldData()
{
    unsigned char *data_byte;
    short *data_short;
    float *data_float;
    double *data_double;

    if ( m_bCopyHeightData )
    {
        switch ( m_nGetHeightMode )
        {

            // callback
        case 0:
            // do nothing
            break;

            // byte
        case 1:
            dIASSERT( m_pHeightData );
            data_byte = (unsigned char*)m_pHeightData;
            delete [] data_byte;
            break;

            // short
        case 2:
            dIASSERT( m_pHeightData );
            data_short = (short*)m_pHeightData;
            delete [] data_short;
            break;

            // float
        case 3:
            dIASSERT( m_pHeightData );
            data_float = (float*)m_pHeightData;
            delete [] data_float;
            break;

            // double
        case 4:
            dIASSERT( m_pHeightData );
            data_double = (double*)m_pHeightData;
            delete [] data_double;
            break;

        }
    }
}


// here we define new collider() functions according to the class numbers
dColliderFn * setColliders(int num){     	
	
	printf("dxGeom type num = %d\n", num);
	
	dColliderFn *fn;
	//if(num == 0 || num == 4) {		//dSphereClass  or dRayClass
	if(num == 0) {		 				//dSphereClass  	
		fn = &dCollideMlsfield;
		return fn;
	} 
	else return NULL;
}

dxMlsfield::dxMlsfield( dSpaceID space,
                             dMlsfieldDataID data,
                             int bPlaceable )			:
    dxGeom( space, bPlaceable ),
    tempRectangularBuffer(0),
    tempRectangularBufferSize(0),
    tempHeightBuffer(0),
    tempHeightInstances(0),
    tempHeightBufferSizeX(0),
    tempHeightBufferSizeZ(0)
{
    //to create the function pointer of the dColliderFnFn which uses for dCollideUserGeomWithGeom   
    dGeomClass mls_colliders;	
    mls_colliders.collider = setColliders;   
    type = dCreateGeomClass(&mls_colliders);
    
    this->m_p_data = data;
    
}


// compute axis aligned bounding box
void dxMlsfield::computeAABB()
{
    const dxMlsfieldData *d = m_p_data;

    if ( d->m_bWrapMode == 0 )
    {
        // Finite
        if ( gflags & GEOM_PLACEABLE )
        {
            dReal dx[6], dy[6], dz[6];

            // Y-axis
            if (d->m_fMinHeight != -dInfinity)
            {
                dy[0] = ( final_posr->R[ 1] * d->m_fMinHeight );
                dy[1] = ( final_posr->R[ 5] * d->m_fMinHeight );
                dy[2] = ( final_posr->R[ 9] * d->m_fMinHeight );
            }
            else
            {
                // Multiplication is performed to obtain infinity of correct sign
                dy[0] = ( final_posr->R[ 1] ? final_posr->R[ 1] * -dInfinity : REAL(0.0) );
                dy[1] = ( final_posr->R[ 5] ? final_posr->R[ 5] * -dInfinity : REAL(0.0) );
                dy[2] = ( final_posr->R[ 9] ? final_posr->R[ 9] * -dInfinity : REAL(0.0) );
            }

            if (d->m_fMaxHeight != dInfinity)
            {
                dy[3] = ( final_posr->R[ 1] * d->m_fMaxHeight );
                dy[4] = ( final_posr->R[ 5] * d->m_fMaxHeight );
                dy[5] = ( final_posr->R[ 9] * d->m_fMaxHeight );
            }
            else
            {
                dy[3] = ( final_posr->R[ 1] ? final_posr->R[ 1] * dInfinity : REAL(0.0) );
                dy[4] = ( final_posr->R[ 5] ? final_posr->R[ 5] * dInfinity : REAL(0.0) );
                dy[5] = ( final_posr->R[ 9] ? final_posr->R[ 9] * dInfinity : REAL(0.0) );
            }

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

            // X-axis
            dx[0] = 0;	dx[3] = ( final_posr->R[ 0] * d->m_fWidth );
            dx[1] = 0;	dx[4] = ( final_posr->R[ 4] * d->m_fWidth );
            dx[2] = 0;	dx[5] = ( final_posr->R[ 8] * d->m_fWidth );

            // Z-axis
            dz[0] = 0;	dz[3] = ( final_posr->R[ 2] * d->m_fDepth );
            dz[1] = 0;	dz[4] = ( final_posr->R[ 6] * d->m_fDepth );
            dz[2] = 0;	dz[5] = ( final_posr->R[10] * d->m_fDepth );

#else // DHEIGHTFIELD_CORNER_ORIGIN

            // X-axis
            dx[0] = ( final_posr->R[ 0] * -d->m_fHalfWidth );
            dx[1] = ( final_posr->R[ 4] * -d->m_fHalfWidth );
            dx[2] = ( final_posr->R[ 8] * -d->m_fHalfWidth );
            dx[3] = ( final_posr->R[ 0] * d->m_fHalfWidth );
            dx[4] = ( final_posr->R[ 4] * d->m_fHalfWidth );
            dx[5] = ( final_posr->R[ 8] * d->m_fHalfWidth );

            // Z-axis
            dz[0] = ( final_posr->R[ 2] * -d->m_fHalfDepth );
            dz[1] = ( final_posr->R[ 6] * -d->m_fHalfDepth );
            dz[2] = ( final_posr->R[10] * -d->m_fHalfDepth );
            dz[3] = ( final_posr->R[ 2] * d->m_fHalfDepth );
            dz[4] = ( final_posr->R[ 6] * d->m_fHalfDepth );
            dz[5] = ( final_posr->R[10] * d->m_fHalfDepth );

#endif // DHEIGHTFIELD_CORNER_ORIGIN

            // X extents
            aabb[0] = final_posr->pos[0] +
                dMIN3( dMIN( dx[0], dx[3] ), dMIN( dy[0], dy[3] ), dMIN( dz[0], dz[3] ) );
            aabb[1] = final_posr->pos[0] +
                dMAX3( dMAX( dx[0], dx[3] ), dMAX( dy[0], dy[3] ), dMAX( dz[0], dz[3] ) );

            // Y extents
            aabb[2] = final_posr->pos[1] +
                dMIN3( dMIN( dx[1], dx[4] ), dMIN( dy[1], dy[4] ), dMIN( dz[1], dz[4] ) );
            aabb[3] = final_posr->pos[1] +
                dMAX3( dMAX( dx[1], dx[4] ), dMAX( dy[1], dy[4] ), dMAX( dz[1], dz[4] ) );

            // Z extents
            aabb[4] = final_posr->pos[2] +
                dMIN3( dMIN( dx[2], dx[5] ), dMIN( dy[2], dy[5] ), dMIN( dz[2], dz[5] ) );
            aabb[5] = final_posr->pos[2] +
                dMAX3( dMAX( dx[2], dx[5] ), dMAX( dy[2], dy[5] ), dMAX( dz[2], dz[5] ) );
        }
        else
        {

#ifdef DHEIGHTFIELD_CORNER_ORIGIN

            aabb[0] = 0;					aabb[1] = d->m_fWidth;
            aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
            aabb[4] = 0;					aabb[5] = d->m_fDepth;

#else // DHEIGHTFIELD_CORNER_ORIGIN

            aabb[0] = -d->m_fHalfWidth;		aabb[1] = +d->m_fHalfWidth;
            aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
            aabb[4] = -d->m_fHalfDepth;		aabb[5] = +d->m_fHalfDepth;

#endif // DHEIGHTFIELD_CORNER_ORIGIN

        }
    }
    else
    {
        // Infinite
        if ( gflags & GEOM_PLACEABLE )
        {
            aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
            aabb[2] = -dInfinity;			aabb[3] = +dInfinity;
            aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
        }
        else
        {
            aabb[0] = -dInfinity;			aabb[1] = +dInfinity;
            aabb[2] = d->m_fMinHeight;		aabb[3] = d->m_fMaxHeight;
            aabb[4] = -dInfinity;			aabb[5] = +dInfinity;
        }
    }

}


// dxMlsfield destructor
dxMlsfield::~dxMlsfield()
{	
	resetRectangularBuffer();
}

void dxMlsfield::allocateRectangularBuffer(size_t numRect)
{
    size_t alignedNumRect = AlignBufferSize(numRect, TEMP_RECTANGULAR_BUFFER_ELEMENT_COUNT_ALIGNMENT);
    tempRectangularBufferSize = alignedNumRect;
    tempRectangularBuffer = new MlsFieldRectangular[alignedNumRect];
}
void dxMlsfield::resetRectangularBuffer()
{
    delete[] tempRectangularBuffer;
}


void dxMlsfield::resetHeightBuffer()
{
    delete[] tempHeightInstances;
    delete[] tempHeightBuffer;
}


void dxMlsfield::allocateHeightBuffer(size_t numX, size_t numZ)
{
    size_t alignedNumX = AlignBufferSize(numX, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_X);
    size_t alignedNumZ = AlignBufferSize(numZ, TEMP_HEIGHT_BUFFER_ELEMENT_COUNT_ALIGNMENT_Z);
    tempHeightBufferSizeX = alignedNumX;
    tempHeightBufferSizeZ = alignedNumZ;
    tempHeightBuffer = new MlsFieldVertex *[alignedNumX];
    size_t numCells = alignedNumX * alignedNumZ;
    tempHeightInstances = new MlsFieldVertex [numCells];

    MlsFieldVertex *ptrHeightMatrix = tempHeightInstances;
    for (size_t indexX = 0; indexX != alignedNumX; indexX++)
    {
        tempHeightBuffer[indexX] = ptrHeightMatrix;
        ptrHeightMatrix += alignedNumZ;
    }
}

dMlsfieldDataID dGeomMlsfieldDataCreate()
{
    return new dxMlsfieldData();
}


void dGeomMlsfieldDataBuildCallback( dMlsfieldDataID d,
                                       void* pUserData, dMlsfieldGetHeight* pCallback,
                                       dReal width, dReal depth, int widthSamples, int depthSamples,
                                       dReal scale, dReal offset, dReal thickness, int bWrap )
{
    dUASSERT( d, "argument not Mlsfield data" );
    dIASSERT( pCallback );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // callback
    d->m_nGetHeightMode = 0;
    d->m_pUserData = pUserData;
    d->m_pGetHeightCallback = pCallback;
    
    printf("--------------(widthSamples, depthSamples) = (%d %d) \n",widthSamples, depthSamples);

    // set info
    d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );

    // default bounds
    d->m_fMinHeight = -dInfinity;
    d->m_fMaxHeight = dInfinity;
}


void dGeomMlsfieldDataBuildByte( dMlsfieldDataID d,
                                   const unsigned char *pHeightData, int bCopyHeightData,
                                   dReal width, dReal depth, int widthSamples, int depthSamples,
                                   dReal scale, dReal offset, dReal thickness, int bWrap )
{
    dUASSERT( d, "Argument not Mlsfield data" );
    dIASSERT( pHeightData );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // set info
    d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
    d->m_nGetHeightMode = 1;
    d->m_bCopyHeightData = bCopyHeightData;

    if ( d->m_bCopyHeightData == 0 )
    {
        // Data is referenced only.
        d->m_pHeightData = pHeightData;
    }
    else
    {
        // We own the height data, allocate storage
        d->m_pHeightData = new unsigned char[ d->m_nWidthSamples * d->m_nDepthSamples ];
        dIASSERT( d->m_pHeightData );

        // Copy data.
        memcpy( (void*)d->m_pHeightData, pHeightData,
            sizeof( unsigned char ) * d->m_nWidthSamples * d->m_nDepthSamples );
    }

    // Find height bounds
    d->ComputeHeightBounds();
}


void dGeomMlsfieldDataBuildShort( dMlsfieldDataID d,
                                    const short* pHeightData, int bCopyHeightData,
                                    dReal width, dReal depth, int widthSamples, int depthSamples,
                                    dReal scale, dReal offset, dReal thickness, int bWrap )
{
    dUASSERT( d, "Argument not Mlsfield data" );
    dIASSERT( pHeightData );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // set info
    d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
    d->m_nGetHeightMode = 2;
    d->m_bCopyHeightData = bCopyHeightData;

    if ( d->m_bCopyHeightData == 0 )
    {
        // Data is referenced only.
        d->m_pHeightData = pHeightData;
    }
    else
    {
        // We own the height data, allocate storage
        d->m_pHeightData = new short[ d->m_nWidthSamples * d->m_nDepthSamples ];
        dIASSERT( d->m_pHeightData );

        // Copy data.
        memcpy( (void*)d->m_pHeightData, pHeightData,
            sizeof( short ) * d->m_nWidthSamples * d->m_nDepthSamples );
    }

    // Find height bounds
    d->ComputeHeightBounds();
}


void dGeomMlsfieldDataBuildSingle( dMlsfieldDataID d,
                                     const float *pHeightData, int bCopyHeightData,
                                     dReal width, dReal depth, int widthSamples, int depthSamples,
                                     dReal scale, dReal offset, dReal thickness, int bWrap )
{
    dUASSERT( d, "Argument not Mlsfield data" );
    dIASSERT( pHeightData );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // set info
    d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
    d->m_nGetHeightMode = 3;
    d->m_bCopyHeightData = bCopyHeightData;

    if ( d->m_bCopyHeightData == 0 )
    {
        // Data is referenced only.
        d->m_pHeightData = pHeightData;
    }
    else
    {
        // We own the height data, allocate storage
        d->m_pHeightData = new float[ d->m_nWidthSamples * d->m_nDepthSamples ];
        dIASSERT( d->m_pHeightData );

        // Copy data.
        memcpy( (void*)d->m_pHeightData, pHeightData,
            sizeof( float ) * d->m_nWidthSamples * d->m_nDepthSamples );
    }

    // Find height bounds
    d->ComputeHeightBounds();
}

void dGeomMlsfieldDataBuildDouble( dMlsfieldDataID d,
                                     const double *pHeightData, int bCopyHeightData,
                                     dReal width, dReal depth, int widthSamples, int depthSamples,
                                     dReal scale, dReal offset, dReal thickness, int bWrap )
{
    dUASSERT( d, "Argument not Heightfield data" );
    dIASSERT( pHeightData );
    dIASSERT( widthSamples >= 2 );	// Ensure we're making something with at least one cell.
    dIASSERT( depthSamples >= 2 );

    // set info
    d->SetData( widthSamples, depthSamples, width, depth, scale, offset, thickness, bWrap );
    d->m_nGetHeightMode = 4;
    d->m_bCopyHeightData = bCopyHeightData;

    if ( d->m_bCopyHeightData == 0 )
    {
        // Data is referenced only.
        d->m_pHeightData = pHeightData;
    }
    else
    {
        // We own the height data, allocate storage
        d->m_pHeightData = new double[ d->m_nWidthSamples * d->m_nDepthSamples ];
        dIASSERT( d->m_pHeightData );

        // Copy data.
        memcpy( (void*)d->m_pHeightData, pHeightData,
            sizeof( double ) * d->m_nWidthSamples * d->m_nDepthSamples );
    }

    // Find height bounds
    d->ComputeHeightBounds();
}




void dGeomMlsfieldDataSetBounds( dMlsfieldDataID d, dReal minHeight, dReal maxHeight )
{
    dUASSERT(d, "Argument not Mlsfield data");
    d->m_fMinHeight = ( minHeight * d->m_fScale ) + d->m_fOffset - d->m_fThickness;
    d->m_fMaxHeight = ( maxHeight * d->m_fScale ) + d->m_fOffset;
}


void dGeomMlsfieldDataDestroy( dMlsfieldDataID d )
{
    dUASSERT(d, "argument not Mlsfield data");
    delete d;
}


//////// Mlsfield geom interface ////////////////////////////////////////////////////


dGeomID dCreateMlsfield( dSpaceID space, dMlsfieldDataID data, int bPlaceable )
{
    return new dxMlsfield( space, data, bPlaceable );
}


void dGeomMlsfieldSetMlsfieldData( dGeomID g, dMlsfieldDataID d )
{
    dxMlsfield* geom = (dxMlsfield*) g;
    geom->m_p_data = d;
}


dMlsfieldDataID dGeomMlsfieldGetMlsfieldData( dGeomID g )
{
    dxMlsfield* geom = (dxMlsfield*) g;
    return geom->m_p_data;
}

//////// dxMlsfield /////////////////////////////////////////////////////////////////


// Typedef for generic 'get point depth' function
typedef dReal dGetDepthFn( dGeomID g, dReal x, dReal y, dReal z );


#define DMESS(A)	\
    dMessage(0,"Contact Plane (%d %d %d) %.5e %.5e (%.5e %.5e %.5e)(%.5e %.5e %.5e)).",	\
    x,z,(A),	\
    pContact->depth,	\
    dGeomSphereGetRadius(o2),		\
    pContact->pos[0],	\
    pContact->pos[1],	\
    pContact->pos[2],	\
    pContact->normal[0],	\
    pContact->normal[1],	\
    pContact->normal[2]);

//static inline bool DescendingRectangularSort(const MlsFieldRectangular * const A, const MlsFieldRectangular * const B)
//{
    //return ((A->maxAAAB - B->maxAAAB) > dEpsilon);
//}



int dxMlsfield::dCollideMlsfieldZone( const int minX, const int maxX, const int minZ, const int maxZ, 
                                           dxGeom* o2, const int numMaxContactsPossible,
                                           int flags, dContactGeom* contact, 
                                           int skip )
{
    dContactGeom *pContact = 0;
//printf(".called mls...1...\n");
    int  x, z;
    // check if not above or inside terrain first
    // while filling a Mlsmap partial temporary buffer
    const unsigned int numX = (maxX - minX) + 1;
    const unsigned int numZ = (maxZ - minZ) + 1;
    const dReal minO2Height = o2->aabb[2];
    const dReal maxO2Height = o2->aabb[3];
    unsigned int x_local, z_local;
    dReal maxY = - dInfinity;
    dReal minY = dInfinity;
    
    int numTerrainContacts = 0;
    // localize and const for faster access
    
    const dReal cfSampleWidth = m_p_data->m_fSampleWidth;
    const dReal cfSampleDepth = m_p_data->m_fSampleDepth;
    {
        if (tempHeightBufferSizeX < numX || tempHeightBufferSizeZ < numZ)
        {
            resetHeightBuffer();
            allocateHeightBuffer(numX, numZ);
        }

        dReal Xpos, Ypos;

        for ( x = minX, x_local = 0; x_local < numX; x++, x_local++)
        {
            Xpos = x * cfSampleWidth; // Always calculate pos via multiplication to avoid computational error accumulation during multiple additions

            const dReal c_Xpos = Xpos;
            MlsFieldVertex *MlsFieldRow = tempHeightBuffer[x_local];
            for ( z = minZ, z_local = 0; z_local < numZ; z++, z_local++)       //[x_local][z_local]...YH
            {
                Ypos = z * cfSampleDepth; // Always calculate pos via multiplication to avoid computational error accumulation during multiple additions

                const dReal h = m_p_data->GetHeight(x, z);
                MlsFieldRow[z_local].vertex[0] = c_Xpos;
                MlsFieldRow[z_local].vertex[1] = h;
                MlsFieldRow[z_local].vertex[2] = Ypos;

                maxY = dMAX(maxY, h);
                minY = dMIN(minY, h);
            }
        }


        if (minO2Height - maxY > -dEpsilon )    //TODO: make clear!!
        {
            //totally above Mlsfield
            return 0;
        }
        
        if (minY - maxO2Height > -dEpsilon )
        {
            // totally under Mlsfield
            pContact = CONTACT(contact, 0);			//set pointer of dContactGeom....YH

            pContact->pos[0] = o2->final_posr->pos[0];
            pContact->pos[1] = minY;
            pContact->pos[2] = o2->final_posr->pos[2];

            pContact->normal[0] = 0;
            pContact->normal[1] = -1;
            pContact->normal[2] = 0;

            pContact->depth =  minY - maxO2Height;

            pContact->side1 = -1;
            pContact->side2 = -1;
printf(".totally under Mlsfield...\n");
            return 1;
//			return numTerrainContacts++;
        }
        
    }




    dContactGeom *BoxContact = m_p_data->m_contacts;  

    const unsigned int numRectMax = (maxX - minX) * (maxZ - minZ);
    if (tempRectangularBufferSize < numRectMax)
    {
        resetRectangularBuffer();
        allocateRectangularBuffer(numRectMax);
    }


    unsigned int numRect = 0;
    MlsFieldVertex *A, *B, *C, *D;
    /*    (y is up)
         A--------B-...x
         |        |
         |        |
         |        |
         C--------D   
         .
         .
         .
         z
    */  

    // keep only rectangular that does intersect geom
    const unsigned int maxX_local = maxX - minX;
    const unsigned int maxZ_local = maxZ - minZ;

    for ( x_local = 0; x_local < maxX_local; x_local++)
    {
        MlsFieldVertex *MlsFieldRow      = tempHeightBuffer[x_local];
        MlsFieldVertex *MlsFieldNextRow  = tempHeightBuffer[x_local + 1];

        // First A
        C = &MlsFieldRow    [0];
        // First B
        D = &MlsFieldNextRow[0];

        for ( z_local = 0; z_local < maxZ_local; z_local++)
        {
            A = C;
            B = D;

            C = &MlsFieldRow    [z_local + 1];
            D = &MlsFieldNextRow[z_local + 1];

            const dReal AHeight = A->vertex[1];
            const dReal BHeight = B->vertex[1];
            const dReal CHeight = C->vertex[1];
            const dReal DHeight = D->vertex[1];

            const bool isACollide = AHeight > minO2Height;
            const bool isBCollide = BHeight > minO2Height;
            const bool isCCollide = CHeight > minO2Height;
            const bool isDCollide = DHeight > minO2Height;

            if (isACollide || isBCollide || isCCollide || isDCollide)
            {
                MlsFieldRectangular * const CurrRectUp = &tempRectangularBuffer[numRect++];

              // changing point order here implies to change it in isOnHeightField
                CurrRectUp->vertices[0] = A;
                CurrRectUp->vertices[1] = B;
                CurrRectUp->vertices[2] = C;
                CurrRectUp->vertices[3] = D;

            }
        }
    }
    
    //make dxBox[numRect] which have collided     
    dIASSERT (numRect != 0);
    int maxBoxNum = 4;
    double boxlength = 0.5;
    
	dxBox* colliding_box[maxBoxNum];
	for(int i=0; i<4; i++) colliding_box[i] = new dxBox (0,1,1,1);   //TODE space pointer has to be added

    for (unsigned int k = 0; k < numRect; k++)
    {
		MlsFieldRectangular * const colRect = &tempRectangularBuffer[k];

		for(unsigned int i=0;i<4;i++){   //we need to build 4 boxs per a rect
		   //set positions and size of Boxs A,B,C,D from collision
		   dVector3Copy(colRect->vertices[i]->vertex, colliding_box[i]->final_posr->pos); 
		   colliding_box[i]->side[0] = colRect->vertices[1]->vertex[0]-colRect->vertices[0]->vertex[0];				   
		   colliding_box[i]->side[1] = boxlength;	
		   colliding_box[i]->side[2] = colRect->vertices[2]->vertex[2]-colRect->vertices[0]->vertex[2];	
		   colliding_box[i]->final_posr->pos[0] -= (colliding_box[i]->side[0]/2);   
		   colliding_box[i]->final_posr->pos[1] = colliding_box[i]->final_posr->pos[1] - boxlength/2;  	
		   colliding_box[i]->final_posr->pos[2] -= (colliding_box[i]->side[2]/2); 			   		   	   
		   int collided = dCollideSphereBox (o2, colliding_box[i], flags, BoxContact, skip);

		   if(collided && numTerrainContacts < 4) {	   
  		       pContact = CONTACT(contact, numTerrainContacts*skip);			   
		       dVector3Copy(BoxContact->pos, pContact->pos);
               //create contact using Plane Normal
               dOPESIGN(pContact->normal, =, -, BoxContact->normal);	       
		       pContact->depth = BoxContact->depth;	
       
 	           numTerrainContacts++;	
		   }
		}

	}
	for(int i=0; i<maxBoxNum; i++) delete colliding_box[i];   
	return numTerrainContacts;  
}

int dCollideMlsfield( dxGeom *o1, dxGeom *o2, int flags, dContactGeom* contact, int skip )
{
    dIASSERT( skip >= (int)sizeof(dContactGeom) );
    //dIASSERT( o1->type == dMlsfieldClass );
    dIASSERT((flags & NUMC_MASK) >= 1);
    
    int i;

    // if ((flags & NUMC_MASK) == 0) -- An assertion check is made on entry
    //	{ flags = (flags & ~NUMC_MASK) | 1; dIASSERT((1 & ~NUMC_MASK) == 0); }

    int numMaxTerrainContacts = (flags & NUMC_MASK);

 //int numMaxTerrainContacts = 30;

    dxMlsfield *terrain = (dxMlsfield*) o1;

    dVector3 posbak;
    dMatrix3 Rbak;
    dReal aabbbak[6];
    int gflagsbak;
    dVector3 pos0,pos1;
    dMatrix3 R1;

    int numTerrainContacts = 0;
    int numTerrainOrigContacts = 0;

    //@@ Should find a way to set reComputeAABB to false in default case
    // aka DHEIGHTFIELD_CORNER_ORIGIN not defined and terrain not PLACEABLE
    // so that we can free some memory and speed up things a bit
    // while saving some precision loss 
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
    const bool reComputeAABB = true;
#else
    const bool reComputeAABB = ( terrain->gflags & GEOM_PLACEABLE ) ? true : false;
#endif //DHEIGHTFIELD_CORNER_ORIGIN

    //
    // Transform O2 into Heightfield Space
    //
    
    if (reComputeAABB)
    {
        // Backup original o2 position, rotation and AABB.
        dVector3Copy( o2->final_posr->pos, posbak );
        dMatrix3Copy( o2->final_posr->R, Rbak );
        memcpy( aabbbak, o2->aabb, sizeof( dReal ) * 6 );
        gflagsbak = o2->gflags;
    }

    if ( terrain->gflags & GEOM_PLACEABLE )
    {
        // Transform o2 into Mlsfield space.
        dSubtractVectors3( pos0, o2->final_posr->pos, terrain->final_posr->pos );
        dMultiply1_331( pos1, terrain->final_posr->R, pos0 );
        dMultiply1_333( R1, terrain->final_posr->R, o2->final_posr->R );

        // Update o2 with transformed position and rotation.
        dVector3Copy( pos1, o2->final_posr->pos );
        dMatrix3Copy( R1, o2->final_posr->R );
    }

#ifndef DHEIGHTFIELD_CORNER_ORIGIN
    o2->final_posr->pos[ 0 ] += terrain->m_p_data->m_fHalfWidth;
    o2->final_posr->pos[ 2 ] += terrain->m_p_data->m_fHalfDepth;
#endif // DHEIGHTFIELD_CORNER_ORIGIN


    // Rebuild AABB for O2
    if (reComputeAABB)
        o2->computeAABB();

    //
    // Collide
    //

    //check if inside boundaries
    // using O2 aabb
    //  aabb[6] is (minx, maxx, miny, maxy, minz, maxz) 
    const bool wrapped = terrain->m_p_data->m_bWrapMode != 0;

    if ( !wrapped )
    {
        if (    o2->aabb[0] > terrain->m_p_data->m_fWidth //MinX
            ||  o2->aabb[4] > terrain->m_p_data->m_fDepth)//MinZ
            goto dCollideMlsfieldExit;

        if (    o2->aabb[1] < 0 //MaxX
            ||  o2->aabb[5] < 0)//MaxZ
            goto dCollideMlsfieldExit;
    }
    { // To narrow scope of following variables
        const dReal fInvSampleWidth = terrain->m_p_data->m_fInvSampleWidth;
        int nMinX = (int)dFloor(dNextAfter(o2->aabb[0] * fInvSampleWidth, -dInfinity));
        int nMaxX = (int)dCeil(dNextAfter(o2->aabb[1] * fInvSampleWidth, dInfinity));
        const dReal fInvSampleDepth = terrain->m_p_data->m_fInvSampleDepth;
        int nMinZ = (int)dFloor(dNextAfter(o2->aabb[4] * fInvSampleDepth, -dInfinity));
        int nMaxZ = (int)dCeil(dNextAfter(o2->aabb[5] * fInvSampleDepth, dInfinity));

        if ( !wrapped )
        {
            nMinX = dMAX( nMinX, 0 );
            nMaxX = dMIN( nMaxX, terrain->m_p_data->m_nWidthSamples - 1);  //select overlabing area between o1 and o2
            nMinZ = dMAX( nMinZ, 0 );
            nMaxZ = dMIN( nMaxZ, terrain->m_p_data->m_nDepthSamples - 1);

            dIASSERT ((nMinX < nMaxX) && (nMinZ < nMaxZ));
        }

        numTerrainOrigContacts = numTerrainContacts;
        numTerrainContacts += terrain->dCollideMlsfieldZone(
            nMinX,nMaxX,nMinZ,nMaxZ,o2,numMaxTerrainContacts - numTerrainContacts,
            flags,CONTACT(contact,numTerrainContacts*skip),skip	);
      
        dIASSERT( numTerrainContacts <= numMaxTerrainContacts );
    }
    

    dContactGeom *pContact;
    for ( i = numTerrainOrigContacts; i != numTerrainContacts; ++i )
    {
        pContact = CONTACT(contact,i*skip);
        pContact->g1 = o1;
        pContact->g2 = o2;
         pContact->side1 = -1; //-- Oleh_Derevenko: sides must not be erased here as they are set by respective colliders during ray/plane tests 
         pContact->side2 = -1;
    }

  
    //------------------------------------------------------------------------------

dCollideMlsfieldExit:

    if (reComputeAABB)
    {
        // Restore o2 position, rotation and AABB
        dVector3Copy( posbak, o2->final_posr->pos );
        dMatrix3Copy( Rbak, o2->final_posr->R );
        memcpy( o2->aabb, aabbbak, sizeof(dReal)*6 );
        o2->gflags = gflagsbak;

        //
        // Transform Contacts to World Space
        //
        if ( terrain->gflags & GEOM_PLACEABLE )
        {
            for ( i = 0; i < numTerrainContacts; ++i )
            {
                pContact = CONTACT(contact,i*skip);
                dCopyVector3( pos0, pContact->pos );

#ifndef DHEIGHTFIELD_CORNER_ORIGIN
                pos0[ 0 ] -= terrain->m_p_data->m_fHalfWidth;
                pos0[ 2 ] -= terrain->m_p_data->m_fHalfDepth;
#endif // !DHEIGHTFIELD_CORNER_ORIGIN

                dMultiply0_331( pContact->pos, terrain->final_posr->R, pos0 );

                dAddVectors3( pContact->pos, pContact->pos, terrain->final_posr->pos );
                dCopyVector3( pos0, pContact->normal );

                dMultiply0_331( pContact->normal, terrain->final_posr->R, pos0 );
            }
        }
#ifndef DHEIGHTFIELD_CORNER_ORIGIN
        else
        {
            for ( i = 0; i < numTerrainContacts; ++i )
            {
                pContact = CONTACT(contact,i*skip);
                pContact->pos[ 0 ] -= terrain->m_p_data->m_fHalfWidth;
                pContact->pos[ 2 ] -= terrain->m_p_data->m_fHalfDepth;
            }
        }
#endif // !DHEIGHTFIELD_CORNER_ORIGIN
    }
    // Return contact count.
    return numTerrainContacts;
}



